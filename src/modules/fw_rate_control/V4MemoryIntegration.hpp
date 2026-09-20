// SPDX-License-Identifier: BSD-3-Clause
#pragma once
#include "ResidualSlowFeedforwardMemory.hpp"
#include "BumplessRollITransfer.hpp"
#include <uORB/topics/flap_v4_memory_status.h>

// Single FW controller thread only. No estimator law lives in this adapter.
class V4MemoryIntegration final
{
public:
	using Config = ResidualSlowFeedforwardMemory::Config;
	struct Context {
		uint64_t now{}, evidence_timestamp{};
		float dt{};
		bool enabled{}, armed{}, landed{}, mission{}, failsafe{}, pilot_abort{};
		bool control_valid{true}, hard_reset{}, stable{true}, maneuver{};
		uint8_t nav_state{};
	};
	V4MemoryIntegration(RateControl &native, BumplessRollITransfer &v3) :
		_native(native), _v3(v3), _core(native) {}
	bool active() const { return _active; }
	// No armed S->B migration. Acquisition is deliberately restricted to disarm.
	void select(bool enable, bool armed, bool supported)
	{
		if (_active && !enable && fabsf(_core.result().b) <= 0.f) {
			_active = false;
			_v3.synchronizeReset(_native.rollIntegralResetEpoch());
			_native.setRollITransferContext(0.f, 0.f, false);
		}
		if (!_active && enable && !armed && supported) {
			_native.resetIntegral(0);
			_v3.synchronizeReset(_native.rollIntegralResetEpoch());
			_active = true;
		}
	}
	void allocator(uint64_t timestamp, float unallocated_roll)
	{
		_allocator_time = timestamp;
		_allocator_roll = unallocated_roll;
	}
	void before(const Context &ctx, const Config &config)
	{
		_natural_recorded = false;
		_status = {};
		_status.timestamp = ctx.now;
		_status.enabled = ctx.enabled;
		_status.active = _active;
		_status.enable_pending = ctx.enabled && !_active;
		_status.nav_state = ctx.nav_state;
		if (!_active) { return; }
		_status.i_pre_transfer_raw = _native.rollIntegralRaw();
		_status.b_pre_raw = _core.result().b;
		_status.t_pre_transfer_raw = _status.i_pre_transfer_raw + _status.b_pre_raw;
		_status.allocator_fresh = _allocator_time > 0 && _allocator_time <= ctx.now
			&& ctx.now - _allocator_time <= 100000 && PX4_ISFINITE(_allocator_roll);
		_status.allocator_pos_sat = _status.allocator_fresh && _allocator_roll > FLT_EPSILON;
		_status.allocator_neg_sat = _status.allocator_fresh && _allocator_roll < -FLT_EPSILON;
		const uint64_t previous_evidence = _core.result().evidence_time;
		_status.evidence_gap_reset = previous_evidence > 0 && ctx.now > previous_evidence
			&& ctx.now - previous_evidence > ResidualSlowFeedforwardMemory::MaxGap;
		// Decimate actual sensor timestamps to nominal 20 Hz, never fabricate time.
		const bool fresh = ctx.evidence_timestamp > 0 && ctx.evidence_timestamp <= ctx.now
			&& (_last_offered == 0 || (ctx.evidence_timestamp > _last_offered
			    && ctx.evidence_timestamp - _last_offered >= 50000));
		_status.evidence_fresh = fresh && ctx.now - ctx.evidence_timestamp <= 150000;
		_status.evidence_age_ms = ctx.evidence_timestamp > 0 && ctx.evidence_timestamp <= ctx.now
			? (ctx.now - ctx.evidence_timestamp) * .001f : -1.f;
		if (fresh) { _last_offered = ctx.evidence_timestamp; }
		ResidualSlowFeedforwardMemory::Inputs in{};
		in.now = ctx.now;
		in.timestamp = ctx.evidence_timestamp;
		in.control_dt = ctx.dt;
		in.fresh_sample = _status.evidence_fresh;
		in.allocator_fresh = _status.allocator_fresh;
		in.positive_saturation = _status.allocator_pos_sat;
		in.negative_saturation = _status.allocator_neg_sat;
		in.enabled = ctx.enabled;
		in.mission = ctx.mission;
		in.stable = ctx.stable;
		in.maneuver = ctx.maneuver;
		in.pilot_abort = ctx.pilot_abort;
		in.failsafe = ctx.failsafe;
		in.hard_reset = ctx.hard_reset || ctx.landed || !ctx.armed;
		in.control_valid = ctx.control_valid;
		// A configure recovery must remain visible; do not overwrite it with update.
		const uint32_t epoch_before = _native.rollIntegralResetEpoch();
		const bool valid = _core.configure(config);
		if (valid) { _bmax = config.bmax; }
		const auto result = !valid || epoch_before != _native.rollIntegralResetEpoch()
			? _core.result() : _core.update(in);
		record(result);
	}
	void after(float frozen_g)
	{
		if (!_active) { return; }
		_status.i_post_natural_raw = _native.rollIntegralRaw();
		_status.t_post_natural_raw = _status.i_post_natural_raw + _status.b_raw;
		_status.b_after_gain = frozen_g * _status.b_raw;
		_status.i_cycle_end_raw = _status.i_post_natural_raw;
		_status.b_cycle_end_raw = _core.result().b;
		_natural_recorded = true;
	}
	void reset()
	{
		if (_active) {
			ResidualSlowFeedforwardMemory::Inputs in{};
			in.hard_reset = true;
			const auto r = _core.update(in);
			if (!_natural_recorded) {
				record(r);
			} else {
				// Output validation reset occurs AFTER these snapshots. Keep their time meaning.
				_status.output_recovery = true;
				_status.recovery_active = r.recovery;
				_status.recovery_reason = static_cast<uint8_t>(r.reason);
				_status.reset_epoch = r.epoch;
				_status.apply_valid = false;
				_status.learn_valid = false;
				_status.window_valid = false;
			}
			_status.i_cycle_end_raw = r.i;
			_status.b_cycle_end_raw = r.b;
		}
	}
	float composeRaw(float raw) const { return _core.composeRaw(raw); }
	const flap_v4_memory_status_s &status() const { return _status; }

private:
	friend struct V4IntegrationTestAccess;
	void record(const ResidualSlowFeedforwardMemory::Result &r)
	{
		_status.b_raw = r.b;
		_status.q_hat_raw = r.q_hat;
		_status.r_hat_raw = r.residual;
		_status.innovation_s_raw = r.innovation;
		_status.i_post_transfer_raw = r.i;
		_status.t_post_transfer_raw = r.i + r.b;
		_status.i_post_natural_raw = r.i;
		_status.t_post_natural_raw = r.i + r.b;
		_status.i_cycle_end_raw = r.i;
		_status.b_cycle_end_raw = r.b;
		_status.requested_delta_b_raw = r.requested;
		_status.accepted_delta_b_raw = r.accepted;
		_status.learn_valid = r.learn;
		_status.apply_valid = r.apply;
		_status.window_valid = r.gate_valid;
		_status.window_span_s = r.samples > 0 ? (r.evidence_time - r.gate_time) * 1e-6f : 0.f;
		_status.window_sample_count = r.samples;
		_status.window_std = r.stddev;
		_status.window_sign_fraction = r.same_sign_fraction;
		_status.handback_active = r.handback;
		_status.recovery_active = r.recovery;
		_status.recovery_reason = static_cast<uint8_t>(r.reason);
		_status.reset_epoch = r.epoch;
		_status.imax = _native.rollIntegralLimit();
		_status.bmax = _bmax;
		_status.b_at_cap = fabsf(r.b) >= _bmax;
	}
	RateControl &_native;
	BumplessRollITransfer &_v3;
	ResidualSlowFeedforwardMemory _core;
	flap_v4_memory_status_s _status{};
	uint64_t _allocator_time{}, _last_offered{};
	float _allocator_roll{}, _bmax{.1f};
	bool _active{};
	bool _natural_recorded{};
};
