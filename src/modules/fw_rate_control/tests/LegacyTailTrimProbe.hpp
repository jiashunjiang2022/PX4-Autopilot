#pragma once
// TDD migration adapter only: real V3 raw-I update, NOT a physical reference controller.
// Missing physical semantics are deliberately NOT implemented here.
#ifndef TAIL_TRIM_V3_FIXTURES_INCLUDED
#include "BumplessRollITransferTest.cpp"
#endif

class LegacyTailTrimProbe
{
public:

	struct Config {
		float b_max{0.f};
		float b_slew{0.f};
		float g_safe_min{0.f};
		float reserve_pos_min{0.f};
		float reserve_neg_min{0.f};
		float k_a{1.1f};
	};
	struct Inputs {
		float dt{0.f};
		float g_cycle{0.f};
		float tail_pitch_context{0.f};
		float current_native_i{0.f};
		float native_i_min{0.f};
		float native_i_max{0.f};
		float target_b{0.f};
		bool gate{false};
		bool learning_allowed{false};
		bool maneuver_active{false};
		bool safety_release_required{false};
		bool reversal_unwind_latched{false};
		bool input_valid{true};
		bool reset{false};
		uint32_t reset_epoch{0};
	};
	struct Result {
		float b_before{0.f}, b_after{0.f};
		float requested_delta_b{0.f}, accepted_delta_b{0.f};
		float requested_delta_i{0.f}, accepted_delta_i{0.f};
		float g_used{0.f}, transfer_mismatch{0.f}, tau_trim{0.f};
		float roll_reserve_pos{0.f}, roll_reserve_neg{0.f}, roll_reserve_sym{0.f};
		bool limited_by_bmax{false}, limited_by_shared_axis{false};
		bool limited_by_i{false}, limited_by_slew{false};
		bool growth_blocked_by_gate{false}, growth_blocked_by_maneuver{false};
		bool safety_release_active{false}, reversal_unwind_active{false};
		bool reversal_reached_zero{false}, transaction_valid{false};
		bool feasible{false}, recovery_required{false}, reset_applied{false};
	};

	LegacyTailTrimProbe(Config config, float b = 0.f, uint32_t epoch = 0) : _config(config) {
		(void)epoch;
		configure(_rc);
		BumplessRollITransferTestAccess::configureAdaptiveHold(_transfer, b, _rc.rollIntegralResetEpoch(), b, 0.f);
	}
	float state() const { return _transfer.transferredRaw(); }
	Result step(Inputs in) {
		Result r{};
		r.b_before = state();
		set_residual(_rc, in.current_native_i, state());
		auto old = enabled_inputs(in.dt);
		old.g_current = in.g_cycle;
		old.cap_raw = _config.b_max;
		set_adaptive_defaults(old, 0);
		old.hold_cap_raw = _config.b_max;
		old.adapt_slew_raw_per_s = _config.b_slew;
		old.safety_slew_raw_per_s = _config.b_slew;
		old.failsafe = in.safety_release_required;
		// Express target through existing raw estimator. Maneuver/physical pitch have
		// no V3 Inputs fields; no fake policy is inserted by this adapter.
		BumplessRollITransferTestAccess::setAdaptiveTHat(_transfer,
			in.target_b + copysignf(old.residual_reserve_raw, in.target_b), true);
		const auto out = _transfer.update(old, _rc);
		r.b_after = out.transferred_i_raw;
		r.accepted_delta_b = out.accepted_delta_s_raw;
		r.accepted_delta_i = out.accepted_delta_i_raw;
		r.requested_delta_b = out.requested_delta_s_raw;
		r.requested_delta_i = out.requested_delta_i_raw;
		r.g_used = out.g_current;
		r.tau_trim = out.effective_slow_torque;
		r.transfer_mismatch = in.g_cycle*r.accepted_delta_i + _config.k_a*r.accepted_delta_b;
		// Expose actual legacy raw headroom outputs to test the proposed
		// semantic migration. These are NOT physical directional reserves.
		r.roll_reserve_pos = out.headroom_positive_raw;
		r.roll_reserve_neg = out.headroom_negative_raw;
		r.roll_reserve_sym = fminf(r.roll_reserve_pos, r.roll_reserve_neg);
		r.transaction_valid = !out.recovery;
		r.safety_release_active = out.safety_exit;
		return r;
	}
private:
	Config _config;
	RateControl _rc;
	BumplessRollITransfer _transfer;
};
