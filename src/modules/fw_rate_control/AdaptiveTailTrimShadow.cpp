#include "AdaptiveTailTrimShadow.hpp"
#include <new>

namespace {
bool positive(float x) { return std::isfinite(x) && x > 0.f; }
bool opposite(float x, float y) { return (x > 0.f && y < 0.f) || (x < 0.f && y > 0.f); }
bool validDt(float dt) { return positive(dt) && dt <= .05f; }
}

void AdaptiveTailTrimEstimator::reset()
{
	// Ring contents are inaccessible until overwritten; no large stack temporary.
	_hat = _elapsed = _reference = 0.f;
	_head = _count = 0;
	_initialized = false;
	_result = {};
}

AdaptiveTailTrimEstimator::Result AdaptiveTailTrimEstimator::update(float b, float dt, bool valid,
		bool learning, Config cfg)
{
	if (!valid || !learning || !std::isfinite(b) || !validDt(dt) || !positive(cfg.tau)
	    || !positive(cfg.window) || cfg.window > 10.f || !std::isfinite(cfg.std_limit) || cfg.std_limit < 0.f
	    || !std::isfinite(cfg.sign_fraction) || cfg.sign_fraction < 0.f || cfg.sign_fraction > 1.f) {
		reset();
		return _result;
	}

	_hat = _initialized ? _hat + dt / (cfg.tau + dt) * (b - _hat) : b;
	_initialized = true;

	if (opposite(_hat, _reference)) {
		_head = _count = 0;
		_elapsed = 0.f;
		_result = {};
	}

	if (fabsf(_hat) > 1e-7f) { _reference = _hat; }

	_elapsed += dt;
	const auto required = static_cast<uint16_t>(ceilf(cfg.window * 20.f));

	if (_elapsed + 1e-6f >= .05f) {
		_elapsed = fmaxf(0.f, _elapsed - .05f);
		_samples[_head] = b;
		_head = (_head + 1) % 200;
		if (_count < required) { ++_count; }
		double sum = 0., sum_sq = 0.;
		unsigned same = 0;

		for (unsigned n = 0; n < _count; ++n) {
			const float value = _samples[(_head + 200 - _count + n) % 200];
			const double x = static_cast<double>(value);
			sum += x;
			sum_sq += x * x;
			if (value * _hat > 0.f) { ++same; }
		}

		const double count = static_cast<double>(_count);
		_result.std = static_cast<float>(sqrt(fmax(0., sum_sq / count - (sum / count) * (sum / count))));
		_result.same_sign_fraction = static_cast<float>(same) / static_cast<float>(_count);
	}

	_result.b_hat = _hat;
	_result.sample_count = _count;
	_result.valid = true;
	_result.gate = _count >= required && _result.std <= cfg.std_limit
		       && _result.same_sign_fraction >= cfg.sign_fraction;
	return _result;
}

bool AdaptiveTailTrimManeuverGate::update(float phi, float p, float dt, bool valid)
{
	if (!valid || !std::isfinite(phi) || !std::isfinite(p) || !validDt(dt)
	    || fabsf(phi) > Enter || fabsf(p) > Enter) {
		_active = true;
		_exit_elapsed = 0.f;

	} else if (_active) {
		if (fabsf(phi) < Exit && fabsf(p) < Exit) {
			_exit_elapsed += dt;
			if (_exit_elapsed + 1e-6f >= Hold) { _active = false; _exit_elapsed = 0.f; }

		} else {
			_exit_elapsed = 0.f;
		}
	}

	return _active;
}

CausalTailPitchEnvelope::Result CausalTailPitchEnvelope::update(float left, float right, uint64_t stamp,
		uint64_t now, float dt, bool mapping)
{
	Result r{};
	if (!mapping) { r.invalid |= Mapping; }
	if (!std::isfinite(left) || !std::isfinite(right)) { r.invalid |= Nonfinite; }
	if (fabsf(left) > 1.f || fabsf(right) > 1.f) { r.invalid |= Range; }
	if (stamp > now) { r.invalid |= Future; }
	if (stamp == 0 || (now >= stamp && now - stamp > FreshnessUs) || !validDt(dt)) { r.invalid |= Stale; }
	r.valid = r.invalid == 0;

	if (r.valid) {
		r.raw = .5f * (left + right);
		r.roll = .5f * (right - left);
		_envelope = fmaxf(fabsf(r.raw), _envelope * expf(-dt)); // provisional tau = 1 s
	}

	r.envelope = _envelope; // invalid: retain peak, do not claim current authority
	return r;
}

bool AdaptiveTailTrimShadow::validConfig() const
{
	return std::isfinite(_config.bmax) && _config.bmax >= 0.f && _config.bmax <= .1f
	       && std::isfinite(_config.reserve) && _config.reserve >= 0.f && _config.reserve <= 1.f
	       && positive(_config.slew) && _config.slew <= .1f
	       && positive(_config.roll_reserve) && _config.roll_reserve <= 1.f
	       && positive(_config.tau) && _config.tau <= 30.f
	       && _config.window >= .5f && _config.window <= 10.f
	       && std::isfinite(_config.std_raw) && _config.std_raw >= 0.f
	       && _config.sign_fraction >= .5f && _config.sign_fraction <= 1.f;
}

void AdaptiveTailTrimShadow::configure(Config c)
{
	_config = c;
	// Placement construction replaces only the small owned core, never allocates.
	_core.~AdaptiveTailTrimCore();
	new (&_core) AdaptiveTailTrimCore({c.bmax, c.slew, GMin, c.roll_reserve, c.roll_reserve, 1.1f});
	reset();
}

void AdaptiveTailTrimShadow::reset(uint32_t epoch)
{
	AdaptiveTailTrimCore::Inputs reset_input{};
	reset_input.reset = true;
	reset_input.reset_epoch = epoch;
	_core.step(reset_input);
	_epoch = epoch;
	_offset = _target = 0.f;
	_last_time = 0;
	_enabled = _reversal = _rearm = false;
	_estimator.reset();
	_maneuver.reset();
	_pitch.reset();
}

AdaptiveTailTrimShadow::Result AdaptiveTailTrimShadow::update(Inputs in)
{
	Result r{};
	r.enabled = in.enabled;
	r.actual_tail_trim_torque = 0.f;
	const float total = in.i_actual + in.s_actual;
	const bool numeric_valid = validDt(in.dt) && std::isfinite(in.g) && in.g > GMin
				   && std::isfinite(in.i_actual) && std::isfinite(in.s_actual) && std::isfinite(total)
				   && positive(in.imax) && std::isfinite(total + _offset)
				   && fabsf(total + _offset) <= in.imax;
	const bool time_valid = in.now > 0 && (_last_time == 0
				  || (in.now > _last_time && in.now - _last_time <= 50000));
	const bool reset_needed = !in.enabled || !_enabled || !in.armed || in.landed || !validConfig()
				 || !in.mapping_valid || in.epoch != _epoch || !numeric_valid || !time_valid;

	if (reset_needed) {
		reset(in.epoch);
		_enabled = in.enabled;
		_last_time = in.now;
		r.reset = true;
		// Distinguish mapping invalid even when it forces a complete shadow reset.
		if (!in.mapping_valid) { r.pitch.invalid = CausalTailPitchEnvelope::Mapping; }
		return r;
	}

	_last_time = in.now;
	r.g = in.g;
	r.i_actual = in.i_actual;
	r.s_actual = in.s_actual;
	r.t_actual = total;
	r.b_obs = in.g * total / 1.1f;

	if (!std::isfinite(r.b_obs)) { reset(in.epoch); r = {}; r.enabled = in.enabled; r.reset = true; return r; }

	r.maneuver = _maneuver.update(in.phi_sp, in.p_sp, in.dt, in.setpoint_valid);
	r.learning = in.control_valid && in.setpoint_valid && !in.safety && !r.maneuver;
	// Pitch is deliberately absent from detector validity and learning inputs.
	r.estimate = _estimator.update(r.b_obs, in.dt, in.control_valid, r.learning,
			{_config.tau, _config.window, in.g * _config.std_raw / 1.1f, _config.sign_fraction});
	r.pitch = _pitch.update(in.left, in.right, in.actuator_timestamp, in.now, in.dt, in.mapping_valid);

	if (r.learning && r.estimate.valid) {
		_target = copysignf(fminf(_config.bmax, fmaxf(fabsf(r.estimate.b_hat) - _config.reserve, 0.f)), r.estimate.b_hat);
		if (!_rearm && r.estimate.gate && opposite(_core.state(), _target)) { _reversal = true; }
	}

	AdaptiveTailTrimCore::Inputs core_input{};
	core_input.dt = in.dt;
	core_input.g_cycle = in.g;
	core_input.current_native_i = total + _offset;
	core_input.native_i_min = -in.imax;
	core_input.native_i_max = in.imax;
	core_input.tail_pitch_context = r.pitch.valid ? r.pitch.envelope : 1.f;
	core_input.target_b = _target;
	core_input.gate = r.estimate.gate && !_rearm;
	core_input.learning_allowed = r.learning && r.pitch.valid;
	core_input.maneuver_active = r.maneuver;
	core_input.safety_release_required = in.safety || !in.control_valid || !in.setpoint_valid || !r.pitch.valid;
	core_input.reversal_unwind_latched = _reversal;
	core_input.reset_epoch = _epoch;
	r.transfer = _core.step(core_input);
	_offset += r.transfer.accepted_delta_i;

	if (_rearm && core_input.learning_allowed) { _rearm = false; }

	if (r.transfer.reversal_reached_zero) {
		_estimator.reset();
		r.estimate = {};
		_reversal = false;
		_rearm = true;
	}

	r.target = _target;
	r.b_shadow = _core.state();
	r.offset = _offset;
	r.i_shadow = total + _offset;
	r.b_total = in.g * r.i_shadow / 1.1f + r.b_shadow;
	r.reversal = _reversal;
	r.valid = r.transfer.transaction_valid && r.pitch.valid && in.control_valid;
	return r;
}
