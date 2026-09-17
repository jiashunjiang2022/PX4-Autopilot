#pragma once

#include <cmath>
#include <cstdint>
#include <float.h>

// Unintegrated value-snapshot core. No RateControl mutation, uORB or actuator access.
// Caller must apply accepted I and b as one transaction at eventual integration.
class AdaptiveTailTrimCore final
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

	explicit AdaptiveTailTrimCore(Config config, float initial_b = 0.f, uint32_t epoch = 0) :
		_config(config), _epoch(epoch)
	{
		_initial_valid = validConfig() && finite(initial_b) && std::fabs(initial_b) <= _config.b_max;
		_b = _initial_valid ? initial_b : 0.f;
	}

	float state() const { return _b; }

	// By-value input: no external gain reference can change the transaction.
	Result step(Inputs in)
	{
		Result out{};
		out.b_before = out.b_after = _b;
		out.g_used = finite(in.g_cycle) ? in.g_cycle : 0.f;
		out.tau_trim = validConfig() ? _config.k_a * _b : 0.f;

		if (in.reset || in.reset_epoch != _epoch) {
			_b = 0.f;
			_epoch = in.reset_epoch;
			_unwind = _await_fresh_gate = _safety_exit = false;
			_initial_valid = validConfig();
			out.b_after = out.tau_trim = 0.f;
			out.reset_applied = true;
			out.recovery_required = true;
			return out;
		}

		if (!_initial_valid || !validConfig() || !in.input_valid
		    || !finite(in.dt) || in.dt <= 0.f
		    || !finite(in.g_cycle) || in.g_cycle <= _config.g_safe_min
		    || !finite(in.tail_pitch_context) || !finite(in.target_b)
		    || !finite(in.current_native_i) || !finite(in.native_i_min) || !finite(in.native_i_max)
		    || in.native_i_min > in.native_i_max
		    || in.current_native_i < in.native_i_min || in.current_native_i > in.native_i_max) {
			out.recovery_required = true;
			return out;
		}

		const double g = static_cast<double>(in.g_cycle);
		const double ka = static_cast<double>(_config.k_a);
		const double before = static_cast<double>(_b);
		const double room = 1. - std::fabs(static_cast<double>(in.tail_pitch_context));
		const double lower = fmax(-static_cast<double>(_config.b_max),
					 static_cast<double>(_config.reserve_neg_min) - room);
		const double upper = fmin(static_cast<double>(_config.b_max),
					 room - static_cast<double>(_config.reserve_pos_min));
		const bool shared_valid = lower <= upper && before >= lower && before <= upper;
		const bool trusted = in.learning_allowed && !in.maneuver_active;
		const double native_i = static_cast<double>(in.current_native_i);
		const double i_min_delta = static_cast<double>(in.native_i_min) - native_i;
		const double i_max_delta = static_cast<double>(in.native_i_max) - native_i;
		const double max_step = static_cast<double>(_config.b_slew) * static_cast<double>(in.dt);

		bool unwind = _unwind;
		bool await_gate = _await_fresh_gate;
		bool safety = _safety_exit || in.safety_release_required || !shared_valid;
		double target = clamp(static_cast<double>(in.target_b),
				      -static_cast<double>(_config.b_max), static_cast<double>(_config.b_max));
		out.limited_by_bmax = std::fabs(in.target_b) > _config.b_max;
		out.limited_by_shared_axis = !shared_valid;

		if (trusted && await_gate && !in.gate && !in.reversal_unwind_latched) { await_gate = false; }

		// Only a trusted window may establish a new reversal; no evidence is accumulated here.
		if (trusted && !await_gate && in.reversal_unwind_latched && opposite(before, target)) { unwind = true; }

		out.safety_release_active = safety;
		out.reversal_unwind_active = unwind;

		if (safety || unwind) {
			target = 0.;

		} else if (!trusted) {
			out.growth_blocked_by_maneuver = true;
			target = before;

		} else if (await_gate || in.reversal_unwind_latched || opposite(before, target)) {
			// Opposite target without a trusted latch, or stale evidence after zero.
			target = before;
			out.growth_blocked_by_gate = true;

		} else if (!in.gate && std::fabs(target) > std::fabs(before)) {
			target = before;
			out.growth_blocked_by_gate = true;
		}

		const double request = target - before;
		const double request_i = -ka * request / g;

		if (!representable(request_i)) { out.recovery_required = true; return out; }

		out.requested_delta_b = static_cast<float>(request);
		out.requested_delta_i = static_cast<float>(request_i);

		if (!safety && !unwind) {
			const double admitted = clamp(target, lower, upper);
			out.limited_by_shared_axis = out.limited_by_shared_axis || target < lower || target > upper;
			target = admitted;
		}

		const double desired_step = target - before;
		double db = clamp(desired_step, -max_step, max_step);
		out.limited_by_slew = std::fabs(desired_step) > max_step;

		if (!safety) {
			const double limited = clamp(db, -g * i_max_delta / ka, -g * i_min_delta / ka);
			out.limited_by_i = limited < db || limited > db;
			db = limited;
		}

		float after = static_cast<float>(before + db);
		double actual_db = static_cast<double>(after) - before;
		double di = 0.;
		// Round inward at float endpoints. Fixed iteration count; never violate
		// bounds to claim a normal conserved transaction.
		bool accepted = false;

		for (int n = 0; n < 4; ++n) {
			actual_db = static_cast<double>(after) - before;
			di = -ka * actual_db / g;

			if (safety) {
				const double limited = clamp(di, i_min_delta, i_max_delta);
				out.limited_by_i = out.limited_by_i || limited < di || limited > di;
				di = limited;
			}

			if (!representable(di)) { break; }

			float di_float = static_cast<float>(di);

			if (static_cast<double>(di_float) < i_min_delta || static_cast<double>(di_float) > i_max_delta) {
				di_float = nextafterf(di_float, 0.f);
			}

			di = static_cast<double>(di_float);
			const bool shared_ok = safety || unwind || (static_cast<double>(after) >= lower
					       && static_cast<double>(after) <= upper);
			const bool toward_target = actual_db * desired_step >= 0.
						   && std::fabs(actual_db) <= std::fabs(desired_step);
			const bool bounded = std::fabs(static_cast<double>(after)) <= static_cast<double>(_config.b_max)
					     && std::fabs(actual_db) <= max_step && toward_target && shared_ok
					     && di >= i_min_delta && di <= i_max_delta;
			const double mismatch = g * di + ka * actual_db;

			if (bounded && std::isfinite(mismatch) && (safety || std::fabs(mismatch) <= 2e-6)) {
				accepted = true;
				break;
			}

			after = nextafterf(after, _b);
		}

		if (!accepted) { out.recovery_required = true; return out; }

		out.b_after = after;
		out.accepted_delta_b = after - _b;
		out.accepted_delta_i = static_cast<float>(di);
		out.transfer_mismatch = static_cast<float>(g * di + ka * static_cast<double>(out.accepted_delta_b));
		out.tau_trim = _config.k_a * after;
		out.roll_reserve_pos = static_cast<float>(room - static_cast<double>(after));
		out.roll_reserve_neg = static_cast<float>(room + static_cast<double>(after));
		out.roll_reserve_sym = fminf(out.roll_reserve_pos, out.roll_reserve_neg);
		out.feasible = room - static_cast<double>(after) >= static_cast<double>(_config.reserve_pos_min)
			       && room + static_cast<double>(after) >= static_cast<double>(_config.reserve_neg_min);
		out.transaction_valid = true;

		if (unwind && std::fabs(after) <= 0.f) {
			out.reversal_reached_zero = std::fabs(_b) > 0.f;
			unwind = false;
			await_gate = true;
		}

		_b = after;
		_unwind = unwind;
		_await_fresh_gate = await_gate;
		_safety_exit = safety && std::fabs(after) > 0.f;
		return out;
	}

private:
	static bool finite(float x) { return std::isfinite(x); }
	static bool representable(double x) {
		return std::isfinite(x) && std::fabs(x) <= static_cast<double>(FLT_MAX);
	}
	static double clamp(double x, double lo, double hi) { return fmax(lo, fmin(x, hi)); }
	static bool opposite(double a, double b) { return (a > 0. && b < 0.) || (a < 0. && b > 0.); }
	bool validConfig() const {
		return finite(_config.b_max) && _config.b_max >= 0.f && _config.b_max <= .1f
		       && finite(_config.b_slew) && _config.b_slew > 0.f
		       && finite(_config.g_safe_min) && _config.g_safe_min > 0.f
		       && finite(_config.reserve_pos_min) && _config.reserve_pos_min > 0.f && _config.reserve_pos_min <= 1.f
		       && finite(_config.reserve_neg_min) && _config.reserve_neg_min > 0.f && _config.reserve_neg_min <= 1.f
		       && finite(_config.k_a) && std::fabs(_config.k_a - 1.1f) <= 1e-6f;
	}
	const Config _config;
	float _b{0.f};
	uint32_t _epoch{0};
	bool _initial_valid{false};
	bool _unwind{false};
	bool _await_fresh_gate{false};
	bool _safety_exit{false};
};
