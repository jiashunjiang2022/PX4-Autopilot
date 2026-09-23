#pragma once
#include <cmath>

// Normalized pre-allocation request model, NOT servo-position feedback.
class FastActuatorMargin
{
public:
	static constexpr float EPS = 2e-6f;
	struct Result {
		bool valid{}, baseline_feasible{}, blocked{true};
		float baseline_roll{}, baseline_pitch{}, tail0_baseline{}, tail1_baseline{};
		float roll_lower{}, roll_upper{}, roll_headroom_negative{}, roll_headroom_positive{};
		float fast_lower{}, fast_upper{};
	};
	static bool zero(float value)
	{
		return std::isfinite(value) && std::fabs(value) <= 0.f;
	}
	static bool configuration(bool geometry, float flap0, float flap1, float spoil0, float spoil1,
				  int rotors, float rotor_y, float rotor_z, float slew0, float slew1)
	{
		return geometry && zero(flap0) && zero(flap1) && zero(spoil0) && zero(spoil1)
		       && (rotors == 0 || (rotors == 1 && zero(rotor_y) && zero(rotor_z)))
		       && zero(slew0) && zero(slew1);
	}
	static float tail0(float r, float p)
	{
		return .5f * p - r / 1.1f;
	}
	static float tail1(float r, float p)
	{
		return .5f * p + r / 1.1f;
	}
	static bool postcheck(float r, float p)
	{
		return std::isfinite(r) && std::isfinite(p) && std::fabs(r) <= 1.f + EPS
		       && std::fabs(p) <= 1.f + EPS && std::fabs(tail0(r, p)) <= 1.f + EPS
		       && std::fabs(tail1(r, p)) <= 1.f + EPS;
	}
	static Result compute(float r, float p, float scale)
	{
		Result out{};
		out.baseline_roll = r;
		out.baseline_pitch = p;
		if (!std::isfinite(r) || !std::isfinite(p) || !std::isfinite(scale) || scale <= 0.f) {
			return out;
		}
		out.valid = true;
		out.tail0_baseline = tail0(r, p);
		out.tail1_baseline = tail1(r, p);
		const float cap = 1.1f * (1.f - .5f * std::fabs(p));
		const float upper = cap < 1.f ? cap : 1.f;
		// Strict baseline acceptance: never repair an already infeasible request.
		if (std::fabs(p) > 1.f || std::fabs(r) > upper
		    || std::fabs(out.tail0_baseline) > 1.f || std::fabs(out.tail1_baseline) > 1.f) {
			return out;
		}
		out.roll_lower = -upper;
		out.roll_upper = upper;
		out.roll_headroom_negative = r + upper;
		out.roll_headroom_positive = upper - r;
		out.fast_lower = -out.roll_headroom_negative / scale;
		out.fast_upper = out.roll_headroom_positive / scale;
		if (!std::isfinite(out.fast_lower) || !std::isfinite(out.fast_upper)) {
			out.valid = false;
			out.fast_lower = out.fast_upper = 0.f;
			return out;
		}
		out.baseline_feasible = true;
		out.blocked = false;
		return out;
	}
};
