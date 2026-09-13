#pragma once

#include <cmath>
#include <cstdint>

class FixedB0Experiment final
{
public:
	enum class ExitReason : uint8_t {
		None = 0,
		NormalDisable = 1,
		ModeExit = 2,
		Failsafe = 3,
		ConfigInvalid = 4,
		RateControlDisabled = 5,
		InvalidParameter = 6,
		HardReset = 7
	};

	struct Inputs {
		bool enabled{false}; bool armed{false}; bool airborne{false}; bool fixed_wing{false};
		bool vtol{false}; bool tailsitter{false}; bool transition{false}; bool failsafe{false};
		bool rates_enabled{false}; bool supported_mode{false}; bool config_valid{false};
		float b0_tail{0.f}; float slew_tail_per_s{0.02f};
	};
	struct Result {
		bool gate_valid{false}; bool config_valid{false}; float target_tail{0.f}; float applied_tail{0.f};
		float internal_scale{1.1f}; float requested_torque{0.f}; float applied_torque{0.f}; bool total_clipped{false};
		ExitReason exit_reason{ExitReason::None};
	};
	static constexpr float InternalTorquePerTail = 1.1f;
	static constexpr float B0MaxTail = 0.22f;
	static constexpr float SlewMin = 0.005f;
	static constexpr float SlewMax = 0.05f;
	static constexpr float SafetyExitSlewTailPerS = 0.20f;

	Result update(const Inputs &in, float dt, float roll_baseline)
	{
		Result out{};
		out.config_valid = in.config_valid;
		const bool b0_valid = std::isfinite(in.b0_tail) && fabsf(in.b0_tail) <= B0MaxTail;
		const bool slew_valid = std::isfinite(in.slew_tail_per_s);
		const bool hard_reset = !in.armed || !in.airborne;
		const bool healthy = in.armed && in.airborne && in.fixed_wing && !in.vtol && !in.tailsitter
				    && !in.transition && !in.failsafe && in.rates_enabled && in.supported_mode
				    && in.config_valid && b0_valid && slew_valid;
		const bool gate = in.enabled && healthy;
		out.gate_valid = gate;

		if (hard_reset) {
			out.exit_reason = ExitReason::HardReset;
		} else if (gate) {
			out.exit_reason = ExitReason::None;
		} else if (!in.enabled && healthy) {
			out.exit_reason = ExitReason::NormalDisable;
		} else if (in.failsafe) {
			out.exit_reason = ExitReason::Failsafe;
		} else if (!in.rates_enabled) {
			out.exit_reason = ExitReason::RateControlDisabled;
		} else if (!in.config_valid || !in.fixed_wing || in.vtol || in.tailsitter || in.transition) {
			out.exit_reason = ExitReason::ConfigInvalid;
		} else if (!b0_valid || !slew_valid) {
			out.exit_reason = ExitReason::InvalidParameter;
		} else {
			out.exit_reason = ExitReason::ModeExit;
		}

		const float target = gate ? in.b0_tail : 0.f;
		out.target_tail = target;
		if (hard_reset || !std::isfinite(dt) || dt <= 0.f) {
			_applied_tail = hard_reset ? 0.f : _applied_tail;
		} else {
			const float slew = (out.exit_reason == ExitReason::None || out.exit_reason == ExitReason::NormalDisable)
					   ? constrain(in.slew_tail_per_s, SlewMin, SlewMax)
					   : SafetyExitSlewTailPerS;
			const float step = slew * dt;
			_applied_tail += constrain(target - _applied_tail, -step, step);
		}
		if (!std::isfinite(_applied_tail)) { _applied_tail = 0.f; }
		out.applied_tail = _applied_tail;
		out.requested_torque = target * InternalTorquePerTail;
		out.applied_torque = _applied_tail * InternalTorquePerTail;
		const float total = roll_baseline + out.applied_torque;
		out.total_clipped = std::isfinite(total) && (total < -1.f || total > 1.f);
		return out;
	}
	void reset() { _applied_tail = 0.f; }

private:
	static float constrain(float value, float lower, float upper) { return value < lower ? lower : (value > upper ? upper : value); }
	float _applied_tail{0.f};
};
