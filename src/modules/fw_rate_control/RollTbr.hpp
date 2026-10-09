#pragma once

#include <mathlib/math/filter/AlphaFilter.hpp>
#include <lib/slew_rate/SlewRate.hpp>
#include <cmath>
#include <cstdint>
#include <cstring>

namespace roll_tbr
{

enum class ResetReason : uint8_t {
	None = 0, Disabled, Disarmed, Unsupported, NotMission, Landed,
	InvalidDt, InvalidConfig, NonfiniteInput, ControllerInactive, IntegralReset, ConfigChanged
};

struct Config {
	float fc{0.625f};
	float k{0.01f};
	float max{0.025f};
	float slew{0.035031594f};
};

struct Input {
	int32_t enable{0};
	bool armed{false};
	bool supported{false};
	bool mission{false};
	bool landed{true};
	bool controller_active{false};
	bool inputs_valid{false};
	bool integral_reset{false};
	float dt{0.f};
	float measured_dt{0.f};
	float setpoint{0.f};
	float rate{0.f};
};

struct Result {
	float error{0.f};
	float filtered{0.f};
	float raw{0.f};
	float capped{0.f};
	float output{0.f};
	bool active{false};
	bool config_valid{false};
	bool cap_active{false};
	bool slew_active{false};
	ResetReason reset_reason{ResetReason::ControllerInactive};
};

class RollTbr
{
public:
	void reset()
	{
		_filter.reset(0.f);
		_slew.setForcedValue(0.f);
		_have_config = false;
	}

	Result update(const Config &config, const Input &input)
	{
		Result result{};
		result.config_valid = validConfig(config, input.dt) && (input.enable == 0 || input.enable == 1);
		ResetReason reason = ResetReason::None;

		if (input.enable == 0) { reason = ResetReason::Disabled; }
		else if (!result.config_valid) { reason = ResetReason::InvalidConfig; }
		else if (!input.armed) { reason = ResetReason::Disarmed; }
		else if (!input.supported) { reason = ResetReason::Unsupported; }
		else if (!input.mission) { reason = ResetReason::NotMission; }
		else if (input.landed) { reason = ResetReason::Landed; }
		else if (!input.controller_active) { reason = ResetReason::ControllerInactive; }
		else if (input.integral_reset) { reason = ResetReason::IntegralReset; }
		else if (!validDt(input.dt) || !validDt(input.measured_dt)) { reason = ResetReason::InvalidDt; }
		else if (!input.inputs_valid || !std::isfinite(input.setpoint) || !std::isfinite(input.rate)) {
			reason = ResetReason::NonfiniteInput;
		}

		if (reason != ResetReason::None) {
			reset();
			result.reset_reason = reason;
			return result;
		}

		const bool changed = _have_config && std::memcmp(&_config, &config, sizeof(config)) != 0;

		if (changed) { reset(); }

		_config = config;
		_have_config = true;

		if (!_filter.setCutoffFreq(1.f / input.dt, config.fc)) {
			reset();
			result.reset_reason = ResetReason::InvalidConfig;
			result.config_valid = false;
			return result;
		}

		result.error = input.setpoint - input.rate;
		result.filtered = _filter.update(result.error);
		result.raw = config.k * result.filtered;

		if (!std::isfinite(result.error) || !std::isfinite(result.filtered) || !std::isfinite(result.raw)) {
			reset();
			result = Result{};
			result.config_valid = true;
			result.reset_reason = ResetReason::NonfiniteInput;
			return result;
		}

		result.capped = math::constrain(result.raw, -config.max, config.max);
		result.cap_active = std::fabs(result.raw) > config.max;
		const float previous = _slew.getState();
		_slew.setSlewRate(config.slew);
		result.output = _slew.update(result.capped, input.dt);
		result.slew_active = std::fabs(result.capped - previous) > config.slew * input.dt;
		result.active = true;
		result.reset_reason = changed ? ResetReason::ConfigChanged : ResetReason::None;
		return result;
	}

	static bool validDt(float dt)
	{
		return std::isfinite(dt) && dt >= 0.002f && dt <= 0.04f;
	}

	static bool validConfig(const Config &c, float dt)
	{
		return std::isfinite(c.fc) && c.fc > 0.f && c.fc <= 10.f
		       && (!validDt(dt) || c.fc < 0.5f / dt)
		       && std::isfinite(c.k) && c.k >= 0.f && c.k <= 1.f
		       && std::isfinite(c.max) && c.max > 0.f && c.max <= 1.f
		       && std::isfinite(c.slew) && c.slew > 0.f && c.slew <= 10.f;
	}

private:
	AlphaFilter<float> _filter;
	SlewRate<float> _slew;
	Config _config{};
	bool _have_config{false};
};

} // namespace roll_tbr
