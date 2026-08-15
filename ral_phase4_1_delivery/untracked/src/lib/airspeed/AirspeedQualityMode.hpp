#pragma once

#include <mathlib/math/Functions.hpp>

#include <cstdint>

namespace airspeed_quality
{

enum class ExperimentMode : uint8_t {
	Baseline = 0,
	ConstantR = 1,
	VarianceOnly = 2,
	FullProposed = 3,
};

struct ModeConfig {
	ExperimentMode mode{ExperimentMode::Baseline};
	bool valid{true};
	bool adaptive_r_enabled{false};
	bool constant_r_enabled{false};
	bool quality_fusion_gate_enabled{false};
	bool selector_quality_enabled{false};
};

constexpr ModeConfig mode_config(int32_t requested_mode)
{
	switch (requested_mode) {
	case static_cast<int32_t>(ExperimentMode::Baseline):
		return {ExperimentMode::Baseline, true, false, false, false, false};

	case static_cast<int32_t>(ExperimentMode::ConstantR):
		return {ExperimentMode::ConstantR, true, false, true, false, false};

	case static_cast<int32_t>(ExperimentMode::VarianceOnly):
		return {ExperimentMode::VarianceOnly, true, true, false, false, false};

	case static_cast<int32_t>(ExperimentMode::FullProposed):
		return {ExperimentMode::FullProposed, true, true, false, true, true};

	default:
		return {ExperimentMode::Baseline, false, false, false, false, false};
	}
}

constexpr ModeConfig source_bound_config(const ModeConfig &config, bool source_identity_match)
{
	ModeConfig result{config};

	if (!source_identity_match) {
		result.adaptive_r_enabled = false;
		result.quality_fusion_gate_enabled = false;
	}

	return result;
}

inline float observation_variance(float nominal_variance, float quality, float adaptive_max_factor,
		float constant_factor, const ModeConfig &config)
{
	const float nominal = math::max(nominal_variance, 0.f);

	if (config.constant_r_enabled) {
		return nominal * math::max(constant_factor, 1.f);
	}

	if (config.adaptive_r_enabled) {
		const float q = math::constrain(quality, 0.f, 1.f);
		const float rmax = math::max(adaptive_max_factor, 1.f);
		return nominal * (1.f + (1.f - q) * (rmax - 1.f));
	}

	return nominal;
}

} // namespace airspeed_quality
