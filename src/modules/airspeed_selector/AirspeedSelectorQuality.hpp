#pragma once

#include <cstdint>

namespace airspeed_selector_quality
{

struct FallbackChoice {
	int8_t source{-1};
	bool available{false};
};

struct QualityDecision {
	bool reject{false};
	bool reopen{false};
	bool timed_out{false};
};

struct QualityLatchState {
	bool latched{false};
};

enum class FallbackOutcome : uint8_t {
	None = 0,
	AlternatePhysical = 1,
	GroundMinusWind = 2,
	Synthetic = 3,
	Unavailable = 4,
};

struct FallbackStatus {
	int8_t source{-1};
	bool available{false};
	FallbackOutcome outcome{FallbackOutcome::Unavailable};
};

constexpr bool source_identity_matches(int32_t physical_sensor_count, int8_t pre_quality_source,
		uint32_t pre_quality_device_id, uint8_t quality_source_instance, uint32_t quality_device_id)
{
	return physical_sensor_count == 1
	       && pre_quality_source == 1
	       && pre_quality_device_id != 0
	       && quality_device_id != 0
	       && quality_source_instance == 0
	       && quality_device_id == pre_quality_device_id;
}

constexpr bool pre_quality_output_finite(bool indicated_finite, bool calibrated_finite, bool true_finite)
{
	return indicated_finite && calibrated_finite && true_finite;
}

constexpr bool is_physical_source(int8_t source)
{
	return source >= 1 && source <= 3;
}

constexpr bool original_selection_was_fallback(int8_t source)
{
	return source == 0 || source == 4;
}

constexpr bool custom_blockage_enabled(int32_t configured_value)
{
	return configured_value == 1;
}

constexpr FallbackOutcome fallback_outcome_for_source(int8_t source)
{
	if (is_physical_source(source)) {
		return FallbackOutcome::AlternatePhysical;
	}

	if (source == 0) {
		return FallbackOutcome::GroundMinusWind;
	}

	if (source == 4) {
		return FallbackOutcome::Synthetic;
	}

	return FallbackOutcome::Unavailable;
}

constexpr FallbackStatus fallback_status_for_source(int8_t source)
{
	return {source, source != -1, fallback_outcome_for_source(source)};
}

constexpr QualityDecision evaluate_quality(bool fresh, bool fuse_enabled)
{
	if (!fresh) {
		// Stale or invalid quality cannot change an existing gate-derived decision.
		return {false, false, true};
	}

	return {!fuse_enabled, fuse_enabled, false};
}

inline void update_latch(const QualityDecision &decision, QualityLatchState &state)
{
	if (decision.reject) {
		state.latched = true;

	} else if (decision.reopen) {
		state = {};
	}
}

constexpr FallbackChoice choose_fallback(int8_t alternate_physical_source, bool ground_minus_wind_available,
		bool synthetic_available, int32_t configured_fallback)
{
	if (alternate_physical_source > 0) {
		return {alternate_physical_source, true};
	}

	if ((configured_fallback == 1) && ground_minus_wind_available) {
		return {0, true};
	}

	if ((configured_fallback == 2) && synthetic_available) {
		return {4, true};
	}

	return {-1, false};
}

} // namespace airspeed_selector_quality
