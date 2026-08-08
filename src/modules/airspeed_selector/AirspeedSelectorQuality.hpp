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
	uint64_t hold_until{0};
	uint64_t reenable_since{0};
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

constexpr QualityDecision evaluate_quality(bool fresh, bool q_valid, float q, bool fuse_enabled,
		bool spectral_driven, float reject_threshold, float hysteresis)
{
	if (!fresh) {
		return {true, false, true};
	}

	const bool q_too_low = q_valid && (q < reject_threshold);
	return {
		!q_valid || !fuse_enabled || (q_too_low && spectral_driven),
		q_valid && fuse_enabled && (q > reject_threshold + hysteresis),
		false
	};
}

inline void update_latch(uint64_t now, uint64_t hold_duration, uint64_t reenable_dwell,
		const QualityDecision &decision, QualityLatchState &state)
{
	if (decision.reject) {
		state.latched = true;
		state.hold_until = now + hold_duration;
		state.reenable_since = 0;
	}

	if (!state.latched) {
		return;
	}

	if ((now < state.hold_until) || !decision.reopen) {
		state.reenable_since = 0;

	} else if (state.reenable_since == 0) {
		state.reenable_since = now;

	} else if ((now - state.reenable_since) >= reenable_dwell) {
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
