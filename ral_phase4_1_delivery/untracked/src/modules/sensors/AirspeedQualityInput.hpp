#pragma once

#include <cmath>
#include <cstdint>

namespace airspeed_quality_input
{

constexpr bool source_identity_changed(uint32_t previous_device_id, uint8_t previous_instance,
		uint32_t device_id, uint8_t instance)
{
	return previous_device_id != 0 && (previous_device_id != device_id || previous_instance != instance);
}

constexpr bool source_device_valid(uint32_t device_id)
{
	return device_id != 0;
}

inline bool pressure_in_range(float pressure_pa, float absolute_limit_pa)
{
	return std::isfinite(pressure_pa) && std::isfinite(absolute_limit_pa) && absolute_limit_pa > 0.f
	       && std::fabs(pressure_pa) <= absolute_limit_pa;
}

inline bool source_rate_valid(float rate_hz, uint8_t interval_count, float minimum_hz, float maximum_hz,
		uint8_t required_intervals)
{
	return std::isfinite(rate_hz) && interval_count >= required_intervals
	       && rate_hz >= minimum_hz && rate_hz <= maximum_hz;
}

inline bool source_rate_requires_reconfigure(float measured_hz, float configured_hz, float fractional_threshold)
{
	return std::isfinite(measured_hz) && std::isfinite(configured_hz) && configured_hz > 0.f
	       && std::fabs(measured_hz - configured_hz) > configured_hz * fractional_threshold;
}

constexpr bool bracketed(uint64_t previous_timestamp, uint64_t current_timestamp, uint64_t output_timestamp)
{
	return current_timestamp > previous_timestamp && output_timestamp >= previous_timestamp
	       && output_timestamp <= current_timestamp;
}

inline float interpolate(float previous_value, float current_value, uint64_t previous_timestamp,
		uint64_t current_timestamp, uint64_t output_timestamp)
{
	const float fraction = static_cast<float>(output_timestamp - previous_timestamp)
			       / static_cast<float>(current_timestamp - previous_timestamp);
	return previous_value + fraction * (current_value - previous_value);
}

} // namespace airspeed_quality_input
