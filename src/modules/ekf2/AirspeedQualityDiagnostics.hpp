#pragma once

#include "EKF/common.h"

#include <lib/airspeed/AirspeedQualityMode.hpp>
#include <mathlib/math/Functions.hpp>
#include <uORB/topics/ekf2_airspeed_quality.h>

#include <cstdint>

namespace airspeed_quality
{

constexpr bool monitoring_publication_required(const ModeConfig &mode_config, bool quality_input_updated,
		bool observation_published)
{
	return mode_config.selector_quality_enabled && quality_input_updated && !observation_published;
}

inline void set_monitoring_diagnostic(ekf2_airspeed_quality_s &diagnostic, uint64_t quality_timestamp_sample,
		uint8_t quality_source_instance, uint32_t quality_device_id)
{
	// A monitoring update carries current physical-source quality without claiming an EKF airspeed observation.
	diagnostic.qmon = true;
	diagnostic.timestamp_sample = 0;
	diagnostic.ekf_buffer_timestamp_sample = 0;
	diagnostic.quality_timestamp_sample = quality_timestamp_sample;
	diagnostic.quality_age_us = UINT32_MAX;
	diagnostic.airspeed_source = -1;
	diagnostic.airspeed_device_id = 0;
	diagnostic.quality_source_instance = quality_source_instance;
	diagnostic.quality_device_id = quality_device_id;
	diagnostic.eas2tas = NAN;
	diagnostic.r_as_used = NAN;
}

inline void set_observation_diagnostic(ekf2_airspeed_quality_s &diagnostic, const estimator::airspeedSample &sample,
		uint64_t observation_timestamp_sample, uint64_t ekf_buffer_timestamp_sample, uint64_t quality_timestamp_sample,
		int8_t airspeed_source, uint32_t airspeed_device_id, uint8_t quality_source_instance,
		uint32_t quality_device_id)
{
	diagnostic.qmon = false;
	diagnostic.timestamp_sample = observation_timestamp_sample;
	diagnostic.ekf_buffer_timestamp_sample = ekf_buffer_timestamp_sample;
	diagnostic.quality_timestamp_sample = quality_timestamp_sample;
	diagnostic.quality_age_us = (quality_timestamp_sample > 0 && observation_timestamp_sample >= quality_timestamp_sample)
				    ? static_cast<uint32_t>(math::min(observation_timestamp_sample - quality_timestamp_sample,
								     static_cast<uint64_t>(UINT32_MAX))) : UINT32_MAX;
	diagnostic.airspeed_source = airspeed_source;
	diagnostic.airspeed_device_id = airspeed_device_id;
	diagnostic.quality_source_instance = quality_source_instance;
	diagnostic.quality_device_id = quality_device_id;
	diagnostic.eas2tas = sample.eas2tas;
	diagnostic.r_as_used = sample.noise_var;
	diagnostic.fuse_enabled = sample.fuse_enabled;
}

} // namespace airspeed_quality
