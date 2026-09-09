#pragma once

#include <cmath>
#include <uORB/topics/ekf2_airspeed_quality.h>

#include <cstdint>

namespace airspeed_quality
{

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
	diagnostic.nominal_r_as = NAN;
	diagnostic.r_as_used = NAN;
}

} // namespace airspeed_quality
