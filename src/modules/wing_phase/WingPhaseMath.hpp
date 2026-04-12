#pragma once

#include <drivers/drv_hrt.h>

#include <stdint.h>

namespace wing_phase
{

struct EncoderSample {
	hrt_abstime timestamp{0};
	double total_count{0.0};
};

struct Result {
	bool valid{false};
	float phase_rad{0.f};
	float phase_unwrapped_rad{0.f};
};

struct CountInterpolationResult {
	bool valid{false};
	double total_count{0.0};
};

CountInterpolationResult interpolate_count_at_timestamp(const EncoderSample &previous, const EncoderSample &current,
		hrt_abstime target_timestamp);
Result compute_phase(double encoder_total_count, double zero_count, float counts_per_cycle, bool zero_locked);

} // namespace wing_phase
