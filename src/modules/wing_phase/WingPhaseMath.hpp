#pragma once

#include <stdint.h>

namespace wing_phase
{

struct Result {
	bool valid{false};
	float phase_rad{0.f};
	float phase_unwrapped_rad{0.f};
};

Result compute_phase(int64_t encoder_total_count, int64_t zero_count, float counts_per_cycle, bool zero_locked);

} // namespace wing_phase
