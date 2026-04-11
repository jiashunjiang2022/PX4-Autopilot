#include "WingPhaseMath.hpp"

#include <float.h>
#include <math.h>

namespace wing_phase
{

Result compute_phase(int64_t encoder_total_count, int64_t zero_count, float counts_per_cycle, bool zero_locked)
{
	static constexpr float kTwoPi = 6.28318530717958647692f;

	if (!zero_locked || !isfinite(counts_per_cycle) || counts_per_cycle <= FLT_EPSILON) {
		return {};
	}

	const double delta_count = static_cast<double>(encoder_total_count - zero_count);
	const double phase_unwrapped_rad = delta_count * (static_cast<double>(kTwoPi) / static_cast<double>(counts_per_cycle));
	double phase_rad = fmod(phase_unwrapped_rad, static_cast<double>(kTwoPi));

	if (phase_rad < 0.0) {
		phase_rad += static_cast<double>(kTwoPi);
	}

	Result result{};
	result.valid = true;
	result.phase_rad = static_cast<float>(phase_rad);
	result.phase_unwrapped_rad = static_cast<float>(phase_unwrapped_rad);
	return result;
}

} // namespace wing_phase
