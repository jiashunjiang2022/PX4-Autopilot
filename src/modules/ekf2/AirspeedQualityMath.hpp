#ifndef AIRSPEED_QUALITY_MATH_HPP
#define AIRSPEED_QUALITY_MATH_HPP

#include <cstdint>
#include <math.h>

namespace airspeed_quality
{

enum class SpectralInvalidReason : uint8_t {
	None = 0,
	InsufficientSamples = 1,
	InputRate = 2,
	Nyquist = 3,
	ReferenceBand = 4,
	FlapBand = 5,
	ZeroEnergy = 6,
	Nonfinite = 7,
	InputGap = 8,
	NoRecentFlap = 9,
};

struct SpectralResult {
	float ratio{NAN};
	float flap_center_hz{NAN};
	bool valid{false};
	SpectralInvalidReason invalid_reason{SpectralInvalidReason::InsufficientSamples};
};

enum class TimestampStatus : uint8_t {
	Accepted = 0,
	Duplicate,
	NonMonotonic,
	LongGap,
};

TimestampStatus validate_timestamp(uint64_t previous_timestamp, uint64_t timestamp, uint64_t maximum_gap_us);
bool flap_frequency_timed_out(uint64_t reference_timestamp, uint64_t sample_timestamp, uint64_t timeout_us);

int required_window_samples(float sample_rate_hz, float window_seconds, int capacity);
bool window_ready(int sample_count, int required_samples);
int ring_window_start(int head, int required_samples, int capacity);

SpectralResult evaluate_spectrum(const float *samples, int count, float sample_rate_hz,
		float reference_lower_hz, float reference_upper_hz, float flap_center_hz,
		float flap_half_width_hz, float frequency_step_hz);

} // namespace airspeed_quality

#endif // AIRSPEED_QUALITY_MATH_HPP
