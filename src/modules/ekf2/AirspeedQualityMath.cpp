#include "AirspeedQualityMath.hpp"

#include <mathlib/math/Functions.hpp>
#include <px4_platform_common/defines.h>

#include <float.h>
#include <math.h>

namespace airspeed_quality
{

TimestampStatus validate_timestamp(uint64_t previous_timestamp, uint64_t timestamp, uint64_t maximum_gap_us)
{
	if (previous_timestamp == 0) {
		return TimestampStatus::Accepted;
	}

	if (timestamp == previous_timestamp) {
		return TimestampStatus::Duplicate;
	}

	if (timestamp < previous_timestamp) {
		return TimestampStatus::NonMonotonic;
	}

	if ((timestamp - previous_timestamp) > maximum_gap_us) {
		return TimestampStatus::LongGap;
	}

	return TimestampStatus::Accepted;
}

bool flap_frequency_timed_out(uint64_t reference_timestamp, uint64_t sample_timestamp, uint64_t timeout_us)
{
	if (sample_timestamp == 0) {
		return true;
	}

	const uint64_t age_us = reference_timestamp > sample_timestamp ? reference_timestamp - sample_timestamp : 0;
	return age_us > timeout_us;
}

int required_window_samples(float sample_rate_hz, float window_seconds, int capacity)
{
	if (!PX4_ISFINITE(sample_rate_hz) || !PX4_ISFINITE(window_seconds) || sample_rate_hz <= 0.f
	    || window_seconds <= 0.f || capacity < 2) {
		return 0;
	}

	return math::constrain(static_cast<int>(roundf(sample_rate_hz * window_seconds)), 2, capacity);
}

bool window_ready(int sample_count, int required_samples)
{
	return required_samples >= 2 && sample_count >= required_samples;
}

int ring_window_start(int head, int required_samples, int capacity)
{
	if (capacity < 2 || head < 0 || head >= capacity || required_samples < 2 || required_samples > capacity) {
		return -1;
	}

	return (head - required_samples + capacity) % capacity;
}

static float goertzel_power(const float *samples, int count, float frequency_hz, float sample_rate_hz)
{
	const float omega = 2.f * M_PI_F * frequency_hz / sample_rate_hz;
	const float coefficient = 2.f * cosf(omega);
	float s1 = 0.f;
	float s2 = 0.f;

	for (int i = 0; i < count; ++i) {
		const float s0 = samples[i] + coefficient * s1 - s2;
		s2 = s1;
		s1 = s0;
	}

	return s1 * s1 + s2 * s2 - coefficient * s1 * s2;
}

SpectralResult evaluate_spectrum(const float *samples, int count, float sample_rate_hz,
		float reference_lower_hz, float reference_upper_hz, float flap_center_hz,
		float flap_half_width_hz, float frequency_step_hz)
{
	SpectralResult result{};
	result.flap_center_hz = flap_center_hz;

	if ((samples == nullptr) || (count < 2)) {
		result.invalid_reason = SpectralInvalidReason::InsufficientSamples;
		return result;
	}

	if (!PX4_ISFINITE(sample_rate_hz) || (sample_rate_hz <= 0.f)) {
		result.invalid_reason = SpectralInvalidReason::InputRate;
		return result;
	}

	if (!PX4_ISFINITE(reference_lower_hz) || !PX4_ISFINITE(reference_upper_hz)
	    || (reference_lower_hz <= 0.f) || (reference_upper_hz <= reference_lower_hz)) {
		result.invalid_reason = SpectralInvalidReason::ReferenceBand;
		return result;
	}

	const float nyquist_hz = 0.5f * sample_rate_hz;

	if (reference_upper_hz >= nyquist_hz) {
		result.invalid_reason = SpectralInvalidReason::Nyquist;
		return result;
	}

	if (!PX4_ISFINITE(flap_center_hz) || !PX4_ISFINITE(flap_half_width_hz)
	    || (flap_half_width_hz <= 0.f)
	    || ((flap_center_hz - flap_half_width_hz) < reference_lower_hz)
	    || ((flap_center_hz + flap_half_width_hz) > reference_upper_hz)) {
		result.invalid_reason = SpectralInvalidReason::FlapBand;
		return result;
	}

	if (!PX4_ISFINITE(frequency_step_hz) || (frequency_step_hz <= 0.f)) {
		result.invalid_reason = SpectralInvalidReason::ReferenceBand;
		return result;
	}

	float total_power = 0.f;
	float flap_power = 0.f;

	for (float frequency_hz = reference_lower_hz;
	     frequency_hz <= reference_upper_hz + 1e-3f; frequency_hz += frequency_step_hz) {
		const float power = goertzel_power(samples, count, frequency_hz, sample_rate_hz);

		if (!PX4_ISFINITE(power)) {
			result.invalid_reason = SpectralInvalidReason::Nonfinite;
			return result;
		}

		total_power += math::max(power, 0.f);

		if ((frequency_hz >= flap_center_hz - flap_half_width_hz)
		    && (frequency_hz <= flap_center_hz + flap_half_width_hz)) {
			flap_power += math::max(power, 0.f);
		}
	}

	if (!PX4_ISFINITE(total_power) || (total_power <= FLT_EPSILON)) {
		result.invalid_reason = SpectralInvalidReason::ZeroEnergy;
		return result;
	}

	result.ratio = math::constrain(flap_power / total_power, 0.f, 1.f);
	result.valid = PX4_ISFINITE(result.ratio);
	result.invalid_reason = result.valid ? SpectralInvalidReason::None : SpectralInvalidReason::Nonfinite;
	return result;
}

} // namespace airspeed_quality
