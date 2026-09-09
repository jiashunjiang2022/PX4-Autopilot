#include "fast_predictor.hpp"

#include "generated/v2_pitch_base_model.hpp"
#include "generated/v2_pitch_temporal_model.hpp"
#include "generated/v2_roll_model.hpp"

#include <cmath>
#include <cstring>

namespace flap_aug_shadow
{
namespace
{
template<size_t N>
float ridge(const float (&features)[N], const float (&mean)[N], const float (&scale)[N],
	    const float (&coefficient)[N], float intercept)
{
	float result = intercept;

	for (size_t index = 0; index < N; ++index) {
		const float standardized = std::isfinite(features[index]) ?
			(features[index] - mean[index]) / scale[index] : 0.f;
		result += standardized * coefficient[index];
	}

	return result;
}

constexpr size_t RollSourceIndices[20] {7, 8, 9, 10, 11, 12, 13, 14, 15, 16,
					23, 24, 25, 26, 27, 28, 29, 30, 31, 32};
constexpr size_t TemporalOffsets[6] {0, 5, 10, 25, 50, 99};
constexpr float TemporalSlopeDenominator = 33.33f;
}

static_assert(generated::v2_roll::FeatureCount == 20);
static_assert(generated::v2_pitch_base::FeatureCount == FastPredictor::BaseFeatureCount);
static_assert(generated::v2_pitch_temporal::FeatureCount == 517);
static_assert(generated::v2_pitch_temporal::HistorySamples == FastPredictor::HistorySamples);

void FastPredictor::reset()
{
	std::memset(_history, 0, sizeof(_history));
	std::memset(_history_valid, 0, sizeof(_history_valid));
	std::memset(_temporal_features, 0, sizeof(_temporal_features));
	_next = 0;
	_count = 0;
	_last = {};
}

float FastPredictor::predict_roll(const float (&base_features)[BaseFeatureCount])
{
	float selected[generated::v2_roll::FeatureCount] {};

	for (size_t index = 0; index < generated::v2_roll::FeatureCount; ++index) {
		selected[index] = base_features[RollSourceIndices[index]];
	}

	return ridge(selected, generated::v2_roll::Mean, generated::v2_roll::Std,
		     generated::v2_roll::Coefficient, generated::v2_roll::Intercept);
}

float FastPredictor::predict_pitch_base(const float (&base_features)[BaseFeatureCount])
{
	return ridge(base_features, generated::v2_pitch_base::Mean, generated::v2_pitch_base::Std,
		     generated::v2_pitch_base::Coefficient, generated::v2_pitch_base::Intercept);
}

float FastPredictor::predict_pitch_temporal(const float (&temporal_features)[517])
{
	return ridge(temporal_features, generated::v2_pitch_temporal::Mean, generated::v2_pitch_temporal::Std,
		     generated::v2_pitch_temporal::Coefficient, generated::v2_pitch_temporal::Intercept);
}

const float *FastPredictor::history_at_offset(size_t offset_from_newest) const
{
	const size_t newest = (_next + HistorySamples - 1) % HistorySamples;
	return _history[(newest + HistorySamples - offset_from_newest) % HistorySamples];
}

void FastPredictor::build_temporal(float (&output)[517]) const
{
	size_t output_index = 0;

	for (const size_t offset : TemporalOffsets) {
		const float *sample = history_at_offset(offset);

		for (size_t feature = 0; feature < BaseFeatureCount; ++feature) {
			output[output_index++] = sample[feature];
		}
	}

	for (size_t feature = 0; feature < BaseFeatureCount; ++feature) {
		float sum = 0.f;
		size_t finite_count = 0;

		for (size_t offset = 0; offset < HistorySamples; ++offset) {
			const float value = history_at_offset(offset)[feature];

			if (std::isfinite(value)) {
				sum += value;
				++finite_count;
			}
		}

		output[6 * BaseFeatureCount + feature] = finite_count > 0 ? sum / finite_count : NAN;
	}

	for (size_t feature = 0; feature < BaseFeatureCount; ++feature) {
		const float mean = output[6 * BaseFeatureCount + feature];
		float square_sum = 0.f;
		float minimum = INFINITY;
		float maximum = -INFINITY;
		float slope_numerator = 0.f;
		size_t finite_count = 0;

		for (size_t chronological = 0; chronological < HistorySamples; ++chronological) {
			const size_t offset = HistorySamples - 1 - chronological;
			const float value = history_at_offset(offset)[feature];

			if (std::isfinite(value)) {
				const float difference = value - mean;
				square_sum += difference * difference;
				minimum = fminf(minimum, value);
				maximum = fmaxf(maximum, value);
				const float centered_time = static_cast<float>(chronological) * 0.02f - 0.99f;
				slope_numerator += centered_time * difference;
				++finite_count;
			}
		}

		output[7 * BaseFeatureCount + feature] = finite_count > 0 ? sqrtf(square_sum / finite_count) : NAN;
		output[8 * BaseFeatureCount + feature] = finite_count > 0 ? minimum : NAN;
		output[9 * BaseFeatureCount + feature] = finite_count > 0 ? maximum : NAN;
		output[10 * BaseFeatureCount + feature] = finite_count > 0 ? slope_numerator / TemporalSlopeDenominator : NAN;
	}
}

FastPredictor::Result FastPredictor::update(const float (&base_features)[BaseFeatureCount],
		bool roll_input_valid, bool pitch_input_valid)
{
	std::memcpy(_history[_next], base_features, sizeof(base_features));
	_history_valid[_next] = pitch_input_valid;
	_next = (_next + 1) % HistorySamples;
	_count = _count < HistorySamples ? _count + 1 : HistorySamples;

	if (roll_input_valid) {
		_last.roll = predict_roll(base_features);
		_last.roll_valid = std::isfinite(_last.roll);

	} else {
		_last.roll_valid = false;
	}

	bool complete_history = _count == HistorySamples;

	for (size_t index = 0; index < HistorySamples && complete_history; ++index) {
		complete_history = _history_valid[index];
	}

	if (complete_history) {
		build_temporal(_temporal_features);
		_last.pitch = predict_pitch_base(base_features) + predict_pitch_temporal(_temporal_features);
		_last.pitch_valid = std::isfinite(_last.pitch);

	} else {
		_last.pitch_valid = false;
	}

	return _last;
}

} // namespace flap_aug_shadow
