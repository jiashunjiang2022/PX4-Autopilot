/****************************************************************************
 * Frozen V2 fast Shadow inference. No actuator interfaces are present here.
 ****************************************************************************/

#pragma once

#include <cstddef>

namespace flap_aug_shadow
{

class FastPredictor
{
public:
	static constexpr size_t BaseFeatureCount = 47;
	static constexpr size_t HistorySamples = 100;

	struct Result {
		float roll{0.f};
		float pitch{0.f};
		bool roll_valid{false};
		bool pitch_valid{false};
	};

	Result update(const float (&base_features)[BaseFeatureCount], bool roll_input_valid, bool pitch_input_valid);
	void reset();

	static float predict_roll(const float (&base_features)[BaseFeatureCount]);
	static float predict_pitch_base(const float (&base_features)[BaseFeatureCount]);
	static float predict_pitch_temporal(const float (&temporal_features)[517]);

private:
	void build_temporal(float (&output)[517]) const;
	const float *history_at_offset(size_t offset_from_newest) const;

	float _history[HistorySamples][BaseFeatureCount]{};
	bool _history_valid[HistorySamples]{};
	float _temporal_features[517]{};
	size_t _next{0};
	size_t _count{0};
	Result _last{};
};

} // namespace flap_aug_shadow
