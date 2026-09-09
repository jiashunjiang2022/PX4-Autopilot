#include <gtest/gtest.h>

#include "../fast_predictor.hpp"
#include "../generated/v2_pitch_base_model.hpp"
#include "../generated/v2_pitch_temporal_model.hpp"
#include "../generated/v2_roll_model.hpp"
#include "../generated/v3_slow_engineering_model.hpp"
#include "../shadow_safety.hpp"
#include "../slow_engineering_shadow.hpp"

#include <cmath>
#include <cstring>

using namespace flap_aug_shadow;

TEST(FlapAugShadow, FeatureOrder)
{
	EXPECT_STREQ(generated::v2_roll::FeatureNames[0], "roll_sp");
	EXPECT_STREQ(generated::v2_roll::FeatureNames[19], "u_tail_pitch");
	EXPECT_STREQ(generated::v2_pitch_base::FeatureNames[0], "roll");
	EXPECT_STREQ(generated::v2_pitch_base::FeatureNames[46], "wind_estimate_magnitude");
	EXPECT_STREQ(generated::v3_slow::FeatureNames[0], "manual_roll");
	EXPECT_STREQ(generated::v3_slow::FeatureNames[63], "delta__roll__0p4s");
}

TEST(FlapAugShadow, ScalerAndRidge)
{
	float base[FastPredictor::BaseFeatureCount] {};

	for (size_t index = 0; index < FastPredictor::BaseFeatureCount; ++index) {
		base[index] = generated::v2_pitch_base::Mean[index];
	}

	EXPECT_NEAR(FastPredictor::predict_pitch_base(base), generated::v2_pitch_base::Intercept, 1e-7f);
	base[0] += generated::v2_pitch_base::Std[0];
	EXPECT_NEAR(FastPredictor::predict_pitch_base(base),
		    generated::v2_pitch_base::Intercept + generated::v2_pitch_base::Coefficient[0], 1e-7f);

	float v3[SlowEngineeringShadow::FeatureCount] {};

	for (size_t index = 0; index < SlowEngineeringShadow::FeatureCount; ++index) {
		v3[index] = generated::v3_slow::Mean[index];
	}

	EXPECT_NEAR(SlowEngineeringShadow::predict_features(v3), generated::v3_slow::Intercept, 1e-7f);
}

TEST(FlapAugShadow, PitchTemporalHistory)
{
	FastPredictor predictor;
	float base[FastPredictor::BaseFeatureCount] {};

	for (size_t index = 0; index < FastPredictor::BaseFeatureCount; ++index) {
		base[index] = 0.01f * static_cast<float>(index + 1);
	}

	FastPredictor::Result result{};

	for (size_t sample = 0; sample < FastPredictor::HistorySamples; ++sample) {
		result = predictor.update(base, true, true);
		EXPECT_EQ(result.pitch_valid, sample + 1 == FastPredictor::HistorySamples);
	}

	float temporal[517] {};

	for (size_t block = 0; block < 6; ++block) {
		std::memcpy(&temporal[block * FastPredictor::BaseFeatureCount], base, sizeof(base));
	}

	std::memcpy(&temporal[6 * FastPredictor::BaseFeatureCount], base, sizeof(base));
	std::memcpy(&temporal[8 * FastPredictor::BaseFeatureCount], base, sizeof(base));
	std::memcpy(&temporal[9 * FastPredictor::BaseFeatureCount], base, sizeof(base));
	const float expected = FastPredictor::predict_pitch_base(base) + FastPredictor::predict_pitch_temporal(temporal);
	EXPECT_NEAR(result.pitch, expected, 2e-6f);
}

TEST(FlapAugShadow, PhaseHysteresisAndConfidence)
{
	PhaseClassifier classifier;
	PhaseResult phase{};
	constexpr float DegToRad = 0.0174532925199433f;

	for (int index = 0; index < 10; ++index) {
		phase = classifier.update(0.15f, 8.f * DegToRad, 20.f * DegToRad,
					  30.f * DegToRad, 0.15f, true, false);
	}

	EXPECT_EQ(phase.phase, ManeuverPhase::TurnEntry);
	EXPECT_FLOAT_EQ(phase.confidence, 0.f);

	for (int index = 0; index < 25; ++index) {
		phase = classifier.update(0.15f, 8.f * DegToRad, 20.f * DegToRad,
					  30.f * DegToRad, 0.15f, true, false);
	}

	EXPECT_EQ(phase.phase, ManeuverPhase::SteadyTurn);
	EXPECT_FLOAT_EQ(phase.confidence, 0.4f);

	for (int index = 0; index < 15; ++index) {
		phase = classifier.update(0.f, 0.f, 0.f, 0.f, 0.f, true, false);
	}

	EXPECT_EQ(phase.phase, ManeuverPhase::TurnExit);

	for (int index = 0; index < 25; ++index) {
		phase = classifier.update(0.f, 0.f, 0.f, 0.f, 0.f, true, false);
	}

	EXPECT_EQ(phase.phase, ManeuverPhase::StraightOrLowManeuver);
	EXPECT_FLOAT_EQ(phase.confidence, 1.f);
	phase = classifier.update(0.f, 0.f, 0.f, 0.f, 0.f, false, false);
	EXPECT_EQ(phase.phase, ManeuverPhase::ExtremeOrInvalid);
	EXPECT_FLOAT_EQ(phase.confidence, 0.f);
}

TEST(FlapAugShadow, SlowBoundedUpdateAndModeRetention)
{
	SlowEngineeringShadow slow;
	float current[SlowEngineeringShadow::CurrentFeatureCount] {};
	SlowEngineeringShadow::Result result{};

	for (size_t sample = 0; sample < SlowEngineeringShadow::HistorySamples; ++sample) {
		result = slow.update(current, true, 1.f, 1.f, true);
	}

	ASSERT_TRUE(result.model_valid);
	ASSERT_TRUE(result.update_enabled);
	EXPECT_LE(std::fabs(result.innovation), 0.2f);
	EXPECT_LE(std::fabs(result.slow_hat), 0.6f);
	const float before_transition = slow.state();
	result = slow.update(current, true, 1.f, 0.f, false);
	EXPECT_FLOAT_EQ(result.slow_hat, before_transition);
	result = slow.update(current, true, 1.f, 1.f, true);
	EXPECT_GE(std::fabs(result.slow_hat), std::fabs(before_transition));
	EXPECT_LE(std::fabs(result.slew_rate), 0.02f);
}

TEST(FlapAugShadow, MissingDataIsFiniteAndInvalid)
{
	FastPredictor fast;
	float base[FastPredictor::BaseFeatureCount] {};
	const auto fast_result = fast.update(base, false, false);
	EXPECT_FALSE(fast_result.roll_valid);
	EXPECT_FALSE(fast_result.pitch_valid);
	EXPECT_TRUE(std::isfinite(fast_result.roll));
	EXPECT_TRUE(std::isfinite(fast_result.pitch));

	SlowEngineeringShadow slow;
	float current[SlowEngineeringShadow::CurrentFeatureCount] {};
	const auto slow_result = slow.update(current, false, NAN, 0.f, false);
	EXPECT_FALSE(slow_result.model_valid);
	EXPECT_TRUE(std::isfinite(slow_result.slow_hat));
}

TEST(FlapAugShadow, ActualInjectionAlwaysZero)
{
	EXPECT_FLOAT_EQ(ActualInjection::Slow, 0.f);
	EXPECT_FLOAT_EQ(ActualInjection::FastRoll, 0.f);
	EXPECT_FLOAT_EQ(ActualInjection::FastPitch, 0.f);
}
