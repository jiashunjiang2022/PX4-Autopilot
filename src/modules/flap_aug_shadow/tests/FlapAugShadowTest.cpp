#include <gtest/gtest.h>

#include "../generated/v3_slow_engineering_model.hpp"
#include "../shadow_safety.hpp"
#include "../slow_engineering_shadow.hpp"

#include <cmath>
#include <cstring>

using namespace flap_aug_shadow;

TEST(FlapAugShadow, FeatureOrder)
{
	EXPECT_STREQ(generated::v3_slow::FeatureNames[0], "manual_roll");
	EXPECT_STREQ(generated::v3_slow::FeatureNames[63], "delta__roll__0p4s");
}

TEST(FlapAugShadow, ScalerAndRidge)
{
	float v3[SlowEngineeringShadow::FeatureCount] {};

	for (size_t index = 0; index < SlowEngineeringShadow::FeatureCount; ++index) {
		v3[index] = generated::v3_slow::Mean[index];
	}

	EXPECT_NEAR(SlowEngineeringShadow::predict_features(v3), generated::v3_slow::Intercept, 1e-7f);
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
		EXPECT_EQ(result.history_ready, sample + 1 == SlowEngineeringShadow::HistorySamples);
	}

	ASSERT_TRUE(result.model_valid);
	ASSERT_TRUE(result.residual_valid);
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
	SlowEngineeringShadow slow;
	float current[SlowEngineeringShadow::CurrentFeatureCount] {};
	const auto slow_result = slow.update(current, false, NAN, 0.f, false);
	EXPECT_FALSE(slow_result.history_ready);
	EXPECT_FALSE(slow_result.model_valid);
	EXPECT_FALSE(slow_result.residual_valid);
	EXPECT_TRUE(std::isfinite(slow_result.slow_hat));
}

TEST(FlapAugShadow, ActualInjectionAlwaysZero)
{
	EXPECT_FLOAT_EQ(ActualInjection::Slow, 0.f);
	EXPECT_FLOAT_EQ(ActualInjection::FastRoll, 0.f);
	EXPECT_FLOAT_EQ(ActualInjection::FastPitch, 0.f);
}
