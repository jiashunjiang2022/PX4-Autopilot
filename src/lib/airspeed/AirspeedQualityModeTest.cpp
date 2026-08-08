#include <gtest/gtest.h>

#include "AirspeedQualityMode.hpp"

using airspeed_quality::ExperimentMode;

TEST(AirspeedQualityMode, truth_table)
{
	const auto baseline = airspeed_quality::mode_config(0);
	EXPECT_EQ(baseline.mode, ExperimentMode::Baseline);
	EXPECT_FALSE(baseline.adaptive_r_enabled);
	EXPECT_FALSE(baseline.constant_r_enabled);
	EXPECT_FALSE(baseline.quality_fusion_gate_enabled);
	EXPECT_FALSE(baseline.selector_quality_enabled);

	const auto constant_r = airspeed_quality::mode_config(1);
	EXPECT_TRUE(constant_r.constant_r_enabled);
	EXPECT_FALSE(constant_r.adaptive_r_enabled);
	EXPECT_FALSE(constant_r.quality_fusion_gate_enabled);
	EXPECT_FALSE(constant_r.selector_quality_enabled);

	const auto variance_only = airspeed_quality::mode_config(2);
	EXPECT_TRUE(variance_only.adaptive_r_enabled);
	EXPECT_FALSE(variance_only.quality_fusion_gate_enabled);
	EXPECT_FALSE(variance_only.selector_quality_enabled);

	const auto full = airspeed_quality::mode_config(3);
	EXPECT_TRUE(full.adaptive_r_enabled);
	EXPECT_TRUE(full.quality_fusion_gate_enabled);
	EXPECT_TRUE(full.selector_quality_enabled);
}

TEST(AirspeedQualityMode, unknown_fails_closed_to_baseline)
{
	const auto config = airspeed_quality::mode_config(99);
	EXPECT_FALSE(config.valid);
	EXPECT_EQ(config.mode, ExperimentMode::Baseline);
	EXPECT_FALSE(config.adaptive_r_enabled);
	EXPECT_FALSE(config.quality_fusion_gate_enabled);
	EXPECT_FALSE(config.selector_quality_enabled);
}

TEST(AirspeedQualityMode, variance_mapping_uses_variance_multipliers)
{
	EXPECT_FLOAT_EQ(airspeed_quality::observation_variance(4.f, 0.2f, 5.f, 3.f,
			airspeed_quality::mode_config(0)), 4.f);
	EXPECT_FLOAT_EQ(airspeed_quality::observation_variance(4.f, 0.2f, 5.f, 3.f,
			airspeed_quality::mode_config(1)), 12.f);
	EXPECT_FLOAT_EQ(airspeed_quality::observation_variance(4.f, 0.5f, 5.f, 3.f,
			airspeed_quality::mode_config(2)), 12.f);
}

TEST(AirspeedQualityMode, identity_mismatch_disables_quality_effects)
{
	const auto matched = airspeed_quality::source_bound_config(airspeed_quality::mode_config(3), true);
	EXPECT_TRUE(matched.adaptive_r_enabled);
	EXPECT_TRUE(matched.quality_fusion_gate_enabled);

	const auto mismatched = airspeed_quality::source_bound_config(airspeed_quality::mode_config(3), false);
	EXPECT_FALSE(mismatched.adaptive_r_enabled);
	EXPECT_FALSE(mismatched.quality_fusion_gate_enabled);
	EXPECT_TRUE(mismatched.selector_quality_enabled);

	const auto constant = airspeed_quality::source_bound_config(airspeed_quality::mode_config(1), false);
	EXPECT_FALSE(constant.constant_r_enabled);
}

TEST(AirspeedQualityMode, stale_variance_only_sample_uses_nominal_variance)
{
	const auto stale = airspeed_quality::sample_config(airspeed_quality::mode_config(2), true, false);
	EXPECT_FALSE(stale.adaptive_r_enabled);
	EXPECT_FLOAT_EQ(airspeed_quality::observation_variance(4.f, 0.2f, 5.f, 3.f, stale), 4.f);
}

TEST(AirspeedQualityMode, constant_r_only_applies_to_matching_physical_source)
{
	const auto matched = airspeed_quality::sample_config(airspeed_quality::mode_config(1), true, false);
	EXPECT_TRUE(matched.constant_r_enabled);
	EXPECT_FLOAT_EQ(airspeed_quality::observation_variance(4.f, 0.5f, 5.f, 3.f, matched), 12.f);

	const auto synthetic_or_unknown = airspeed_quality::sample_config(airspeed_quality::mode_config(1), false, true);
	EXPECT_FALSE(synthetic_or_unknown.constant_r_enabled);
	EXPECT_FLOAT_EQ(airspeed_quality::observation_variance(4.f, 0.5f, 5.f, 3.f, synthetic_or_unknown), 4.f);
}
