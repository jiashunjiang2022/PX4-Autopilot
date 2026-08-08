#include <gtest/gtest.h>

#include "AirspeedQualityInput.hpp"

#include <lib/airspeed/airspeed.h>

TEST(AirspeedQualityInput, source_identity_changes_are_explicit)
{
	EXPECT_FALSE(airspeed_quality_input::source_device_valid(0));
	EXPECT_TRUE(airspeed_quality_input::source_device_valid(42));
	EXPECT_FALSE(airspeed_quality_input::source_identity_changed(0, 0, 42, 0));
	EXPECT_FALSE(airspeed_quality_input::source_identity_changed(42, 0, 42, 0));
	EXPECT_TRUE(airspeed_quality_input::source_identity_changed(42, 0, 43, 0));
	EXPECT_TRUE(airspeed_quality_input::source_identity_changed(42, 0, 42, 1));
}

TEST(AirspeedQualityInput, signed_pressure_is_continuous_and_range_checked)
{
	const float positive = calc_IAS_corrected(AIRSPEED_COMPENSATION_MODEL_PITOT, AIRSPEED_SENSOR_MODEL_MEMBRANE,
			       0.2f, 1.5f, 4.f, 101325.f, 15.f);
	const float negative = calc_IAS_corrected(AIRSPEED_COMPENSATION_MODEL_PITOT, AIRSPEED_SENSOR_MODEL_MEMBRANE,
			       0.2f, 1.5f, -4.f, 101325.f, 15.f);
	EXPECT_GT(positive, 0.f);
	EXPECT_LT(negative, 0.f);
	EXPECT_FLOAT_EQ(positive, -negative);
	EXPECT_TRUE(airspeed_quality_input::pressure_in_range(-0.01f, 7000.f));
	EXPECT_TRUE(airspeed_quality_input::pressure_in_range(-6894.757f, 7000.f));
	EXPECT_FALSE(airspeed_quality_input::pressure_in_range(7001.f, 7000.f));
	EXPECT_FALSE(airspeed_quality_input::pressure_in_range(NAN, 7000.f));
	EXPECT_FALSE(airspeed_quality_input::pressure_in_range(INFINITY, 7000.f));
}

TEST(AirspeedQualityInput, rate_contract_requires_stability_and_reconfigures_sparingly)
{
	EXPECT_FALSE(airspeed_quality_input::source_rate_valid(83.333f, 9, 70.f, 100.f, 10));
	EXPECT_TRUE(airspeed_quality_input::source_rate_valid(83.333f, 10, 70.f, 100.f, 10));
	EXPECT_FALSE(airspeed_quality_input::source_rate_valid(60.f, 10, 70.f, 100.f, 10));
	EXPECT_FALSE(airspeed_quality_input::source_rate_valid(105.f, 10, 70.f, 100.f, 10));
	EXPECT_FALSE(airspeed_quality_input::source_rate_requires_reconfigure(85.f, 83.333f, 0.05f));
	EXPECT_TRUE(airspeed_quality_input::source_rate_requires_reconfigure(75.f, 83.333f, 0.05f));
}

TEST(AirspeedQualityInput, interpolation_requires_two_real_bracketing_samples)
{
	EXPECT_TRUE(airspeed_quality_input::bracketed(10000, 30000, 20000));
	EXPECT_FLOAT_EQ(airspeed_quality_input::interpolate(-2.f, 2.f, 10000, 30000, 20000), 0.f);
	EXPECT_FALSE(airspeed_quality_input::bracketed(10000, 30000, 40000));
	EXPECT_FALSE(airspeed_quality_input::bracketed(10000, 10000, 10000));
}
