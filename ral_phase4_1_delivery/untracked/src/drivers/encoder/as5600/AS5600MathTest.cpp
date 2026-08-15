#include <gtest/gtest.h>

#include "AS5600Math.hpp"

TEST(AS5600Math, converts_rpm_using_eight_to_one_ratio)
{
	EXPECT_FLOAT_EQ(as5600::flap_frequency_from_rpm(480.f, 8.f), 1.f);
	EXPECT_FLOAT_EQ(as5600::flap_frequency_from_rpm(-3264.f, 8.f), 6.8f);
}

TEST(AS5600Math, rejects_invalid_ratio)
{
	EXPECT_TRUE(std::isnan(as5600::flap_frequency_from_rpm(480.f, 0.f)));
}
