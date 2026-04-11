/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

/**
 * Run this test only using make tests TESTFILTER=WingPhaseMath
 */

#include <gtest/gtest.h>

#include "WingPhaseMath.hpp"

TEST(WingPhaseMath, invalid_without_hall_lock)
{
	const auto result = wing_phase::compute_phase(1200, 1000, 4096.f * 7.5f, false);

	EXPECT_FALSE(result.valid);
}

TEST(WingPhaseMath, wraps_phase_with_hall_zero)
{
	const auto result = wing_phase::compute_phase(4096 * 8, 4096 * 7, 4096.f * 7.5f, true);

	EXPECT_TRUE(result.valid);
	EXPECT_GT(result.phase_rad, 0.f);
	EXPECT_LT(result.phase_rad, static_cast<float>(2.0 * 3.14159265358979323846));
}

TEST(WingPhaseMath, keeps_unwrapped_phase_since_last_hall)
{
	const auto result = wing_phase::compute_phase(1000 + 30720, 1000, 30720.f, true);

	EXPECT_TRUE(result.valid);
	EXPECT_NEAR(result.phase_unwrapped_rad, static_cast<float>(2.0 * 3.14159265358979323846), 1e-3f);
	EXPECT_NEAR(result.phase_rad, 0.f, 1e-3f);
}
