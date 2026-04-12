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

TEST(WingPhaseMath, supports_fractional_hall_zero_count)
{
	const double counts_per_cycle = 30720.0;
	const double encoder_total_count = 1000.0 + counts_per_cycle;
	const double hall_zero_count = 1000.5;
	const double expected_phase = (counts_per_cycle - 0.5) * (2.0 * 3.14159265358979323846 / counts_per_cycle);

	const auto result = wing_phase::compute_phase(encoder_total_count, hall_zero_count, counts_per_cycle, true);

	EXPECT_TRUE(result.valid);
	EXPECT_NEAR(result.phase_unwrapped_rad, expected_phase, 1e-6f);
	EXPECT_NEAR(result.phase_rad, expected_phase, 1e-6f);
}

TEST(WingPhaseMath, interpolates_hall_zero_count_between_encoder_samples)
{
	const wing_phase::EncoderSample previous{1000000, 1000.0};
	const wing_phase::EncoderSample current{1010000, 2000.0};

	const auto result = wing_phase::interpolate_count_at_timestamp(previous, current, 1002500);

	EXPECT_TRUE(result.valid);
	EXPECT_NEAR(result.total_count, 1250.0, 1e-9);
}

TEST(WingPhaseMath, rejects_hall_timestamp_outside_encoder_bracket)
{
	const wing_phase::EncoderSample previous{1000000, 1000.0};
	const wing_phase::EncoderSample current{1010000, 2000.0};

	EXPECT_FALSE(wing_phase::interpolate_count_at_timestamp(previous, current, 999999).valid);
	EXPECT_FALSE(wing_phase::interpolate_count_at_timestamp(previous, current, 1010001).valid);
}
