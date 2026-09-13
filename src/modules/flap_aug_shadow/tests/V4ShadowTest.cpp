#include <gtest/gtest.h>

#include <cmath>

#include "../v4_shadow.hpp"

using namespace flap_aug_shadow;

TEST(V4Shadow, DisabledDoesNotCreateControlPath)
{
	V4ShadowState state;
	const V4ShadowResult result = state.update(false, 0.4f, true, -0.2f, 0.2f, 0.f, 0.22f, true, 0.03f, 0.f, 0.f, 0.f);
	EXPECT_FALSE(result.enabled);
	EXPECT_TRUE(result.b_prior_valid);
	EXPECT_FLOAT_EQ(result.b_prior, 0.4f);
	EXPECT_FLOAT_EQ(result.delta_b_shadow, 0.f);
	EXPECT_FLOAT_EQ(result.u_trim_candidate_shadow, 0.4f);
	EXPECT_TRUE(result.u_controller_eq_valid);
	EXPECT_FLOAT_EQ(result.u_aug_eq_shadow, 0.f);
	EXPECT_FLOAT_EQ(result.tail_roll_realized, 0.2f);
	EXPECT_FLOAT_EQ(result.tail_pitch_realized, 0.f);
	EXPECT_FLOAT_EQ(result.tail_yaw_realized, 0.f);
	EXPECT_TRUE(result.zero_aug_mapping_valid);
	EXPECT_TRUE(result.r_total_proxy_valid);
	EXPECT_FLOAT_EQ(result.r_total_proxy_shadow, 0.17f);
	EXPECT_TRUE(result.total_mapping_valid);
	EXPECT_FALSE(result.delta_update_implemented);
}

TEST(V4Shadow, PriorAndCandidateAreFiniteAndBounded)
{
	V4ShadowState state;
	const V4ShadowResult result = state.update(true, 9.f, false, NAN, NAN, NAN, NAN, true, INFINITY, 0.f, 0.f, 0.f);
	EXPECT_TRUE(std::isfinite(result.b_prior));
	EXPECT_FALSE(result.b_prior_valid);
	EXPECT_FLOAT_EQ(result.b_prior, 0.f);
	EXPECT_FLOAT_EQ(result.u_trim_candidate_shadow, 0.f);
	EXPECT_FLOAT_EQ(result.u_controller_eq, 0.f);
	EXPECT_FALSE(result.maneuver_hat_valid);
	EXPECT_FLOAT_EQ(result.maneuver_hat, 0.f);
}

TEST(V4Shadow, ThreeSurfaceCoordinateMapping)
{
	V4ShadowState state;
	const V4ShadowResult result = state.update(true, 0.f, true, 0.2f, 0.2f, 0.2f, 0.22f, false, 0.f,
									 0.f, 0.f, 0.f);
	EXPECT_FLOAT_EQ(result.tail_roll_realized, 0.f);
	EXPECT_FLOAT_EQ(result.tail_pitch_realized, 0.2f);
	EXPECT_FLOAT_EQ(result.tail_yaw_realized, 0.2f);
	EXPECT_FLOAT_EQ(result.roll_torque_equiv_prealloc, 0.2f);
	EXPECT_FLOAT_EQ(result.allocation_consistency_error, -0.2f);
	EXPECT_TRUE(result.zero_aug_mapping_valid);
	EXPECT_FALSE(result.r_total_proxy_valid);
}

TEST(V4Shadow, DeltaStartsAtZeroAndResetIsExplicit)
{
	V4ShadowState state;
	const V4ShadowResult before = state.update(true, -1.5f, true, 0.2f, -0.2f, 0.2f, 0.f, false, 0.f, 0.f, 0.f, 0.f);
	EXPECT_FLOAT_EQ(before.delta_b_shadow, 0.f);
	state.reset();
	const V4ShadowResult after = state.update(true, -1.5f, true, 0.2f, -0.2f, 0.2f, 0.f, true, 0.1f, 0.f, 0.f, 0.f);
	EXPECT_FLOAT_EQ(after.delta_b_shadow, 0.f);
	EXPECT_FLOAT_EQ(after.u_trim_candidate_shadow, -1.5f);
	EXPECT_FLOAT_EQ(after.u_aug_eq_shadow, 0.f);
	EXPECT_TRUE(after.r_total_proxy_valid);
	EXPECT_FLOAT_EQ(after.r_total_proxy_shadow, -0.3f);
}

TEST(V4Shadow, NonzeroInjectionInvalidatesZeroAugMapping)
{
	V4ShadowState state;
	const V4ShadowResult result = state.update(true, 0.f, true, -0.2f, 0.2f, 0.2f, 0.f, true, 0.05f,
									 1e-3f, 0.f, 0.f);
	EXPECT_FALSE(result.zero_aug_mapping_valid);
	EXPECT_FALSE(result.nonzero_aug_mapping_valid);
	EXPECT_FALSE(result.r_total_proxy_valid);
	EXPECT_FLOAT_EQ(result.r_total_proxy_shadow, 0.f);
}
