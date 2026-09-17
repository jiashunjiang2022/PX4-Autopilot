#include <gtest/gtest.h>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>

#if !defined(TAIL_TRIM_LEGACY_PROBE)
#include "../AdaptiveTailTrimCore.hpp"
using CoreUnderTest = AdaptiveTailTrimCore;
#else
#include "LegacyTailTrimProbe.hpp"
using CoreUnderTest = LegacyTailTrimProbe;
#endif

namespace tail_core_test {
using Core = CoreUnderTest;
constexpr float Tol = 2e-7f;
constexpr float TorqueTol = 2e-6f;
Core::Config config() { return {.1f, .5f, .01f, .2f, .2f, 1.1f}; }
Core::Inputs input(float target = .08f) {
	Core::Inputs in{};
	in.dt = .02f; in.g_cycle = .7f; in.current_native_i = 0.f;
	in.native_i_min = -.2f; in.native_i_max = .2f;
	in.target_b = target; in.gate = true; in.learning_allowed = true;
	return in;
}
void invariant(const Core::Result &r) {
	EXPECT_NEAR(r.g_used*r.accepted_delta_i + 1.1f*r.accepted_delta_b, 0.f, TorqueTol);
	EXPECT_NEAR(r.transfer_mismatch, 0.f, TorqueTol);
	EXPECT_NEAR(r.b_after-r.b_before, r.accepted_delta_b, Tol);
}
}

using namespace tail_core_test;

TEST(AdaptiveTailTrimCore, PhysicalTrimIndependentOfG) {
	for (float g : {.5f, .8f, 1.2f}) {
		Core core(config(), .05f); auto in = input(.05f); in.g_cycle = g;
		const auto r = core.step(in); ASSERT_TRUE(r.transaction_valid);
		EXPECT_FLOAT_EQ(r.b_after, .05f); EXPECT_NEAR(r.tau_trim, .055f, Tol);
	}
}
TEST(AdaptiveTailTrimCore, PhysicalBMax) {
	Core core(config()); auto in = input(.5f);
	for (int n = 0; n < 20; ++n) {
		const auto r = core.step(in); ASSERT_TRUE(r.transaction_valid);
		in.current_native_i += r.accepted_delta_i;
		EXPECT_LE(std::fabs(r.b_after), .1f); EXPECT_TRUE(r.limited_by_bmax);
	}
	EXPECT_NEAR(core.state(), .1f, Tol);
}
TEST(AdaptiveTailTrimCore, DirectionalReservePositiveBias) {
	Core core(config(), .1f); auto in = input(.1f); in.tail_pitch_context = .3f;
	const auto r = core.step(in);
	EXPECT_NEAR(r.roll_reserve_pos, .6f, Tol); EXPECT_NEAR(r.roll_reserve_neg, .8f, Tol);
	EXPECT_NEAR(r.roll_reserve_sym, .6f, Tol);
}
TEST(AdaptiveTailTrimCore, DirectionalReserveNegativeBias) {
	Core core(config(), -.1f); auto in = input(-.1f); in.tail_pitch_context = -.3f;
	const auto r = core.step(in);
	EXPECT_NEAR(r.roll_reserve_pos, .8f, Tol); EXPECT_NEAR(r.roll_reserve_neg, .6f, Tol);
}
TEST(AdaptiveTailTrimCore, SharedAxisBlocksGrowth) {
	auto cfg = config(); cfg.reserve_pos_min = cfg.reserve_neg_min = .15f;
	for (float pitch : {.9f, -.9f, 1.f}) {
		Core core(cfg); auto in = input(.08f); in.tail_pitch_context = pitch;
		const auto r = core.step(in); ASSERT_TRUE(r.transaction_valid);
		EXPECT_FLOAT_EQ(r.b_after, 0.f); EXPECT_TRUE(r.limited_by_shared_axis); EXPECT_FALSE(r.feasible);
	}
}
TEST(AdaptiveTailTrimCore, SharedAxisAllowsGrowth) {
	Core core(config(), .04f); auto in = input(.1f); in.tail_pitch_context = .4f;
	const auto r = core.step(in); ASSERT_TRUE(r.transaction_valid);
	EXPECT_GT(r.b_after, .04f); EXPECT_TRUE(r.feasible); invariant(r);
	Core at_cap(config(), .1f); in.target_b = .1f;
	EXPECT_TRUE(at_cap.step(in).feasible);
}
TEST(AdaptiveTailTrimCore, GrowthRequiresGate) {
	Core core(config(), .04f); auto in = input(); in.gate = false;
	const auto r = core.step(in);
	EXPECT_FLOAT_EQ(r.b_after, .04f); EXPECT_TRUE(r.growth_blocked_by_gate);
}
TEST(AdaptiveTailTrimCore, NormalManeuverFreezesGrowth) {
	Core core(config(), .04f); auto in = input(); in.maneuver_active = true; in.learning_allowed = false;
	for (int n = 0; n < 300; ++n) {
		const auto r = core.step(in); EXPECT_FLOAT_EQ(r.b_after, .04f);
		EXPECT_TRUE(r.growth_blocked_by_maneuver); EXPECT_FLOAT_EQ(r.accepted_delta_i, 0.f);
	}
}
TEST(AdaptiveTailTrimCore, NormalManeuverFreezesNormalRelease) {
	Core core(config(), .06f); auto in = input(.02f); in.maneuver_active = true;
	for (int n = 0; n < 300; ++n) { EXPECT_FLOAT_EQ(core.step(in).b_after, .06f); }
}
TEST(AdaptiveTailTrimCore, SafetyReleaseOverridesManeuverFreeze) {
	for (bool explicit_safety : {false, true}) {
		Core core(config(), .06f); auto in = input(.08f); in.maneuver_active = true;
		in.safety_release_required = explicit_safety; in.tail_pitch_context = explicit_safety ? 0.f : .9f;
		const auto r = core.step(in); EXPECT_TRUE(r.safety_release_active); EXPECT_LT(r.b_after, .06f);
	}
}
TEST(AdaptiveTailTrimCore, PartialAcceptedTransfer) {
	auto cfg = config(); cfg.reserve_pos_min = .596f; cfg.reserve_neg_min = .2f;
	Core core(cfg); auto in = input(.01f); in.tail_pitch_context = .4f;
	const auto r = core.step(in); ASSERT_TRUE(r.transaction_valid);
	EXPECT_NEAR(r.requested_delta_b, .01f, Tol); EXPECT_NEAR(r.accepted_delta_b, .004f, Tol);
	EXPECT_NEAR(r.accepted_delta_i, -(1.1f/.7f)*.004f, Tol);
	EXPECT_TRUE(r.limited_by_shared_axis); invariant(r);
}
TEST(AdaptiveTailTrimCore, BumplessPhysicalInvariant) {
	for (float sign : {-1.f, 1.f}) {
		Core core(config()); auto in = input(sign*.01f);
		const auto r = core.step(in); ASSERT_TRUE(r.transaction_valid);
		EXPECT_NEAR(r.accepted_delta_b, sign*.01f, Tol); invariant(r);
	}
}
TEST(AdaptiveTailTrimCore, NativeICapacityLimitsTransfer) {
	Core core(config()); auto in = input(.01f); in.native_i_min = -.003f;
	const auto r = core.step(in); ASSERT_TRUE(r.transaction_valid);
	EXPECT_TRUE(r.limited_by_i); EXPECT_GT(r.accepted_delta_b, 0.f);
	EXPECT_NEAR(r.accepted_delta_b, .003f*.7f/1.1f, Tol);
	EXPECT_GE(in.current_native_i+r.accepted_delta_i, in.native_i_min); invariant(r);
}
TEST(AdaptiveTailTrimCore, SmallGRejected) {
	for (float g : {0.f, -1.f, 1e-30f, .01f}) {
		Core core(config(), .04f); auto in = input(.08f); in.g_cycle = g;
		const auto r = core.step(in); EXPECT_FALSE(r.transaction_valid); EXPECT_TRUE(r.recovery_required);
		EXPECT_FLOAT_EQ(r.accepted_delta_b, 0.f); EXPECT_FLOAT_EQ(r.accepted_delta_i, 0.f);
		EXPECT_FLOAT_EQ(r.b_after, .04f);
	}
}
TEST(AdaptiveTailTrimCore, NaNInfRejected) {
	for (float bad : {NAN, INFINITY, -INFINITY}) {
		for (int field = 0; field < 5; ++field) {
			Core core(config(), .04f); auto in = input();
			float *values[] = {&in.g_cycle, &in.dt, &in.tail_pitch_context, &in.target_b, &in.current_native_i};
			*values[field] = bad; const auto r = core.step(in);
			EXPECT_FALSE(r.transaction_valid); EXPECT_TRUE(r.recovery_required);
			EXPECT_FLOAT_EQ(r.accepted_delta_b, 0.f); EXPECT_FLOAT_EQ(r.accepted_delta_i, 0.f);
			EXPECT_TRUE(std::isfinite(r.b_after)); EXPECT_TRUE(std::isfinite(r.tau_trim));
		}
	}
}
TEST(AdaptiveTailTrimCore, SlewBound) {
	auto cfg = config(); cfg.b_slew = .01f; Core core(cfg); auto in = input();
	const auto r = core.step(in); ASSERT_TRUE(r.transaction_valid);
	EXPECT_NEAR(r.accepted_delta_b, .0002f, Tol); EXPECT_TRUE(r.limited_by_slew);
}
TEST(AdaptiveTailTrimCore, ZeroFirstReversal) {
	Core core(config(), .025f); auto in = input(-.08f); in.reversal_unwind_latched = true;
	bool reached = false;
	for (int n = 0; n < 10; ++n) {
		const auto r = core.step(in); in.current_native_i += r.accepted_delta_i;
		EXPECT_GE(r.b_after, 0.f); reached |= r.reversal_reached_zero;
	}
	EXPECT_TRUE(reached); EXPECT_FLOAT_EQ(core.state(), 0.f);
}
TEST(AdaptiveTailTrimCore, NoCrossingBeforeZero) {
	Core core(config(), .004f); auto in = input(-.08f); in.reversal_unwind_latched = true;
	const auto r = core.step(in); ASSERT_TRUE(r.transaction_valid);
	EXPECT_FLOAT_EQ(r.b_after, 0.f); EXPECT_TRUE(r.reversal_reached_zero); invariant(r);
	// Stale true gate/latch cannot initiate opposite growth.
	EXPECT_FLOAT_EQ(core.step(in).b_after, 0.f);
}
TEST(AdaptiveTailTrimCore, SafetyExitCanDecayWithoutExactHandback) {
	Core core(config(), .05f); auto in = input(.05f);
	in.current_native_i = in.native_i_max; in.safety_release_required = true;
	const auto r = core.step(in); ASSERT_TRUE(r.transaction_valid);
	EXPECT_TRUE(r.safety_release_active); EXPECT_TRUE(r.limited_by_i);
	EXPECT_LT(r.b_after, .05f); EXPECT_FLOAT_EQ(r.accepted_delta_i, 0.f);
	EXPECT_LE(std::fabs(r.transfer_mismatch), 1.1f*std::fabs(r.accepted_delta_b)+TorqueTol);
}
TEST(AdaptiveTailTrimCore, ResetLeavesNoHiddenTrim) {
	Core core(config(), .05f); auto in = input(); in.reset_epoch = 1; in.g_cycle = NAN;
	const auto r = core.step(in); EXPECT_TRUE(r.reset_applied); EXPECT_FLOAT_EQ(core.state(), 0.f);
	EXPECT_FLOAT_EQ(r.b_after, 0.f); EXPECT_FALSE(r.transaction_valid);
}
TEST(AdaptiveTailTrimCore, FrozenGRecorded) {
	float external_gain = .7f; auto snapshot = input(.01f); snapshot.g_cycle = external_gain;
	external_gain = .8f;
	Core core(config()); const auto r = core.step(snapshot);
	ASSERT_TRUE(r.transaction_valid); EXPECT_FLOAT_EQ(r.g_used, .7f);
	EXPECT_NEAR(r.accepted_delta_i, -1.1f*.01f/.7f, Tol);
	EXPECT_GT(std::fabs(external_gain*r.accepted_delta_i+1.1f*r.accepted_delta_b), .001f);
	invariant(r);
}
TEST(AdaptiveTailTrimCore, TransferMismatchNearZeroNormalCase) {
	double maximum = 0.;
	for (float gain : {.05f, .5f, .7f, 1.2f, 2.f}) {
		for (float target : {-.1f, -.02f, .02f, .1f}) {
			Core core(config()); auto in = input(target); in.g_cycle = gain;
			for (int n = 0; n < 20; ++n) {
				const auto r = core.step(in); ASSERT_TRUE(r.transaction_valid); invariant(r);
				maximum = fmax(maximum, std::fabs(static_cast<double>(r.transfer_mismatch)));
				in.current_native_i += r.accepted_delta_i;
			}
		}
	}
	std::printf("MAX_NORMAL_PHYSICAL_MISMATCH=%.12g\n", maximum);
}
TEST(AdaptiveTailTrimCore, TransferMismatchFiniteSafetyCase) {
	double maximum = 0.;
	for (float sign : {-1.f, 1.f}) {
		Core core(config(), sign*.05f); auto in = input(sign*.05f);
		in.current_native_i = sign*.2f; in.safety_release_required = true;
		for (int n = 0; n < 10; ++n) {
			const auto r = core.step(in); ASSERT_TRUE(r.transaction_valid);
			EXPECT_TRUE(std::isfinite(r.transfer_mismatch)); EXPECT_GE(sign*r.b_after, 0.f);
			EXPECT_LE(std::fabs(r.transfer_mismatch), 1.1f*std::fabs(r.accepted_delta_b)+TorqueTol);
			maximum = fmax(maximum, std::fabs(static_cast<double>(r.transfer_mismatch)));
		}
		EXPECT_FLOAT_EQ(core.state(), 0.f);
	}
	std::printf("MAX_SAFETY_PHYSICAL_MISMATCH=%.12g\n", maximum);
}
TEST(AdaptiveTailTrimCore, NoNewManeuverReversalAndPriorLatchUnwinds) {
	Core frozen(config(), .04f); auto in = input(-.08f);
	in.maneuver_active = true; in.reversal_unwind_latched = true;
	EXPECT_FLOAT_EQ(frozen.step(in).b_after, .04f);
	Core latched(config(), .04f); in.maneuver_active = false;
	ASSERT_TRUE(latched.step(in).reversal_unwind_active);
	in.maneuver_active = true; in.reversal_unwind_latched = false;
	for (int n = 0; n < 20; ++n) {
		const auto r = latched.step(in); EXPECT_GE(r.b_after, 0.f);
	}
	EXPECT_FLOAT_EQ(latched.state(), 0.f);
}
TEST(AdaptiveTailTrimCore, GeometrySweepNeverOptimisticallyFeasible) {
	for (float pitch : {0.f, .3f, .4f, .8f, .9f, 1.f, -1.f}) {
		for (float b : {-.1f, -.04f, 0.f, .04f, .1f}) {
			for (float target : {-.1f, 0.f, .1f}) {
				Core core(config(), b); auto in = input(target); in.tail_pitch_context = pitch;
				const auto r = core.step(in); ASSERT_TRUE(r.transaction_valid);
				const double pos = 1. - std::fabs(static_cast<double>(pitch)) - static_cast<double>(r.b_after);
				const double neg = 1. - std::fabs(static_cast<double>(pitch)) + static_cast<double>(r.b_after);
				EXPECT_NEAR(r.roll_reserve_pos, pos, Tol); EXPECT_NEAR(r.roll_reserve_neg, neg, Tol);
				EXPECT_EQ(r.feasible, pos >= static_cast<double>(config().reserve_pos_min)
					  && neg >= static_cast<double>(config().reserve_neg_min));
			}
		}
	}
}

TEST(AdaptiveTailTrimCore, InvalidConfigAndInitialStateRequireRecovery) {
	for (int field = 0; field < 6; ++field) {
		auto cfg = config();
		float *values[] = {&cfg.b_max, &cfg.b_slew, &cfg.g_safe_min, &cfg.reserve_pos_min, &cfg.reserve_neg_min, &cfg.k_a};
		*values[field] = NAN;
		Core core(cfg, .05f); const auto r = core.step(input());
		EXPECT_FALSE(r.transaction_valid); EXPECT_TRUE(r.recovery_required);
		EXPECT_TRUE(std::isfinite(r.b_after)); EXPECT_TRUE(std::isfinite(r.tau_trim));
	}
	for (float b : {NAN, INFINITY, .11f, -.11f}) {
		Core core(config(), b); auto in = input();
		EXPECT_FALSE(core.step(in).transaction_valid); EXPECT_FLOAT_EQ(core.state(), 0.f);
		in.reset = true; EXPECT_TRUE(core.step(in).reset_applied);
		in.reset = false; EXPECT_TRUE(core.step(in).transaction_valid);
	}
}
TEST(AdaptiveTailTrimCore, NativeBoundsAndInvalidDtRejectWithoutMovement) {
	for (int scenario = 0; scenario < 5; ++scenario) {
		Core core(config(), .04f); auto in = input();
		if (scenario == 0) { in.native_i_min = .1f; }
		if (scenario == 1) { in.native_i_max = -.1f; }
		if (scenario == 2) { in.dt = 0.f; }
		if (scenario == 3) { in.dt = -.01f; }
		if (scenario == 4) { in.input_valid = false; }
		const auto r = core.step(in); EXPECT_FALSE(r.transaction_valid);
		EXPECT_FLOAT_EQ(r.b_after, .04f); EXPECT_FLOAT_EQ(r.accepted_delta_i, 0.f);
	}
}
TEST(AdaptiveTailTrimCore, NormalReleaseWithoutGateAndLearningDisabledHold) {
	Core core(config(), .06f); auto in = input(.02f); in.gate = false;
	const auto r = core.step(in); EXPECT_LT(r.b_after, .06f); invariant(r);
	Core held(config(), .06f); in.learning_allowed = false;
	EXPECT_FLOAT_EQ(held.step(in).b_after, .06f);
}
TEST(AdaptiveTailTrimCore, ReversalRearmRequiresFreshGateHandshake) {
	Core core(config(), .004f); auto in = input(-.08f); in.reversal_unwind_latched = true;
	ASSERT_TRUE(core.step(in).reversal_reached_zero);
	in.reversal_unwind_latched = false;
	EXPECT_FLOAT_EQ(core.step(in).b_after, 0.f); // old true gate
	in.gate = false; EXPECT_FLOAT_EQ(core.step(in).b_after, 0.f);
	in.gate = true; EXPECT_LT(core.step(in).b_after, 0.f);
}
TEST(AdaptiveTailTrimCore, IndependentDirectionalMinimaLimitBothSigns) {
	auto cfg = config(); cfg.reserve_pos_min = .58f; cfg.reserve_neg_min = .55f;
	for (float sign : {-1.f, 1.f}) {
		Core core(cfg); auto in = input(sign*.1f); in.tail_pitch_context = .4f;
		for (int n = 0; n < 12; ++n) {
			const auto r = core.step(in); ASSERT_TRUE(r.transaction_valid);
			in.current_native_i += r.accepted_delta_i;
			EXPECT_TRUE(r.feasible); invariant(r);
		}
		EXPECT_NEAR(core.state(), sign > 0.f ? .02f : -.05f, Tol);
	}
}
