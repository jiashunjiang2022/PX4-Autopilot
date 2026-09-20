// TEST-FIRST: intentional opt-in link RED. Subject has declarations only.
// No fake estimator, transfer algorithm, or state machine is supplied.
// Tolerance frozen before execution: absolute 2e-7 for single-step float math.
// Production RateControl bounds assertions are inherited checks, not V4 support.
#include <gtest/gtest.h>
#include <lib/rate_control/rate_control.hpp>
#include <cmath>
#include <algorithm>
#include "ResidualSlowFeedforwardMemoryContract.hpp"
using namespace v4_contract;
namespace
{
constexpr float E = 2e-7f;
Input fresh(uint64_t t)
{
	Input in;
	in.time = t;
	in.allocator_time = t;
	return in;
}
void warm(Subject &x, float i)
{
	for (uint64_t t = 50000; t <= 3050000; t += 50000)
		x.sample(fresh(t), i);
}
void pair(const Subject &x, float b, float i, float total)
{
	const auto v = x.state();
	EXPECT_NEAR(v.b, b, E);
	EXPECT_NEAR(v.i, i, E);
	EXPECT_NEAR(v.b + v.i, total, E);
	EXPECT_LE(std::fabs(v.b), v.bmax + E);
	EXPECT_LE(std::fabs(v.i), v.imax + E);
	EXPECT_LE(std::fabs(v.b + v.i), v.imax + E);
}
void reset(const Subject &x)
{
	const auto v = x.state();
	EXPECT_TRUE(v.recovery);
	EXPECT_FLOAT_EQ(v.b, 0.f);
	EXPECT_FLOAT_EQ(v.i, 0.f);
	EXPECT_FALSE(v.estimate_valid);
	EXPECT_FALSE(v.gate_valid);
	EXPECT_FALSE(v.learn);
	EXPECT_EQ(v.samples, 0u);
}
}
TEST(ResidualSlowFeedforwardMemory, V4UT01_NativeDisabledEquivalence)
{
	State z;
	Subject x(z);
	Input in;
	in.enabled = false;
	x.tick(in);
	RateControl native;
	native.setPidGains(matrix::Vector3f(.12f, 0.f, 0.f), matrix::Vector3f(), matrix::Vector3f());
	native.setFeedForwardGain(matrix::Vector3f());
	const auto raw = native.update(matrix::Vector3f(), matrix::Vector3f(1.f, 0.f, 0.f), matrix::Vector3f(), .05f, true);
	EXPECT_FLOAT_EQ(x.state().b, 0.f);
	EXPECT_NEAR(x.output(raw(0), .7f, .02f), .7f * raw(0) + .02f, E);
}

TEST(ResidualSlowFeedforwardMemory, V4UT02_PositiveAcceptedTransfer)
{
	State z;
	z.b = .08f;
	z.i = .10f;
	Subject x(z);
	x.requestPair(.01f);
	pair(x, .09f, .09f, .18f);
}

TEST(ResidualSlowFeedforwardMemory, V4UT03_RequestedNotAccepted)
{
	State z;
	z.b = .095f;
	z.i = .085f;
	z.q_hat = .18f;
	Subject x(z);
	x.requestPair(.02f);
	pair(x, .10f, .080f, .180f);
	EXPECT_NEAR(x.state().accepted, .005f, E);
	EXPECT_NEAR(x.state().q_hat, .18f, E);
	EXPECT_NEAR(x.state().residual, .08f, E);
}

TEST(ResidualSlowFeedforwardMemory, V4UT04_PositiveResidualBounds)
{
	auto v = RateControl::computeRollILimits(.20f, .10f, 0.f);
	ASSERT_TRUE(v.valid);
	EXPECT_NEAR(v.lower, -.20f, E);
	EXPECT_NEAR(v.upper, .10f, E);
}

TEST(ResidualSlowFeedforwardMemory, V4UT05_NegativeResidualBounds)
{
	auto v = RateControl::computeRollILimits(.20f, -.10f, 0.f);
	ASSERT_TRUE(v.valid);
	EXPECT_NEAR(v.lower, -.10f, E);
	EXPECT_NEAR(v.upper, .20f, E);
}

TEST(ResidualSlowFeedforwardMemory, V4UT06_TotalIMAXPositive)
{
	State z;
	z.b = .10f;
	z.i = .10f;
	Subject x(z);
	x.naturalIntegrate(1.f, .05f);
	EXPECT_LE(x.state().i, .10f + E);
	EXPECT_LE(x.state().i + x.state().b, .20f + E);
}

TEST(ResidualSlowFeedforwardMemory, V4UT07_TotalIMAXNegative)
{
	State z;
	z.b = -.10f;
	z.i = -.10f;
	Subject x(z);
	x.naturalIntegrate(-1.f, .05f);
	EXPECT_GE(x.state().i, -.10f - E);
	EXPECT_GE(x.state().i + x.state().b, -.20f - E);
}

TEST(ResidualSlowFeedforwardMemory, V4UT08_CapResidualVisible)
{
	State z;
	z.b = .10f;
	z.i = .08f;
	z.q_hat = .18f;
	Subject x(z);
	warm(x, .08f);
	EXPECT_NEAR(x.state().b, .10f, E);
	EXPECT_NEAR(x.state().residual, .08f, E);
	EXPECT_NEAR(x.state().innovation, .03f, E);
	EXPECT_NEAR(x.state().accepted, 0.f, E);
}

TEST(ResidualSlowFeedforwardMemory, V4UT09_DeadbandHoldsMemory)
{
	State z;
	z.b = .06f;
	z.i = .03f;
	z.q_hat = .09f;
	Subject x(z);
	warm(x, .03f);
	EXPECT_NEAR(x.state().innovation, 0.f, E);
	EXPECT_NEAR(x.state().b, .06f, E);
}

TEST(ResidualSlowFeedforwardMemory, V4UT10_DuplicateTimestamp)
{
	State z;
	z.i = .1f;
	z.q_hat = .1f;
	Subject x(z);
	warm(x, .1f);
	const auto before = x.state();
	auto in = fresh(3050000);
	for (int j = 0; j < 100; ++j)
		x.sample(in, .1f);
	EXPECT_EQ(x.state().samples, before.samples);
	EXPECT_FLOAT_EQ(x.state().q_hat, before.q_hat);
	EXPECT_FLOAT_EQ(x.state().b, before.b);
}

TEST(ResidualSlowFeedforwardMemory, V4UT11_GapInvalidatesGate)
{
	State z;
	z.i = .1f;
	z.q_hat = .1f;
	Subject x(z);
	warm(x, .1f);
	ASSERT_TRUE(x.state().gate_valid);
	const float b = x.state().b;
	x.sample(fresh(3350000), .1f);
	EXPECT_FALSE(x.state().gate_valid);
	EXPECT_FALSE(x.state().learn);
	EXPECT_FLOAT_EQ(x.state().b, b);
	x.sample(fresh(3400000), .1f);
	EXPECT_FALSE(x.state().gate_valid);
}

TEST(ResidualSlowFeedforwardMemory, V4UT12_StaleAllocatorBothDirections)
{
	for (float sign : {
	         -1.f, 1.f
	     }) {
		State z;
		z.b = sign * .04f;
		z.i = sign * .1f;
		z.q_hat = z.b + z.i;
		Subject x(z);
		warm(x, z.i);
		const float b = x.state().b;
		auto in = fresh(3100000);
		in.allocator_time -= 200000;
		x.sample(in, z.i);
		EXPECT_FALSE(x.state().learn);
		EXPECT_FLOAT_EQ(x.state().b, b);
		x.sample(fresh(3150000), z.i);
		EXPECT_FALSE(x.state().gate_valid);
	}
}

TEST(ResidualSlowFeedforwardMemory, V4UT13_SaturationBothDirections)
{
	for (float sign : {
	         -1.f, 1.f
	     }) {
		State z;
		z.i = sign * .1f;
		z.q_hat = z.i;
		Subject x(z);
		for (uint64_t t = 50000; t <= 3100000; t += 50000) {
			auto in = fresh(t);
			in.positive_sat = sign > 0;
			in.negative_sat = sign < 0;
			x.sample(in, z.i);
		}
		EXPECT_FLOAT_EQ(x.state().b, 0.f);
	}
}

TEST(ResidualSlowFeedforwardMemory, V4UT14_CommonCoordinateHistory)
{
	State z;
	z.b = .04f;
	z.i = .08f;
	z.history = {{.11f, .12f, .13f}};
	Subject x(z);
	x.recomputeHistoryStatistics();
	const auto before = x.state();
	x.requestPair(.02f);
	x.recomputeHistoryStatistics();
	EXPECT_EQ(x.state().history, before.history);
	EXPECT_NEAR(x.state().b, .06f, E);
	for (unsigned j = 0; j < 3; ++j)
		EXPECT_NEAR(x.state().history[j] - x.state().b, .05f + .01f * j, E);
	EXPECT_NEAR(x.state().stddev, before.stddev, E);
}

TEST(ResidualSlowFeedforwardMemory, V4UT15_RebaseNotEvidence)
{
	State z;
	z.b = .04f;
	z.i = .08f;
	Subject x(z);
	const auto before = x.state();
	x.requestPair(.02f);
	EXPECT_EQ(x.state().samples, before.samples);
	EXPECT_EQ(x.state().gate_time, before.gate_time);
	const float b = x.state().b;
	x.tick(fresh(0));
	EXPECT_FLOAT_EQ(x.state().b, b);
	EXPECT_FALSE(x.state().learn);
}

TEST(ResidualSlowFeedforwardMemory, V4UT16_PartialReverseStops)
{
	State z;
	z.b = .08f;
	z.i = -.02f;
	z.q_hat = .01f;
	Subject x(z);
// Accepted correction brings residual estimate into deadband: .01-.06=-.05.
	x.requestPair(-.02f);
	pair(x, .06f, 0.f, .06f);
	x.tick(fresh(0));
	EXPECT_NEAR(x.state().b, .06f, E);
	warm(x, 0.f);
	EXPECT_NEAR(x.state().b, .06f, E);
}

TEST(ResidualSlowFeedforwardMemory, V4UT17_ZeroCrossingBarrier)
{
	State z;
	z.b = .005f;
	z.i = -.1f;
	Subject x(z);
	x.requestPair(-.02f);
	EXPECT_NEAR(x.state().b, 0.f, E);
	EXPECT_GE(x.state().b, 0.f);
	EXPECT_FALSE(x.state().gate_valid);
}

TEST(ResidualSlowFeedforwardMemory, V4UT18_FreshReverseWindow)
{
	State z;
	z.b = .005f;
	z.i = -.1f;
	z.q_hat = -.095f;
	Subject x(z);
	x.requestPair(-.005f);
	for (uint64_t t = 50000; t <= 2950000; t += 50000)
		x.sample(fresh(t), -.1f);
	EXPECT_FLOAT_EQ(x.state().b, 0.f);
	EXPECT_FALSE(x.state().gate_valid);
}

TEST(ResidualSlowFeedforwardMemory, V4UT19_ManeuverHoldApply)
{
	State z;
	z.b = .08f;
	z.i = .06f;
	z.q_hat = .14f;
	Subject x(z);
	auto in = fresh(50000);
	in.maneuver = true;
	x.sample(in, .06f);
	EXPECT_FALSE(x.state().learn);
	EXPECT_FLOAT_EQ(x.state().b, .08f);
	EXPECT_TRUE(x.state().apply);
}

TEST(ResidualSlowFeedforwardMemory, V4UT20_IMAXShrinkRecovery)
{
	State z;
	z.b = .10f;
	z.i = .10f;
	Subject x(z);
	x.limits(.15f, .10f);
	reset(x);
}

TEST(ResidualSlowFeedforwardMemory, V4UT21_BMAXShrinkRecovery)
{
	State z;
	z.b = .10f;
	z.i = .06f;
	Subject x(z);
	x.limits(.20f, .05f);
	reset(x);
}

TEST(ResidualSlowFeedforwardMemory, V4UT22_NonfiniteState)
{
	for (int field = 0; field < 3; ++field) {
		State z;
		z.b = .04f;
		z.i = .06f;
		z.q_hat = .1f;
		if (field == 0)
			z.b = NAN;
		if (field == 1)
			z.i = NAN;
		if (field == 2)
			z.q_hat = NAN;
		Subject x(z);
		x.tick(fresh(50000));
		EXPECT_FALSE(x.state().learn);
		if (field < 2)
			reset(x);
		else {
			EXPECT_FALSE(x.state().estimate_valid);
			EXPECT_FALSE(x.state().gate_valid);
		}
	}
}

TEST(ResidualSlowFeedforwardMemory, V4UT23_EpochMismatch)
{
	State z;
	z.b = .08f;
	z.i = .06f;
	Subject x(z);
	x.externalReset();
	x.tick(fresh(50000));
	reset(x);
	EXPECT_EQ(x.state().epoch, x.state().native_epoch);
}

TEST(ResidualSlowFeedforwardMemory, V4UT24_LegalPairedHandback)
{
	for (float sign : {
	         -1.f, 1.f
	     }) {
		State z;
		z.b = sign * .08f;
		z.i = sign * .06f;
		Subject x(z);
		for (int j = 0; j < 400; ++j) {
			x.disableStep(.05f);
			EXPECT_NEAR(x.state().b + x.state().i, sign * .14f, E);
		}
		pair(x, 0.f, sign * .14f, sign * .14f);
	}
}

TEST(ResidualSlowFeedforwardMemory, V4UT25_ControlCompositionBumpless)
{
	State z;
	z.b = .08f;
	z.i = .10f;
	Subject x(z);
	const float before = x.output(.12f, .7f, .02f);
	EXPECT_NEAR(before, .23f, E);
	x.requestPair(.01f);
	EXPECT_NEAR(x.output(.12f, .7f, .02f), before, E);
	pair(x, .09f, .09f, .18f);
}

TEST(ResidualSlowFeedforwardMemory, V4UT26_NoHiddenHR)
{
	State z;
	z.b = .1f;
	z.i = .1f;
	z.legacy_hr = 1.f;
	Subject x(z);
	x.naturalIntegrate(1.f, .05f);
	EXPECT_LE(x.state().b + x.state().i, .20f + E);
}

TEST(ResidualSlowFeedforwardMemory, V4UT27_BackwardTimestamp)
{
	State z;
	z.i = .1f;
	z.q_hat = .1f;
	Subject x(z);
	warm(x, .1f);
	const auto before = x.state();
	x.sample(fresh(3000000), .15f);
	EXPECT_EQ(x.state().samples, before.samples);
	EXPECT_FLOAT_EQ(x.state().q_hat, before.q_hat);
	EXPECT_FLOAT_EQ(x.state().b, before.b);
}
TEST(ResidualSlowFeedforwardMemory, V4UT28_GainedIncrementAndSlew)
{
	for (float gain : {
	         .1f, 100.f
	     }) {
		State z;
		z.i = .1f;
		z.q_hat = .1f;
		z.gain = gain;
		Subject x(z);
		warm(x, .1f);
		const auto before = x.state();
		const float q = .1f + before.b;
		const float q_next = before.q_hat + .05f / (z.tau + .05f) * (q - before.q_hat);
		const float innovation = std::max(q_next - before.b - z.reserve, 0.f);
		const float expected = std::min(gain * innovation * .05f, z.slew * .05f);
		ASSERT_GT(expected, 0.f);
		x.sample(fresh(3100000), .1f);
		EXPECT_NEAR(x.state().b - before.b, expected, E);
	}
}
TEST(ResidualSlowFeedforwardMemory, V4UT29_SignFractionAfterRebase)
{
	State z;
	z.b = .10f;
	z.i = 0.f;
	z.history = {{.09f, .10f, .11f}};
	Subject x(z);
	x.recomputeHistoryStatistics();
	EXPECT_NEAR(x.state().positive_fraction, 1.f / 3.f, E);
	x.requestPair(-.02f);
	x.recomputeHistoryStatistics();
	EXPECT_NEAR(x.state().positive_fraction, 1.f, E);
}
TEST(ResidualSlowFeedforwardMemory, V4UT30_StabilizedCannotLearn)
{
	State z;
	z.i = .1f;
	z.q_hat = .1f;
	Subject x(z);
	for (uint64_t t = 50000; t <= 4000000; t += 50000) {
		auto in = fresh(t);
		in.mission = false;
		x.sample(in, .1f);
	}
	EXPECT_FLOAT_EQ(x.state().b, 0.f);
	EXPECT_FALSE(x.state().learn);
}
TEST(ResidualSlowFeedforwardMemory, V4UT31_RecoveryRequiresWarmup)
{
	State z;
	z.b = .1f;
	z.i = .1f;
	Subject x(z);
	x.limits(.15f, .1f);
	reset(x);
	for (uint64_t t = 50000; t <= 2950000; t += 50000)
		x.sample(fresh(t), .08f);
	EXPECT_FALSE(x.state().gate_valid);
	EXPECT_FLOAT_EQ(x.state().b, 0.f);
}
