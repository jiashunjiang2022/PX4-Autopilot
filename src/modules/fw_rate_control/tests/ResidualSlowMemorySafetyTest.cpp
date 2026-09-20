// SPDX-License-Identifier: BSD-3-Clause
// Additional production-boundary tests; original 31 frozen oracles unchanged.
#include <gtest/gtest.h>
#include "ResidualSlowFeedforwardMemoryContract.hpp"
#include <limits>

namespace
{
using Core = ResidualSlowFeedforwardMemory;
using v4_contract::Subject;
using v4_contract::State;
using v4_contract::Input;
constexpr float E = 2e-7f;
Input fresh(uint64_t t)
{
	Input in;
	in.time = t;
	in.allocator_time = t;
	return in;
}
void warm(Subject &s, float i)
{
	for (uint64_t t = 50000; t <= 3050000; t += 50000) {
		s.sample(fresh(t), i);
	}
}
void seed(RateControl &rc, float i)
{
	rc.setIntegratorLimit(matrix::Vector3f(.2f, .3f, .4f));
	ASSERT_TRUE(rc.applyRollITransfer({i, 0.f, RateControl::RollITransferMode::PairPreserving}).valid);
}
Core::Inputs coreInput(uint64_t t)
{
	Core::Inputs in;
	in.now = in.timestamp = t;
	in.fresh_sample = true;
	in.allocator_fresh = true;
	in.enabled = true;
	in.mission = true;
	in.control_dt = .05f;
	return in;
}
}
TEST(ResidualSlowMemorySafety, JointCommitRejectsWithoutEitherMutation)
{
	RateControl rc;
	seed(rc, .1f);
	float b = .08f;
	const auto epoch = rc.rollIntegralResetEpoch();
	for (int fault = 0; fault < 7; ++fault) {
		const float bp = fault == 0 ? .11f : (fault == 1 ? NAN : .09f);
		const float ip = fault == 2 ? .10f : .09f;
		EXPECT_FALSE(rc.commitRollMemoryPair(b, fault == 3 ? .07f : .08f,
		                                     fault == 4 ? .11f : .1f, fault == 5 ? epoch + 1 : epoch,
		                                     fault == 6 ? .15f : .2f, bp, ip, .1f));
		EXPECT_FLOAT_EQ(b, .08f);
		EXPECT_FLOAT_EQ(rc.rollIntegralRaw(), .1f);
	}
}
TEST(ResidualSlowMemorySafety, JointCommitAcceptsExactPairAndLeavesOtherAxes)
{
	RateControl rc;
	seed(rc, .085f);
	float b = .095f;
	EXPECT_TRUE(rc.commitRollMemoryPair(b, .095f, .085f, rc.rollIntegralResetEpoch(), .2f, .1f, .08f, .1f));
	EXPECT_FLOAT_EQ(b, .1f);
	EXPECT_FLOAT_EQ(rc.rollIntegralRaw(), .08f);
	rate_ctrl_status_s status{};
	rc.getRateControlStatus(status);
	EXPECT_FLOAT_EQ(status.pitchspeed_integ, 0.f);
	EXPECT_FLOAT_EQ(status.yawspeed_integ, 0.f);
}
TEST(ResidualSlowMemorySafety, NoPublicationTimeoutAndBoundary)
{
	State z;
	z.i = z.q_hat = .1f;
	Subject x(z);
	warm(x, .1f);
	ASSERT_TRUE(x.state().gate_valid);
	const float b = x.state().b;
	x.tick(fresh(3200000));
	EXPECT_TRUE(x.state().gate_valid); // exactly .15s
	x.tick(fresh(3200001));
	EXPECT_FALSE(x.state().gate_valid);
	EXPECT_FALSE(x.state().learn);
	EXPECT_FLOAT_EQ(x.state().b, b);
	EXPECT_EQ(x.state().evidence_time, 3050000u);
}
TEST(ResidualSlowMemorySafety, AllocatorUnknownRecoveryNeedsFullRealWindow)
{
	State z;
	z.i = z.q_hat = .1f;
	Subject x(z);
	warm(x, .1f);
	auto stale = fresh(3100000);
	stale.allocator_time -= 100001;
	x.sample(stale, .1f);
	const float b = x.state().b;
	for (uint64_t t = 3150000; t <= 6100000; t += 50000) {
		x.sample(fresh(t), .1f);
		EXPECT_FALSE(x.state().gate_valid);
		EXPECT_FLOAT_EQ(x.state().b, b);
	}
	x.sample(fresh(6150000), .1f);
	EXPECT_TRUE(x.state().gate_valid);
	EXPECT_GT(x.state().b, b);
}
TEST(ResidualSlowMemorySafety, FullNegativeReversalRequiresWindowAfterZero)
{
	State z;
	z.b = .08f;
	z.i = -.18f;
	z.q_hat = -.10f;
	Subject x(z);
	uint64_t zero_time = 0;
	bool negative = false;
	for (uint64_t t = 50000; t <= 18000000; t += 50000) {
		const float before = x.state().b;
		x.sample(fresh(t), -.10f - before); // Prescribed plant evidence, total=-.10.
		const auto s = x.state();
		EXPECT_NEAR(s.i + s.b, -.10f, E);
		if (before > 0.f) {
			EXPECT_GE(s.b, 0.f);
		}
		if (zero_time == 0 && s.b == 0.f) {
			zero_time = t;
			EXPECT_FALSE(s.gate_valid);
		}
		if (zero_time && t - zero_time <= 3000000) {
			EXPECT_GE(s.b, 0.f);
		}
		if (s.b < 0.f) {
			EXPECT_TRUE(s.gate_valid);
			negative = true;
			break;
		}
	}
	EXPECT_GT(zero_time, 0u);
	EXPECT_TRUE(negative);
}
TEST(ResidualSlowMemorySafety, PilotAbortAndFailsafePairedHandback)
{
	for (bool abort : {
	         false, true
	     }) {
		RateControl rc;
		seed(rc, .18f);
		Core core(rc);
		ASSERT_TRUE(core.configure(Core::Config{}));
		for (uint64_t t = 50000; t <= 6000000; t += 50000) {
			core.update(coreInput(t));
		}
		ASSERT_GT(core.result().b, 0.f);
		const float total = core.result().i + core.result().b;
		for (uint64_t t = 6050000; t <= 26000000; t += 50000) {
			auto in = coreInput(t);
			in.pilot_abort = abort;
			in.failsafe = !abort;
			auto out = core.update(in);
			EXPECT_FALSE(out.learn);
			EXPECT_FALSE(out.recovery);
			EXPECT_NEAR(out.i + out.b, total, E);
		}
		EXPECT_FLOAT_EQ(core.result().b, 0.f);
		EXPECT_NEAR(rc.rollIntegralRaw(), total, E);
	}
}
TEST(ResidualSlowMemorySafety, HardResetAndInvalidControlSynchronizeEpoch)
{
	for (bool hard : {
	         false, true
	     }) {
		RateControl rc;
		seed(rc, .18f);
		Core core(rc);
		core.configure(Core::Config{});
		for (uint64_t t = 50000; t <= 4000000; t += 50000) {
			core.update(coreInput(t));
		}
		const auto epoch = rc.rollIntegralResetEpoch();
		auto in = coreInput(4050000);
		in.hard_reset = hard;
		in.control_valid = hard;
		const auto out = core.update(in);
		EXPECT_TRUE(out.recovery);
		EXPECT_FLOAT_EQ(out.b, 0.f);
		EXPECT_FLOAT_EQ(out.i, 0.f);
		EXPECT_FALSE(out.estimate_valid);
		EXPECT_FALSE(out.gate_valid);
		EXPECT_EQ(out.samples, 0u);
		EXPECT_EQ(out.epoch, rc.rollIntegralResetEpoch());
		EXPECT_GT(out.epoch, epoch);
	}
}
TEST(ResidualSlowMemorySafety, EstimatorOnlyNaNDoesNotResetNativeAuthority)
{
	State z;
	z.b = .04f;
	z.i = .06f;
	z.q_hat = NAN;
	Subject x(z);
	x.tick(fresh(50000));
	EXPECT_FLOAT_EQ(x.state().b, .04f);
	EXPECT_FLOAT_EQ(x.state().i, .06f);
	EXPECT_EQ(x.state().native_epoch, z.native_epoch);
	EXPECT_FALSE(x.state().estimate_valid);
	EXPECT_FALSE(x.state().learn);
}
TEST(ResidualSlowMemorySafety, RepeatedTransfersConserveWithoutInventingEvidence)
{
	State z;
	z.b = .04f;
	z.i = .06f;
	Subject x(z);
	for (int j = 0; j < 100000; ++j) {
		x.requestPair(j % 2 ? -.00001f : .00001f);
		EXPECT_NEAR(x.state().b + x.state().i, .10f, E);
		EXPECT_EQ(x.state().samples, 0u);
		EXPECT_EQ(x.state().evidence_time, 0u);
	}
}

