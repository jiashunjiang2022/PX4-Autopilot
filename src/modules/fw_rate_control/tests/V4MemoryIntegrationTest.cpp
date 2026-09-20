// SPDX-License-Identifier: BSD-3-Clause
#include <gtest/gtest.h>
#include "../V4MemoryIntegration.hpp"
#include <limits>

struct ResidualSlowFeedforwardMemoryTestAccess {
	static void seed(RateControl &n, ResidualSlowFeedforwardMemory &c, float i, float b)
	{
		n._rate_int(0) = i;
		c._b = b;
		c._epoch = n.rollIntegralResetEpoch();
	}
};
struct V4IntegrationTestAccess {
	static void seed(V4MemoryIntegration &a, RateControl &n, float i, float b)
	{
		ResidualSlowFeedforwardMemoryTestAccess::seed(n, a._core, i, b);
	}
};
class V4MemoryIntegrationTest : public ::testing::Test
{
protected:
	RateControl native;
	BumplessRollITransfer v3;
	V4MemoryIntegration adapter{native, v3};
	V4MemoryIntegration::Config config{};
	V4MemoryIntegration::Context ctx{};
	void SetUp() override
	{
		native.setIntegratorLimit(matrix::Vector3f(.2f, .3f, .4f));
		native.setPidGains(matrix::Vector3f(), matrix::Vector3f(1.f, 0.f, 0.f), matrix::Vector3f());
		adapter.select(true, false, true);
		ctx.enabled = ctx.armed = ctx.mission = true;
		ctx.dt = .02f;
		ctx.now = ctx.evidence_timestamp = 1000000;
		adapter.before(ctx, config); // acquisition epoch synchronization
		V4IntegrationTestAccess::seed(adapter, native, .15f, 0.f);
	}
	void step(bool fresh_allocator = true)
	{
		ctx.now += 50000;
		ctx.evidence_timestamp = ctx.now;
		if (fresh_allocator) { adapter.allocator(ctx.now, 0.f); }
		adapter.before(ctx, config);
	}
	void warm() { for (int j = 0; j < 62; ++j) { step(); } }
	const flap_v4_memory_status_s &s() { return adapter.status(); }
};

TEST_F(V4MemoryIntegrationTest, DisabledPreservesNativeAndV3)
{
	adapter.select(false, true, true);
	const float i = native.rollIntegralRaw();
	const auto epoch = native.rollIntegralResetEpoch();
	ctx.enabled = false;
	adapter.before(ctx, config);
	EXPECT_FALSE(adapter.active());
	EXPECT_FLOAT_EQ(native.rollIntegralRaw(), i);
	EXPECT_EQ(native.rollIntegralResetEpoch(), epoch);
}
TEST_F(V4MemoryIntegrationTest, ArmedAcquisitionDeferred)
{
	adapter.select(false, true, true);
	adapter.select(true, true, true);
	adapter.before(ctx, config);
	EXPECT_FALSE(adapter.active()); EXPECT_TRUE(s().enable_pending);
}
TEST_F(V4MemoryIntegrationTest, V3MemoryZeroWhileV4Owns)
{
	warm(); EXPECT_GT(s().b_raw, 0.f); EXPECT_FLOAT_EQ(v3.transferredRaw(), 0.f);
}
TEST_F(V4MemoryIntegrationTest, RawBInsertedExactlyOnce)
{
	warm(); EXPECT_NEAR(adapter.composeRaw(.03f), .03f + s().b_raw, 2e-7f);
}
TEST_F(V4MemoryIntegrationTest, PureTransferSameCurrentOutput)
{
	warm(); ASSERT_GT(s().accepted_delta_b_raw, 0.f);
	EXPECT_NEAR(s().t_post_transfer_raw, s().t_pre_transfer_raw, 2e-7f);
	rate_ctrl_terms_s terms{};
	const auto output = native.update({}, {}, {}, .02f, false, &terms);
	EXPECT_FLOAT_EQ(terms.i_term[0], s().i_post_transfer_raw);
	EXPECT_NEAR(adapter.composeRaw(output(0)), s().t_pre_transfer_raw, 2e-7f);
}
TEST_F(V4MemoryIntegrationTest, PostTransferNaturalTiming)
{
	warm(); const float post = s().i_post_transfer_raw;
	rate_ctrl_terms_s terms{};
	native.update({}, matrix::Vector3f(.1f, 0.f, 0.f), {}, .02f, false, &terms);
	adapter.after(.7f);
	EXPECT_FLOAT_EQ(terms.i_term[0], post); EXPECT_GT(s().i_post_natural_raw, post);
	EXPECT_FLOAT_EQ(s().t_post_natural_raw, s().i_post_natural_raw + s().b_raw);
	EXPECT_FLOAT_EQ(s().b_after_gain, .7f * s().b_raw);
}
TEST_F(V4MemoryIntegrationTest, AllocatorAge100msBoundary)
{
	adapter.allocator(ctx.now - 100000, 0.f); adapter.before(ctx, config);
	EXPECT_TRUE(s().allocator_fresh);
	++ctx.now; adapter.before(ctx, config); EXPECT_FALSE(s().allocator_fresh);
}
TEST_F(V4MemoryIntegrationTest, AllocatorFutureAndNonfiniteUnknown)
{
	adapter.allocator(ctx.now + 1, 0.f); adapter.before(ctx, config); EXPECT_FALSE(s().allocator_fresh);
	adapter.allocator(ctx.now, std::numeric_limits<float>::quiet_NaN());
	adapter.before(ctx, config); EXPECT_FALSE(s().allocator_fresh);
}
TEST_F(V4MemoryIntegrationTest, StaleAllocatorHoldsB)
{
	warm(); const float b = s().b_raw;
	for (int j = 0; j < 4; ++j) { step(false); }
	const float held = s().b_raw; EXPECT_GE(held, b);
	step(false); EXPECT_FLOAT_EQ(s().b_raw, held); EXPECT_FALSE(s().learn_valid); EXPECT_TRUE(s().apply_valid);
}
TEST_F(V4MemoryIntegrationTest, UnknownToFreshFullWindow)
{
	warm(); ctx.now += 100001; adapter.before(ctx, config); EXPECT_FALSE(s().window_valid);
	for (int j = 0; j < 60; ++j) { step(); EXPECT_FALSE(s().learn_valid); }
	step(); EXPECT_TRUE(s().learn_valid);
}
TEST_F(V4MemoryIntegrationTest, NoPublicationTimeout)
{
	warm(); const float b = s().b_raw;
	ctx.now += 150001; adapter.allocator(ctx.now, 0.f); adapter.before(ctx, config);
	EXPECT_TRUE(s().evidence_gap_reset); EXPECT_FALSE(s().window_valid); EXPECT_FLOAT_EQ(s().b_raw, b);
}
TEST_F(V4MemoryIntegrationTest, DuplicateTimestamp)
{
	warm(); const auto count = s().window_sample_count; const float b = s().b_raw;
	adapter.before(ctx, config); EXPECT_FALSE(s().evidence_fresh);
	EXPECT_EQ(s().window_sample_count, count); EXPECT_FLOAT_EQ(s().b_raw, b);
}
TEST_F(V4MemoryIntegrationTest, BackwardTimestamp)
{
	warm(); const float b = s().b_raw; ctx.evidence_timestamp -= 50000;
	adapter.before(ctx, config); EXPECT_FALSE(s().evidence_fresh); EXPECT_FLOAT_EQ(s().b_raw, b);
}
TEST_F(V4MemoryIntegrationTest, Gap150msBoundary)
{
	warm(); ctx.now += 150000; adapter.allocator(ctx.now, 0.f); adapter.before(ctx, config);
	EXPECT_FALSE(s().evidence_gap_reset); EXPECT_TRUE(s().window_valid);
	++ctx.now; adapter.before(ctx, config); EXPECT_TRUE(s().evidence_gap_reset); EXPECT_FALSE(s().window_valid);
}
TEST_F(V4MemoryIntegrationTest, MissionLearns)
{
	warm(); EXPECT_TRUE(s().learn_valid); EXPECT_GT(s().b_raw, 0.f);
}
TEST_F(V4MemoryIntegrationTest, OrdinaryManeuverHoldApply)
{
	warm(); const float b = s().b_raw; ctx.maneuver = true;
	for (int j = 0; j < 70; ++j) { step(); }
	EXPECT_FLOAT_EQ(s().b_raw, b); EXPECT_TRUE(s().apply_valid); EXPECT_FALSE(s().handback_active);
}
TEST_F(V4MemoryIntegrationTest, MissionToStabHandback)
{
	warm(); const float b = s().b_raw; ctx.mission = false; ctx.pilot_abort = true; step();
	EXPECT_LT(s().b_raw, b); EXPECT_NEAR(s().t_pre_transfer_raw, s().t_post_transfer_raw, 2e-7f);
	EXPECT_FALSE(s().learn_valid);
}
TEST_F(V4MemoryIntegrationTest, FailsafeLegalHandback)
{
	warm(); const float b = s().b_raw; ctx.failsafe = true; step();
	EXPECT_LT(s().b_raw, b); EXPECT_FALSE(s().recovery_active);
	EXPECT_EQ(s().recovery_reason, static_cast<uint8_t>(ResidualSlowFeedforwardMemory::Reason::Failsafe));
}
TEST_F(V4MemoryIntegrationTest, LandedReset)
{
	warm(); ctx.landed = true; step(); EXPECT_FLOAT_EQ(s().b_raw, 0.f);
	EXPECT_FLOAT_EQ(native.rollIntegralRaw(), 0.f); EXPECT_TRUE(s().recovery_active);
}
TEST_F(V4MemoryIntegrationTest, DisarmReset)
{
	warm(); ctx.armed = false; step(); EXPECT_FLOAT_EQ(s().b_raw, 0.f); EXPECT_EQ(s().window_sample_count, 0u);
}
TEST_F(V4MemoryIntegrationTest, EpochMismatch)
{
	warm(); native.resetIntegral(0); step(); EXPECT_FLOAT_EQ(s().b_raw, 0.f);
	EXPECT_EQ(s().recovery_reason, static_cast<uint8_t>(ResidualSlowFeedforwardMemory::Reason::EpochMismatch));
	EXPECT_EQ(s().reset_epoch, native.rollIntegralResetEpoch());
}
TEST_F(V4MemoryIntegrationTest, ImaxShrinkRecoveryBeforeUse)
{
	warm(); config.imax = .1f; step(); EXPECT_TRUE(s().recovery_active);
	EXPECT_FLOAT_EQ(native.rollIntegralRaw(), 0.f); EXPECT_FLOAT_EQ(s().b_raw, 0.f);
	EXPECT_FLOAT_EQ(native.rollIntegralLimit(), .1f);
}
TEST_F(V4MemoryIntegrationTest, BmaxShrinkRecoveryBeforeUse)
{
	warm(); config.bmax = 0.f; step(); EXPECT_TRUE(s().recovery_active);
	EXPECT_FLOAT_EQ(native.rollIntegralRaw(), 0.f); EXPECT_FLOAT_EQ(s().b_raw, 0.f);
}
TEST_F(V4MemoryIntegrationTest, InvalidConfigCannotReachNativeLimit)
{
	warm(); config.imax = std::numeric_limits<float>::quiet_NaN(); step();
	EXPECT_TRUE(s().recovery_active); EXPECT_FLOAT_EQ(native.rollIntegralLimit(), .2f);
	EXPECT_EQ(s().recovery_reason, static_cast<uint8_t>(ResidualSlowFeedforwardMemory::Reason::InvalidConfig));
}
TEST_F(V4MemoryIntegrationTest, DiagnosticsPrePostTiming)
{
	warm(); EXPECT_NEAR(s().b_raw - s().b_pre_raw, s().accepted_delta_b_raw, 2e-7f);
	EXPECT_NEAR(s().i_pre_transfer_raw - s().i_post_transfer_raw, s().accepted_delta_b_raw, 2e-7f);
}
TEST_F(V4MemoryIntegrationTest, RecoveryReasonNotOverwritten)
{
	warm(); config.bmax = 0.f; step();
	EXPECT_EQ(s().recovery_reason, static_cast<uint8_t>(ResidualSlowFeedforwardMemory::Reason::IllegalPair));
	EXPECT_EQ(s().window_sample_count, 0u); step(); EXPECT_FALSE(s().window_valid);
}
TEST_F(V4MemoryIntegrationTest, DisableDrainsBeforeReturningOwnership)
{
	warm(); ctx.enabled = false;
	adapter.select(false, true, true); EXPECT_TRUE(adapter.active());
	for (int j = 0; j < 100; ++j) { step(); if (fabsf(s().b_raw) <= 0.f) { break; } }
	EXPECT_FLOAT_EQ(s().b_raw, 0.f); adapter.select(false, true, true); EXPECT_FALSE(adapter.active());
	EXPECT_FLOAT_EQ(v3.transferredRaw(), 0.f);
}
TEST_F(V4MemoryIntegrationTest, HrAlwaysZeroNativeBound)
{
	V4IntegrationTestAccess::seed(adapter, native, .1f, .1f); step();
	for (int j = 0; j < 1000; ++j) { native.update({}, matrix::Vector3f(1.f, 0.f, 0.f), {}, .02f, false); }
	EXPECT_LE(native.rollIntegralRaw() + s().b_raw, .2000002f);
}
TEST_F(V4MemoryIntegrationTest, SaturationBothSigns)
{
	for (float sign : {1.f, -1.f}) {
		V4IntegrationTestAccess::seed(adapter, native, sign * .15f, 0.f);
		for (int j = 0; j < 150; ++j) {
			ctx.now += 50000; ctx.evidence_timestamp = ctx.now;
			adapter.allocator(ctx.now, sign * .1f); adapter.before(ctx, config);
		}
		EXPECT_FLOAT_EQ(s().b_raw, 0.f); EXPECT_FALSE(s().learn_valid);
	}
}
TEST_F(V4MemoryIntegrationTest, EvidenceDecimatedWithoutInventingTime)
{
	step(); const auto count = s().window_sample_count;
	for (int j = 0; j < 4; ++j) {
		ctx.now += 10000; ctx.evidence_timestamp = ctx.now; adapter.before(ctx, config);
		EXPECT_FALSE(s().evidence_fresh); EXPECT_EQ(s().window_sample_count, count);
	}
	ctx.now += 10000; ctx.evidence_timestamp = ctx.now; adapter.before(ctx, config);
	EXPECT_TRUE(s().evidence_fresh); EXPECT_EQ(s().window_sample_count, count + 1);
}
TEST_F(V4MemoryIntegrationTest, OutputRecoveryPreservesEarlierSnapshots)
{
	warm();
	native.update({}, matrix::Vector3f(.1f, 0.f, 0.f), {}, .02f, false);
	adapter.after(.7f);
	const auto previous = s();
	native.resetIntegral(); adapter.reset();
	EXPECT_FLOAT_EQ(s().i_post_transfer_raw, previous.i_post_transfer_raw);
	EXPECT_FLOAT_EQ(s().i_post_natural_raw, previous.i_post_natural_raw);
	EXPECT_FLOAT_EQ(s().b_raw, previous.b_raw);
	EXPECT_FLOAT_EQ(native.rollIntegralRaw(), 0.f);
	EXPECT_FLOAT_EQ(adapter.composeRaw(0.f), 0.f);
	EXPECT_TRUE(s().output_recovery);
	EXPECT_FLOAT_EQ(s().i_cycle_end_raw, 0.f);
	EXPECT_FLOAT_EQ(s().b_cycle_end_raw, 0.f);
}
