// SPDX-License-Identifier: BSD-3-Clause
// Fault episode tests: real core/adapter and real native natural integration.
#include <gtest/gtest.h>
#include "../V4MemoryIntegration.hpp"
#include <limits>

class V4InvalidConfig : public ::testing::Test
{
protected:
	using Core = ResidualSlowFeedforwardMemory;
	RateControl native;
	BumplessRollITransfer v3;
	V4MemoryIntegration adapter{native, v3};
	Core::Config config{};
	V4MemoryIntegration::Context ctx{};
	void SetUp() override
	{
		native.setIntegratorLimit(matrix::Vector3f(.2f, .3f, .4f));
		native.setPidGains({}, matrix::Vector3f(1.f, 0.f, 0.f), {});
		adapter.select(true, false, true);
		ctx.enabled = ctx.armed = ctx.mission = true;
		ctx.dt = .02f;
		ctx.now = ctx.evidence_timestamp = 1000000;
		adapter.before(ctx, config);
		for (int j = 0; j < 60; ++j) { integrate(); }
		for (int j = 0; j < 62; ++j) { step(); }
		ASSERT_GT(s().b_raw, 0.f);
	}
	void integrate() { native.update({}, matrix::Vector3f(.1f, 0.f, 0.f), {}, .02f, false); }
	void step()
	{
		ctx.now += 50000;
		ctx.evidence_timestamp = ctx.now;
		adapter.allocator(ctx.now, 0.f);
		adapter.before(ctx, config);
	}
	const flap_v4_memory_status_s &s() const { return adapter.status(); }
	void faultStatus(bool recovery)
	{
		EXPECT_EQ(s().recovery_active, recovery);
		EXPECT_EQ(s().recovery_reason, static_cast<uint8_t>(Core::Reason::InvalidConfig));
		EXPECT_FLOAT_EQ(s().b_raw, 0.f);
		EXPECT_FALSE(s().learn_valid);
		EXPECT_FALSE(s().apply_valid);
		EXPECT_FALSE(s().window_valid);
		EXPECT_EQ(s().window_sample_count, 0u);
		EXPECT_FLOAT_EQ(s().requested_delta_b_raw, 0.f);
		EXPECT_FLOAT_EQ(s().accepted_delta_b_raw, 0.f);
		EXPECT_TRUE(adapter.active());
		EXPECT_FLOAT_EQ(v3.transferredRaw(), 0.f);
	}
};

TEST_F(V4InvalidConfig, V4InvalidConfig_FirstCycleRecoversOnce)
{
	const auto epoch = native.rollIntegralResetEpoch();
	config.gsign = 1.5f;
	step();
	faultStatus(true);
	EXPECT_EQ(native.rollIntegralResetEpoch(), epoch + 1);
	EXPECT_FLOAT_EQ(native.rollIntegralRaw(), 0.f);
}

TEST_F(V4InvalidConfig, V4InvalidConfig_RepeatedCyclesDoNotRepeatedlyReset)
{
	config.gsign = 1.5f;
	step();
	const auto epoch = native.rollIntegralResetEpoch();
	for (int j = 0; j < 100; ++j) {
		integrate();
		const float i = native.rollIntegralRaw();
		ASSERT_GT(i, 0.f);
		step();
		EXPECT_EQ(native.rollIntegralResetEpoch(), epoch);
		EXPECT_FLOAT_EQ(native.rollIntegralRaw(), i);
		EXPECT_FLOAT_EQ(adapter.composeRaw(i), i);
		faultStatus(false);
	}
}

TEST_F(V4InvalidConfig, V4InvalidConfig_RestoreRequiresFreshWarmup)
{
	config.gsign = 1.5f;
	step();
	for (int j = 0; j < 100; ++j) { integrate(); step(); }
	config.gsign = .9f;
	// Invalid-period timestamp has already been offered: cannot become evidence on restoration.
	adapter.before(ctx, config);
	EXPECT_FALSE(s().learn_valid);
	EXPECT_EQ(s().window_sample_count, 0u);
	for (int j = 0; j < 60; ++j) {
		step();
		EXPECT_FALSE(s().learn_valid);
		EXPECT_FALSE(s().window_valid);
		EXPECT_FLOAT_EQ(s().b_raw, 0.f);
		EXPECT_EQ(s().window_sample_count, static_cast<uint32_t>(j + 1));
	}
	EXPECT_FLOAT_EQ(s().window_span_s, 2.95f);
	step();
	EXPECT_FLOAT_EQ(s().window_span_s, 3.f);
	EXPECT_TRUE(s().window_valid);
	EXPECT_TRUE(s().learn_valid);
	EXPECT_GT(s().b_raw, 0.f);
}

TEST_F(V4InvalidConfig, V4InvalidConfig_DoesNotInstallInvalidAuthority)
{
	const float limit = native.rollIntegralLimit();
	config.imax = std::numeric_limits<float>::quiet_NaN();
	step();
	faultStatus(true);
	const auto epoch = native.rollIntegralResetEpoch();
	for (int j = 0; j < 100; ++j) {
		// Different invalid values remain one uninterrupted invalid episode.
		if (j == 50) { config.imax = .2f; config.bmax = -.1f; }
		integrate(); step();
		EXPECT_FLOAT_EQ(native.rollIntegralLimit(), limit);
		EXPECT_EQ(native.rollIntegralResetEpoch(), epoch);
		faultStatus(false);
	}
}

TEST(V4InvalidConfigCore, UpdateDoesNotResetLatchedFault)
{
	RateControl native;
	native.setIntegratorLimit(matrix::Vector3f(.2f, .3f, .4f));
	native.setPidGains({}, matrix::Vector3f(1.f, 0.f, 0.f), {});
	ResidualSlowFeedforwardMemory core(native);
	ResidualSlowFeedforwardMemory::Config c{};
	ASSERT_TRUE(core.configure(c));
	c.gsign = 1.5f;
	const auto epoch = native.rollIntegralResetEpoch();
	ASSERT_FALSE(core.configure(c));
	EXPECT_TRUE(core.result().recovery);
	EXPECT_EQ(native.rollIntegralResetEpoch(), epoch + 1);
	for (int j = 0; j < 100; ++j) {
		native.update({}, matrix::Vector3f(.1f, 0.f, 0.f), {}, .02f, false);
		const float i = native.rollIntegralRaw();
		const auto r = core.update({});
		EXPECT_EQ(native.rollIntegralResetEpoch(), epoch + 1);
		EXPECT_FLOAT_EQ(r.i, i);
		EXPECT_FLOAT_EQ(r.b, 0.f);
		EXPECT_FALSE(r.recovery);
		EXPECT_FALSE(r.learn);
		EXPECT_FALSE(r.apply);
		EXPECT_FLOAT_EQ(r.requested, 0.f);
		EXPECT_FLOAT_EQ(r.accepted, 0.f);
		EXPECT_EQ(r.reason, ResidualSlowFeedforwardMemory::Reason::InvalidConfig);
	}
}

TEST_F(V4InvalidConfig, V4InvalidConfig_SecondEpisodeRecoversOnceAgain)
{
	config.gsign = 1.5f; step();
	const auto first_epoch = native.rollIntegralResetEpoch();
	config.gsign = .9f; step();
	integrate();
	config.gsign = 1.5f; step();
	faultStatus(true);
	EXPECT_EQ(native.rollIntegralResetEpoch(), first_epoch + 1);
	step(); faultStatus(false);
	EXPECT_EQ(native.rollIntegralResetEpoch(), first_epoch + 1);
}
