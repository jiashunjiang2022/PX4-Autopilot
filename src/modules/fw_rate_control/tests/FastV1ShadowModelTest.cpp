#include <gtest/gtest.h>
#include "../FastV1ShadowModel.hpp"

TEST(FastV1ShadowModel, WarmupAndResetPreserveCounters)
{
	FastV1ShadowModel m; FastV1ShadowModel::Output o{};
	for (uint32_t k = 0; k < 9; ++k) { ASSERT_TRUE(m.update(k * 50000, 0.01f * k, 0.3f, 0.1f, o)); }
	EXPECT_TRUE(o.valid); const auto seq = o.frame_seq; const auto missed = o.missed_frames;
	m.resetFeatureHistory();
	ASSERT_TRUE(m.update(9 * 50000, 0.2f, 0.3f, 0.1f, o));
	EXPECT_FALSE(o.valid); EXPECT_EQ(o.abs4, 0.f); EXPECT_EQ(o.delta4, 0.f); EXPECT_GT(o.frame_seq, seq); EXPECT_EQ(o.missed_frames, missed);
}

TEST(FastV1ShadowModel, NonfiniteInputInvalidatesBeforeHistoryWrite)
{
	FastV1ShadowModel m; FastV1ShadowModel::Output o{};
	for (uint32_t k = 0; k < 9; ++k) { m.update(k * 50000, 0.01f * k, 0.3f, 0.1f, o); }
	ASSERT_TRUE(o.valid);
	m.update(9 * 50000, NAN, 0.3f, 0.1f, o);
	EXPECT_FALSE(o.valid); EXPECT_EQ(o.abs4, 0.f); EXPECT_EQ(o.delta4, 0.f);
}

TEST(FastV1ShadowModel, FramePhaseAndMissedSlots)
{
	FastV1ShadowModel m; FastV1ShadowModel::Output o{};
	EXPECT_TRUE(m.update(0, 0.f, 0.f, 0.f, o)); EXPECT_EQ(o.frame_dt_us, 0u);
	EXPECT_FALSE(m.update(20000, 0.f, 0.f, 0.f, o));
	EXPECT_TRUE(m.update(60000, 0.f, 0.f, 0.f, o)); EXPECT_EQ(o.frame_dt_us, 60000u); EXPECT_EQ(o.missed_frames, 0u);
	EXPECT_TRUE(m.update(220000, 0.f, 0.f, 0.f, o)); EXPECT_EQ(o.missed_frames, 2u);
}
