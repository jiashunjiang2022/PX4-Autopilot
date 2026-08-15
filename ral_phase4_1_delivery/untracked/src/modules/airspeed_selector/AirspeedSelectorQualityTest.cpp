#include <gtest/gtest.h>

#include "AirspeedSelectorQuality.hpp"

TEST(AirspeedSelectorQuality, preserves_original_validity_outcome)
{
	EXPECT_TRUE(airspeed_selector_quality::original_validity(true, true, true));
	EXPECT_FALSE(airspeed_selector_quality::original_validity(false, true, true));
	EXPECT_FALSE(airspeed_selector_quality::original_validity(true, false, true));
	EXPECT_FALSE(airspeed_selector_quality::original_validity(true, true, false));
}

TEST(AirspeedSelectorQuality, prefers_alternate_physical_source)
{
	const auto choice = airspeed_selector_quality::choose_fallback(2, true, true, 1);
	EXPECT_TRUE(choice.available);
	EXPECT_EQ(choice.source, 2);
}

TEST(AirspeedSelectorQuality, selects_configured_ground_wind_fallback)
{
	const auto choice = airspeed_selector_quality::choose_fallback(-1, true, true, 1);
	EXPECT_TRUE(choice.available);
	EXPECT_EQ(choice.source, 0);
}

TEST(AirspeedSelectorQuality, selects_configured_synthetic_fallback)
{
	const auto choice = airspeed_selector_quality::choose_fallback(-1, false, true, 2);
	EXPECT_TRUE(choice.available);
	EXPECT_EQ(choice.source, 4);
}

TEST(AirspeedSelectorQuality, reports_unavailable_without_configured_source)
{
	const auto choice = airspeed_selector_quality::choose_fallback(-1, true, true, 0);
	EXPECT_FALSE(choice.available);
	EXPECT_EQ(choice.source, -1);
}

TEST(AirspeedSelectorQuality, reports_source_transitions_as_availability_changes)
{
	const auto alternate = airspeed_selector_quality::choose_fallback(2, true, true, 1);
	EXPECT_TRUE(alternate.available);
	EXPECT_EQ(alternate.source, 2);

	const auto ground_wind = airspeed_selector_quality::choose_fallback(-1, true, true, 1);
	EXPECT_TRUE(ground_wind.available);
	EXPECT_EQ(ground_wind.source, 0);

	const auto disabled = airspeed_selector_quality::choose_fallback(-1, false, true, 1);
	EXPECT_FALSE(disabled.available);
	EXPECT_EQ(disabled.source, -1);
}

TEST(AirspeedSelectorQuality, rejects_stale_or_invalid_quality)
{
	const auto stale = airspeed_selector_quality::evaluate_quality(false, true, 1.f, true, true, 0.5f, 0.05f);
	EXPECT_TRUE(stale.reject);
	EXPECT_TRUE(stale.timed_out);

	const auto invalid = airspeed_selector_quality::evaluate_quality(true, false, 0.f, true, true, 0.5f, 0.05f);
	EXPECT_TRUE(invalid.reject);
	EXPECT_FALSE(invalid.timed_out);
}

TEST(AirspeedSelectorQuality, rejects_low_spectral_quality_and_closed_gate)
{
	EXPECT_TRUE(airspeed_selector_quality::evaluate_quality(true, true, 0.4f, true, true, 0.5f, 0.05f).reject);
	EXPECT_FALSE(airspeed_selector_quality::evaluate_quality(true, true, 0.4f, true, false, 0.5f, 0.05f).reject);
	EXPECT_TRUE(airspeed_selector_quality::evaluate_quality(true, true, 0.8f, false, true, 0.5f, 0.05f).reject);
}

TEST(AirspeedSelectorQuality, holds_then_reenables_after_dwell)
{
	airspeed_selector_quality::QualityLatchState state{};
	const auto reject = airspeed_selector_quality::evaluate_quality(true, true, 0.2f, true, true, 0.5f, 0.05f);
	airspeed_selector_quality::update_latch(100, 200, 100, reject, state);
	EXPECT_TRUE(state.latched);

	const auto recover = airspeed_selector_quality::evaluate_quality(true, true, 0.8f, true, true, 0.5f, 0.05f);
	airspeed_selector_quality::update_latch(299, 200, 100, recover, state);
	EXPECT_TRUE(state.latched);
	airspeed_selector_quality::update_latch(300, 200, 100, recover, state);
	EXPECT_TRUE(state.latched);
	airspeed_selector_quality::update_latch(400, 200, 100, recover, state);
	EXPECT_FALSE(state.latched);
}

TEST(AirspeedSelectorQuality, source_identity_requires_one_matching_physical_pitot)
{
	EXPECT_TRUE(airspeed_selector_quality::source_identity_matches(1, 1, 42, 0, 42));
	EXPECT_FALSE(airspeed_selector_quality::source_identity_matches(2, 1, 42, 0, 42));
	EXPECT_FALSE(airspeed_selector_quality::source_identity_matches(1, 2, 42, 0, 42));
	EXPECT_FALSE(airspeed_selector_quality::source_identity_matches(1, 1, 42, 1, 42));
	EXPECT_FALSE(airspeed_selector_quality::source_identity_matches(1, 1, 42, 0, 43));
	EXPECT_FALSE(airspeed_selector_quality::source_identity_matches(1, 1, 0, 0, 0));
}
