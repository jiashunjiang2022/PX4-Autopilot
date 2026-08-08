#include <gtest/gtest.h>

#include "AirspeedSelectorQuality.hpp"

#include <uORB/topics/airspeed_selector_quality_status.h>

TEST(AirspeedSelectorQuality, separates_output_finiteness_from_validator_validity)
{
	EXPECT_TRUE(airspeed_selector_quality::pre_quality_output_finite(true, true, true));
	EXPECT_FALSE(airspeed_selector_quality::pre_quality_output_finite(false, true, true));
	EXPECT_FALSE(airspeed_selector_quality::pre_quality_output_finite(true, false, true));
	EXPECT_FALSE(airspeed_selector_quality::pre_quality_output_finite(true, true, false));
	EXPECT_TRUE(airspeed_selector_quality::original_selection_was_fallback(0));
	EXPECT_TRUE(airspeed_selector_quality::original_selection_was_fallback(4));
	EXPECT_FALSE(airspeed_selector_quality::original_selection_was_fallback(1));
}

TEST(AirspeedSelectorQuality, custom_blockage_is_disabled_unless_explicitly_enabled)
{
	EXPECT_FALSE(airspeed_selector_quality::custom_blockage_enabled(0));
	EXPECT_TRUE(airspeed_selector_quality::custom_blockage_enabled(1));
	EXPECT_FALSE(airspeed_selector_quality::custom_blockage_enabled(2));
}

TEST(AirspeedSelectorQuality, status_schema_exposes_original_selection_semantics)
{
	airspeed_selector_quality_status_s status{};
	status.pre_quality_output_finite = true;
	status.original_sensor_valid = false;
	status.original_selection_was_fallback = true;
	status.original_selected_source = 0;
	status.concurrent_original_sensor_invalid = false;
	EXPECT_TRUE(status.pre_quality_output_finite);
	EXPECT_FALSE(status.original_sensor_valid);
	EXPECT_TRUE(status.original_selection_was_fallback);
	EXPECT_EQ(status.original_selected_source, 0);
	EXPECT_FALSE(status.concurrent_original_sensor_invalid);
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

TEST(AirspeedSelectorQuality, fallback_outcome_always_matches_final_source)
{
	using airspeed_selector_quality::FallbackOutcome;
	EXPECT_EQ(airspeed_selector_quality::fallback_outcome_for_source(1), FallbackOutcome::AlternatePhysical);
	EXPECT_EQ(airspeed_selector_quality::fallback_outcome_for_source(2), FallbackOutcome::AlternatePhysical);
	EXPECT_EQ(airspeed_selector_quality::fallback_outcome_for_source(3), FallbackOutcome::AlternatePhysical);
	EXPECT_EQ(airspeed_selector_quality::fallback_outcome_for_source(0), FallbackOutcome::GroundMinusWind);
	EXPECT_EQ(airspeed_selector_quality::fallback_outcome_for_source(4), FallbackOutcome::Synthetic);
	EXPECT_EQ(airspeed_selector_quality::fallback_outcome_for_source(-1), FallbackOutcome::Unavailable);
	EXPECT_EQ(static_cast<uint8_t>(FallbackOutcome::AlternatePhysical),
		  airspeed_selector_quality_status_s::FALLBACK_OUTCOME_ALTERNATE_PHYSICAL);
	EXPECT_EQ(static_cast<uint8_t>(FallbackOutcome::GroundMinusWind),
		  airspeed_selector_quality_status_s::FALLBACK_OUTCOME_GROUND_MINUS_WIND);
	EXPECT_EQ(static_cast<uint8_t>(FallbackOutcome::Synthetic),
		  airspeed_selector_quality_status_s::FALLBACK_OUTCOME_SYNTHETIC);
	EXPECT_EQ(static_cast<uint8_t>(FallbackOutcome::Unavailable),
		  airspeed_selector_quality_status_s::FALLBACK_OUTCOME_UNAVAILABLE);
}

TEST(AirspeedSelectorQuality, configured_fallback_modes_map_to_reported_outcome)
{
	using airspeed_selector_quality::FallbackOutcome;
	const auto disabled = airspeed_selector_quality::choose_fallback(-1, true, true, 0);
	EXPECT_EQ(airspeed_selector_quality::fallback_outcome_for_source(disabled.source), FallbackOutcome::Unavailable);
	const auto ground_wind = airspeed_selector_quality::choose_fallback(-1, true, true, 1);
	EXPECT_EQ(airspeed_selector_quality::fallback_outcome_for_source(ground_wind.source), FallbackOutcome::GroundMinusWind);
	const auto synthetic = airspeed_selector_quality::choose_fallback(-1, true, true, 2);
	EXPECT_EQ(airspeed_selector_quality::fallback_outcome_for_source(synthetic.source), FallbackOutcome::Synthetic);
}

TEST(AirspeedSelectorQuality, quality_and_blockage_concurrency_uses_actual_final_source)
{
	using airspeed_selector_quality::FallbackOutcome;
	const auto synthetic_after_blockage = airspeed_selector_quality::fallback_status_for_source(4);
	EXPECT_TRUE(synthetic_after_blockage.available);
	EXPECT_EQ(synthetic_after_blockage.source, 4);
	EXPECT_EQ(synthetic_after_blockage.outcome, FallbackOutcome::Synthetic);

	const auto unavailable_after_blockage = airspeed_selector_quality::fallback_status_for_source(-1);
	EXPECT_FALSE(unavailable_after_blockage.available);
	EXPECT_EQ(unavailable_after_blockage.source, -1);
	EXPECT_EQ(unavailable_after_blockage.outcome, FallbackOutcome::Unavailable);
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
