#include <gtest/gtest.h>

#include "AirspeedSelectorQuality.hpp"

#include <lib/airspeed/AirspeedQualityMode.hpp>
#include <uORB/topics/airspeed_selector_quality_status.h>

namespace
{

class FullRecoveryHarness
{
public:
	void update(uint64_t now, uint64_t quality_timestamp, bool physical_source_alive, bool identity_match,
		    bool quality_valid, float quality, bool fuse_enabled)
	{
		(void)quality; // q affects the EKF gate, but is not a second selector decision input.
		_quality_timestamp = quality_timestamp;
		const bool quality_fresh = quality_valid && quality_timestamp > 0 && now >= quality_timestamp
					   && (now - quality_timestamp) < 1000000;

		if (!physical_source_alive || !identity_match) {
			_latch = {};

		} else {
			const auto decision = airspeed_selector_quality::evaluate_quality(quality_fresh, fuse_enabled);
			airspeed_selector_quality::update_latch(decision, _latch);
		}

		_quality_rejected = physical_source_alive && identity_match && _latch.latched;
		_final_source = physical_source_alive && !_quality_rejected ? 1 : -1;
	}

	bool quality_rejected() const { return _quality_rejected; }
	bool latched() const { return _latch.latched; }
	int8_t final_source() const { return _final_source; }
	uint64_t quality_timestamp() const { return _quality_timestamp; }

private:
	airspeed_selector_quality::QualityLatchState _latch{};
	uint64_t _quality_timestamp{0};
	int8_t _final_source{-1};
	bool _quality_rejected{false};
};

} // namespace

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

TEST(AirspeedSelectorQuality, stale_or_invalid_quality_cannot_change_unified_gate_latch)
{
	const auto stale = airspeed_selector_quality::evaluate_quality(false, true);
	EXPECT_FALSE(stale.reject);
	EXPECT_FALSE(stale.reopen);
	EXPECT_TRUE(stale.timed_out);

	airspeed_selector_quality::QualityLatchState latched{true};
	airspeed_selector_quality::update_latch(stale, latched);
	EXPECT_TRUE(latched.latched);

	airspeed_selector_quality::QualityLatchState open{};
	airspeed_selector_quality::update_latch(stale, open);
	EXPECT_FALSE(open.latched);
}

TEST(AirspeedSelectorQuality, selector_mirrors_only_the_unified_gate)
{
	const auto gate_open = airspeed_selector_quality::evaluate_quality(true, true);
	EXPECT_FALSE(gate_open.reject);
	EXPECT_TRUE(gate_open.reopen);

	const auto gate_closed = airspeed_selector_quality::evaluate_quality(true, false);
	EXPECT_TRUE(gate_closed.reject);
	EXPECT_FALSE(gate_closed.reopen);
}

TEST(AirspeedSelectorQuality, selector_latch_has_no_second_hold_or_dwell)
{
	airspeed_selector_quality::QualityLatchState state{};
	airspeed_selector_quality::update_latch(airspeed_selector_quality::evaluate_quality(true, false), state);
	EXPECT_TRUE(state.latched);

	airspeed_selector_quality::update_latch(airspeed_selector_quality::evaluate_quality(true, true), state);
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

TEST(AirspeedSelectorQuality, full_mode_q_near_half_with_open_gate_keeps_physical_source_selected)
{
	const auto full = airspeed_quality::mode_config(3);
	ASSERT_TRUE(full.adaptive_r_enabled);
	ASSERT_TRUE(full.quality_fusion_gate_enabled);
	ASSERT_TRUE(full.selector_quality_enabled);

	FullRecoveryHarness harness;
	harness.update(1000000, 1000000, true, true, true, 0.49f, true);
	EXPECT_FALSE(harness.quality_rejected());
	EXPECT_EQ(harness.final_source(), 1);
}

TEST(AirspeedSelectorQuality, full_mode_below_qoff_does_not_reject_before_ekf_gate_closes)
{
	FullRecoveryHarness harness;
	harness.update(1000000, 1000000, true, true, true, 0.39f, true);
	EXPECT_FALSE(harness.quality_rejected());
	EXPECT_EQ(harness.final_source(), 1);
}

TEST(AirspeedSelectorQuality, full_mode_closed_ekf_gate_rejects_physical_source)
{
	FullRecoveryHarness harness;
	harness.update(1000000, 1000000, true, true, true, 0.8f, false);
	EXPECT_TRUE(harness.quality_rejected());
	EXPECT_EQ(harness.final_source(), -1);
}

TEST(AirspeedSelectorQuality, full_mode_rejects_then_monitors_and_reselects_recovered_source)
{
	FullRecoveryHarness harness;
	harness.update(1000000, 1000000, true, true, true, 0.2f, false);
	ASSERT_TRUE(harness.quality_rejected());
	ASSERT_EQ(harness.final_source(), -1);

	// Fresh qmon updates cannot recover the source until the EKF gate itself reopens.
	for (uint64_t now = 1100000; now <= 3000000; now += 100000) {
		harness.update(now, now, true, true, true, 0.8f, false);
		EXPECT_TRUE(harness.latched());
		EXPECT_EQ(harness.quality_timestamp(), now);
		EXPECT_EQ(harness.final_source(), -1);
	}

	// QON + TON are complete inside the EKF gate; selector adds no second recovery dwell.
	harness.update(3100000, 3100000, true, true, true, 0.8f, true);
	EXPECT_FALSE(harness.latched());
	EXPECT_FALSE(harness.quality_rejected());
	EXPECT_EQ(harness.final_source(), 1);
}

TEST(AirspeedSelectorQuality, stale_open_gate_cannot_recover_rejected_source)
{
	FullRecoveryHarness harness;
	harness.update(1000000, 1000000, true, true, true, 0.2f, false);
	ASSERT_TRUE(harness.latched());

	for (uint64_t now = 2100000; now <= 6000000; now += 100000) {
		harness.update(now, 1000000, true, true, true, 0.9f, true);
	}

	EXPECT_TRUE(harness.latched());
	EXPECT_TRUE(harness.quality_rejected());
	EXPECT_EQ(harness.final_source(), -1);
}

TEST(AirspeedSelectorQuality, invalid_open_gate_cannot_recover_rejected_source)
{
	FullRecoveryHarness harness;
	harness.update(1000000, 1000000, true, true, true, 0.2f, false);
	ASSERT_TRUE(harness.latched());

	for (uint64_t now = 1100000; now <= 3000000; now += 100000) {
		harness.update(now, now, true, true, false, 0.9f, true);
	}

	EXPECT_TRUE(harness.latched());
	EXPECT_EQ(harness.final_source(), -1);
}

TEST(AirspeedSelectorQuality, physical_source_timeout_cannot_be_recovered_by_quality_alone)
{
	FullRecoveryHarness harness;
	harness.update(1000000, 1000000, true, true, true, 0.2f, false);
	ASSERT_TRUE(harness.latched());

	for (uint64_t now = 1100000; now <= 3000000; now += 100000) {
		harness.update(now, now, false, true, true, 0.9f, true);
		EXPECT_EQ(harness.final_source(), -1);
	}

	EXPECT_FALSE(harness.latched());
	EXPECT_FALSE(harness.quality_rejected());
}

TEST(AirspeedSelectorQuality, identity_mismatch_disables_quality_gate_authority)
{
	FullRecoveryHarness harness;
	harness.update(1000000, 1000000, true, false, true, 0.9f, false);
	EXPECT_FALSE(harness.quality_rejected());
	EXPECT_EQ(harness.final_source(), 1);
	EXPECT_FALSE(airspeed_selector_quality::source_identity_matches(1, 1, 42, 0, 43));
}
