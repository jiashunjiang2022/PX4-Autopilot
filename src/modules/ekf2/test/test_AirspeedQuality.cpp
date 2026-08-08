#include <gtest/gtest.h>

#include "../AirspeedQualityMath.hpp"
#include "../AirspeedQualityDiagnostics.hpp"
#include "../AirspeedQualitySnapshot.hpp"

#include <lib/airspeed/AirspeedQualityMode.hpp>

#include <cmath>

namespace
{

constexpr int kSampleCount = 200;
constexpr float kSampleRateHz = 50.f;
constexpr float kPi = 3.14159265358979323846f;

void fill_tone(float *samples, float frequency_hz, float trend_per_second = 0.f)
{
	for (int i = 0; i < kSampleCount; ++i) {
		const float time_s = static_cast<float>(i) / kSampleRateHz;
		samples[i] = sinf(2.f * kPi * frequency_hz * time_s) + trend_per_second * time_s;
	}
}

} // namespace

TEST(AirspeedQuality, detects_flap_tones_through_platform_maximum)
{
	const float frequencies[] {2.f, 3.f, 4.f, 5.f, 5.5f, 6.8f};
	float samples[kSampleCount] {};

	for (float frequency_hz : frequencies) {
		fill_tone(samples, frequency_hz);
		const auto result = airspeed_quality::evaluate_spectrum(samples, kSampleCount, kSampleRateHz,
				    0.5f, 8.f, frequency_hz, 0.5f, 0.25f);
		EXPECT_TRUE(result.valid) << frequency_hz;
		EXPECT_GT(result.ratio, 0.25f) << frequency_hz;
	}
}

TEST(AirspeedQuality, detects_eight_hz_when_reference_band_contains_full_flap_band)
{
	float samples[kSampleCount] {};
	fill_tone(samples, 8.f);
	const auto result = airspeed_quality::evaluate_spectrum(samples, kSampleCount, kSampleRateHz,
			    0.5f, 9.f, 8.f, 0.5f, 0.25f);
	EXPECT_TRUE(result.valid);
	EXPECT_GT(result.ratio, 0.25f);
}

TEST(AirspeedQuality, rejects_flap_band_crossing_reference_boundary)
{
	float samples[kSampleCount] {};
	fill_tone(samples, 8.f);
	const auto result = airspeed_quality::evaluate_spectrum(samples, kSampleCount, kSampleRateHz,
			    0.5f, 8.f, 8.f, 0.5f, 0.25f);
	EXPECT_FALSE(result.valid);
	EXPECT_EQ(result.invalid_reason, airspeed_quality::SpectralInvalidReason::FlapBand);
}

TEST(AirspeedQuality, flap_tone_remains_detectable_with_slow_trend)
{
	float samples[kSampleCount] {};
	fill_tone(samples, 5.5f, 0.05f);
	float mean = 0.f;

	for (float sample : samples) { mean += sample; }

	mean /= kSampleCount;

	for (float &sample : samples) { sample -= mean; }

	const auto result = airspeed_quality::evaluate_spectrum(samples, kSampleCount, kSampleRateHz,
			    0.5f, 8.f, 5.5f, 0.5f, 0.25f);
	EXPECT_TRUE(result.valid);
	EXPECT_GT(result.ratio, 0.2f);
}

TEST(AirspeedQuality, broadband_noise_is_finite)
{
	float samples[kSampleCount] {};
	uint32_t state = 1;

	for (float &sample : samples) {
		state = state * 1664525U + 1013904223U;
		sample = static_cast<float>(state & 0xffffU) / 32768.f - 1.f;
	}

	const auto result = airspeed_quality::evaluate_spectrum(samples, kSampleCount, kSampleRateHz,
			    0.5f, 8.f, 5.f, 0.5f, 0.25f);
	EXPECT_TRUE(result.valid);
	EXPECT_TRUE(std::isfinite(result.ratio));
}

TEST(AirspeedQuality, rejects_zero_energy_and_nyquist_overlap)
{
	float samples[kSampleCount] {};
	const auto zero = airspeed_quality::evaluate_spectrum(samples, kSampleCount, kSampleRateHz,
			  0.5f, 8.f, 5.f, 0.5f, 0.25f);
	EXPECT_FALSE(zero.valid);
	EXPECT_EQ(zero.invalid_reason, airspeed_quality::SpectralInvalidReason::ZeroEnergy);

	fill_tone(samples, 8.f);
	const auto nyquist = airspeed_quality::evaluate_spectrum(samples, kSampleCount, kSampleRateHz,
			     0.5f, 25.f, 8.f, 0.5f, 0.25f);
	EXPECT_FALSE(nyquist.valid);
	EXPECT_EQ(nyquist.invalid_reason, airspeed_quality::SpectralInvalidReason::Nyquist);
}

TEST(AirspeedQuality, timestamp_policy_accepts_jitter_and_drops_but_rejects_bad_order)
{
	using airspeed_quality::TimestampStatus;
	EXPECT_EQ(airspeed_quality::validate_timestamp(1000000, 1019000, 1000000), TimestampStatus::Accepted);
	EXPECT_EQ(airspeed_quality::validate_timestamp(1000000, 1040000, 1000000), TimestampStatus::Accepted);
	EXPECT_EQ(airspeed_quality::validate_timestamp(1000000, 1000000, 1000000), TimestampStatus::Duplicate);
	EXPECT_EQ(airspeed_quality::validate_timestamp(1000000, 999999, 1000000), TimestampStatus::NonMonotonic);
	EXPECT_EQ(airspeed_quality::validate_timestamp(1000000, 2100000, 1000000), TimestampStatus::LongGap);
}

TEST(AirspeedQuality, strict_window_requires_all_two_hundred_samples)
{
	const int required = airspeed_quality::required_window_samples(50.f, 4.f, 256);
	EXPECT_EQ(required, 200);
	EXPECT_FALSE(airspeed_quality::window_ready(199, required));
	EXPECT_TRUE(airspeed_quality::window_ready(200, required));
	EXPECT_EQ(airspeed_quality::required_window_samples(50.f, 5.f, 256), 250);
	EXPECT_EQ(airspeed_quality::required_window_samples(NAN, 4.f, 256), 0);
}

TEST(AirspeedQuality, strict_window_refills_after_drop_and_parameter_change)
{
	int sample_count = 200;
	EXPECT_TRUE(airspeed_quality::window_ready(sample_count, 200));
	EXPECT_EQ(airspeed_quality::validate_timestamp(4000000, 4040001, 40000),
		  airspeed_quality::TimestampStatus::LongGap);
	sample_count = 0;
	EXPECT_FALSE(airspeed_quality::window_ready(sample_count, 200));
	sample_count = 199;
	EXPECT_FALSE(airspeed_quality::window_ready(sample_count, 200));
	EXPECT_TRUE(airspeed_quality::window_ready(++sample_count, 200));

	const int changed_window = airspeed_quality::required_window_samples(50.f, 5.f, 256);
	EXPECT_EQ(changed_window, 250);
	EXPECT_FALSE(airspeed_quality::window_ready(200, changed_window));
}

TEST(AirspeedQuality, ring_window_start_wraps_without_losing_samples)
{
	EXPECT_EQ(airspeed_quality::ring_window_start(200, 200, 256), 0);
	EXPECT_EQ(airspeed_quality::ring_window_start(0, 200, 256), 56);
	EXPECT_EQ(airspeed_quality::ring_window_start(44, 200, 256), 100);
	EXPECT_EQ(airspeed_quality::ring_window_start(0, 250, 256), 6);
	EXPECT_EQ(airspeed_quality::ring_window_start(0, 257, 256), -1);
}

TEST(AirspeedQuality, causal_snapshot_selects_earlier_and_equal_samples)
{
	airspeed_quality::SnapshotRing<float, 4> snapshots;
	ASSERT_TRUE(snapshots.push(1000000, 0.2f));
	ASSERT_TRUE(snapshots.push(1020000, 0.4f));

	const auto earlier = snapshots.select(1010000, 200000);
	ASSERT_TRUE(earlier.causal());
	EXPECT_TRUE(earlier.fresh());
	EXPECT_EQ(earlier.timestamp_sample, 1000000ULL);
	EXPECT_EQ(earlier.age_us, 10000U);
	EXPECT_FLOAT_EQ(*earlier.value, 0.2f);

	const auto equal = snapshots.select(1020000, 200000);
	ASSERT_TRUE(equal.causal());
	EXPECT_TRUE(equal.fresh());
	EXPECT_EQ(equal.timestamp_sample, 1020000ULL);
	EXPECT_EQ(equal.age_us, 0U);
	EXPECT_FLOAT_EQ(*equal.value, 0.4f);
}

TEST(AirspeedQuality, causal_snapshot_rejects_future_and_stale_quality)
{
	using airspeed_quality::SnapshotSelectionStatus;
	airspeed_quality::SnapshotRing<float, 4> snapshots;
	ASSERT_TRUE(snapshots.push(1020000, 0.4f));

	const auto future = snapshots.select(1000000, 200000);
	EXPECT_FALSE(future.causal());
	EXPECT_EQ(future.status, SnapshotSelectionStatus::FutureOnly);

	const auto stale = snapshots.select(1300001, 200000);
	ASSERT_TRUE(stale.causal());
	EXPECT_FALSE(stale.fresh());
	EXPECT_EQ(stale.status, SnapshotSelectionStatus::Stale);
	EXPECT_EQ(stale.age_us, 280001U);
}

TEST(AirspeedQuality, causal_snapshot_wrap_reset_and_startup_are_deterministic)
{
	using airspeed_quality::SnapshotSelectionStatus;
	airspeed_quality::SnapshotRing<int, 3> snapshots;
	EXPECT_EQ(snapshots.select(1000000, 200000).status, SnapshotSelectionStatus::NoHistory);

	ASSERT_TRUE(snapshots.push(1000000, 1));
	ASSERT_TRUE(snapshots.push(1020000, 2));
	ASSERT_TRUE(snapshots.push(1040000, 3));
	ASSERT_TRUE(snapshots.push(1060000, 4));
	EXPECT_EQ(snapshots.size(), 3U);
	const auto wrapped = snapshots.select(1050000, 200000);
	ASSERT_TRUE(wrapped.causal());
	EXPECT_EQ(*wrapped.value, 3);

	snapshots.reset();
	EXPECT_TRUE(snapshots.empty());
	EXPECT_EQ(snapshots.select(1100000, 200000).status, SnapshotSelectionStatus::NoHistory);
	ASSERT_TRUE(snapshots.push(1120000, 5));
	EXPECT_FALSE(snapshots.push(1120000, 6));
}

TEST(AirspeedQuality, causal_snapshot_gap_reset_discards_pre_gap_history)
{
	using airspeed_quality::SnapshotSelectionStatus;
	airspeed_quality::SnapshotRing<float, 4> snapshots;
	ASSERT_TRUE(snapshots.push(1000000, 0.2f));
	ASSERT_TRUE(snapshots.push(1020000, 0.4f));

	// The EKF2 producer resets the ring when the 20 ms input grid has a gap.
	snapshots.reset();
	ASSERT_TRUE(snapshots.push(1200000, 0.8f));
	EXPECT_EQ(snapshots.select(1100000, 200000).status, SnapshotSelectionStatus::FutureOnly);

	const auto after_gap = snapshots.select(1220000, 200000);
	ASSERT_TRUE(after_gap.causal());
	EXPECT_EQ(after_gap.timestamp_sample, 1200000ULL);
	EXPECT_FLOAT_EQ(*after_gap.value, 0.8f);
}

TEST(AirspeedQuality, observation_diagnostic_uses_exact_queued_sample)
{
	estimator::airspeedSample sample{};
	sample.time_us = 900000;
	sample.eas2tas = 1.25f;
	sample.noise_var = 7.5f;
	sample.fuse_enabled = false;
	ekf2_airspeed_quality_s diagnostic{};
	airspeed_quality::set_observation_diagnostic(diagnostic, sample, 1000000, 900000, 980000,
			1, 1234, 0, 1234);
	EXPECT_EQ(diagnostic.timestamp_sample, 1000000ULL);
	EXPECT_EQ(diagnostic.ekf_buffer_timestamp_sample, 900000ULL);
	EXPECT_EQ(diagnostic.quality_timestamp_sample, 980000ULL);
	EXPECT_EQ(diagnostic.quality_age_us, 20000U);
	EXPECT_EQ(diagnostic.airspeed_source, 1);
	EXPECT_EQ(diagnostic.airspeed_device_id, 1234U);
	EXPECT_EQ(diagnostic.quality_source_instance, 0);
	EXPECT_EQ(diagnostic.quality_device_id, 1234U);
	EXPECT_FLOAT_EQ(diagnostic.eas2tas, sample.eas2tas);
	EXPECT_FLOAT_EQ(diagnostic.r_as_used, sample.noise_var);
	EXPECT_EQ(diagnostic.fuse_enabled, sample.fuse_enabled);
}

TEST(AirspeedQuality, varying_modes_map_to_exact_per_sample_diagnostics)
{
	struct TestCase {
		float base_noise;
		float eas2tas;
		float quality;
		float rmax;
		float rcst;
		int32_t mode;
		bool fuse_enabled;
	};

	const TestCase cases[] {
		{1.4f, 1.0f, 0.2f, 5.f, 3.f, 0, true},
		{1.4f, 1.2f, 0.8f, 5.f, 3.f, 1, true},
		{2.0f, 1.5f, 0.5f, 4.f, 2.f, 2, true},
		{1.0f, 2.0f, 0.1f, 5.f, 2.f, 3, false},
	};

	uint64_t timestamp = 1000000;

	for (const TestCase &test_case : cases) {
		const float nominal_variance = test_case.base_noise * test_case.eas2tas
					       * test_case.base_noise * test_case.eas2tas;
		const float used_variance = airspeed_quality::observation_variance(nominal_variance,
					    test_case.quality, test_case.rmax, test_case.rcst,
					    airspeed_quality::mode_config(test_case.mode));
		estimator::airspeedSample sample {
			.time_us = timestamp,
			.true_airspeed = 20.f,
			.eas2tas = test_case.eas2tas,
			.noise_var = used_variance,
			.quality = test_case.quality,
			.fuse_enabled = test_case.fuse_enabled,
		};
		ekf2_airspeed_quality_s diagnostic{};
		airspeed_quality::set_observation_diagnostic(diagnostic, sample, timestamp, timestamp - 50000,
				timestamp - 20000, 1, 1234, 0, 1234);
		EXPECT_FLOAT_EQ(diagnostic.r_as_used, sample.noise_var);
		EXPECT_FLOAT_EQ(diagnostic.eas2tas, sample.eas2tas);
		EXPECT_EQ(diagnostic.fuse_enabled, sample.fuse_enabled);
		EXPECT_EQ(diagnostic.timestamp_sample, sample.time_us);
		EXPECT_EQ(diagnostic.ekf_buffer_timestamp_sample, timestamp - 50000);
		timestamp += 100000;
	}
}
