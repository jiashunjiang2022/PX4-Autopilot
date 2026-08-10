#include <gtest/gtest.h>

#include "../AirspeedQualityMath.hpp"
#include "../AirspeedQualityDiagnostics.hpp"
#include "../AirspeedQualitySnapshot.hpp"

#include <lib/airspeed/AirspeedQualityMode.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

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

class FlapFreshnessPipelineHarness
{
public:
	void publish_flap_frequency(uint64_t timestamp, float frequency_hz)
	{
		_flap_timestamp = timestamp;
		_flap_frequency_hz = frequency_hz;
	}

	void update_quality(uint64_t timestamp, uint64_t freshness_reference)
	{
		_timed_out = airspeed_quality::flap_frequency_timed_out(freshness_reference, _flap_timestamp, kTimeoutUs);
		const bool frequency_valid = std::isfinite(_flap_frequency_hz) && _flap_frequency_hz > 0.f;

		if (_flap_active) {
			if (_timed_out) {
				_flap_active = false;
				_below_off_since = 0;

			} else if (!frequency_valid || _flap_frequency_hz < kFlapOffHz) {
				if (_below_off_since == 0) {
					_below_off_since = timestamp;
				}

				if ((timestamp - _below_off_since) >= kFlapOffUs) {
					_flap_active = false;
					_below_off_since = 0;
				}

			} else {
				_below_off_since = 0;
			}

			_above_on_since = 0;

		} else {
			if (!_timed_out && frequency_valid && _flap_frequency_hz > kFlapOnHz) {
				if (_above_on_since == 0) {
					_above_on_since = timestamp;
				}

				if ((timestamp - _above_on_since) >= kFlapOnUs) {
					_flap_active = true;
					_above_on_since = 0;
				}

			} else {
				_above_on_since = 0;
			}

			_below_off_since = 0;
		}

		if (_flap_active) {
			_last_flap_true = timestamp;
		}

		const bool flap_recently_true = (_last_flap_true > 0) && (timestamp >= _last_flap_true)
						&& ((timestamp - _last_flap_true) <= kFlapGraceUs);

		if (!_flap_active && !flap_recently_true) {
			_window_count = 0;
			_no_recent_flap_reset_count++;
			return;
		}

		_window_count = std::min(_window_count + 1, kWindowSamples);

		if (_window_count == kWindowSamples
		    && (_last_evaluation == 0 || (timestamp - _last_evaluation) >= kEvaluationIntervalUs)) {
			_last_evaluation = timestamp;
			_evaluation_timestamps.push_back(timestamp);
		}
	}

	bool timed_out() const { return _timed_out; }
	bool flap_active() const { return _flap_active; }
	int window_count() const { return _window_count; }
	uint32_t no_recent_flap_reset_count() const { return _no_recent_flap_reset_count; }
	uint64_t flap_timestamp() const { return _flap_timestamp; }
	const std::vector<uint64_t> &evaluation_timestamps() const { return _evaluation_timestamps; }

private:
	static constexpr uint64_t kTimeoutUs{800000};
	static constexpr uint64_t kFlapOnUs{400000};
	static constexpr uint64_t kFlapOffUs{1500000};
	static constexpr uint64_t kFlapGraceUs{700000};
	static constexpr uint64_t kEvaluationIntervalUs{500000};
	static constexpr float kFlapOnHz{1.f};
	static constexpr float kFlapOffHz{0.6f};
	static constexpr int kWindowSamples{200};
	std::vector<uint64_t> _evaluation_timestamps;
	uint64_t _flap_timestamp{0};
	uint64_t _above_on_since{0};
	uint64_t _below_off_since{0};
	uint64_t _last_flap_true{0};
	uint64_t _last_evaluation{0};
	float _flap_frequency_hz{NAN};
	uint32_t _no_recent_flap_reset_count{0};
	int _window_count{0};
	bool _timed_out{true};
	bool _flap_active{false};
};

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

TEST(AirspeedQuality, flap_frequency_freshness_handles_missing_stale_and_future_samples)
{
	constexpr uint64_t timeout_us = 800000;
	constexpr uint64_t reference_timestamp = 5000000;

	EXPECT_TRUE(airspeed_quality::flap_frequency_timed_out(reference_timestamp, 0, timeout_us));
	EXPECT_TRUE(airspeed_quality::flap_frequency_timed_out(reference_timestamp,
			reference_timestamp - 801000, timeout_us));
	EXPECT_FALSE(airspeed_quality::flap_frequency_timed_out(reference_timestamp,
			reference_timestamp - 799000, timeout_us));

	for (uint64_t newer_offset_us : {1000ULL, 5000ULL, 10000ULL, 20000ULL}) {
		EXPECT_FALSE(airspeed_quality::flap_frequency_timed_out(reference_timestamp,
				reference_timestamp + newer_offset_us, timeout_us)) << newer_offset_us;
	}
}

TEST(AirspeedQuality, replay_flap_freshness_uses_deterministic_log_timestamps)
{
	constexpr uint64_t timeout_us = 800000;
	constexpr uint64_t replay_cycle_timestamp = 10000000;

	EXPECT_FALSE(airspeed_quality::flap_frequency_timed_out(replay_cycle_timestamp,
			replay_cycle_timestamp - 800000, timeout_us));
	EXPECT_TRUE(airspeed_quality::flap_frequency_timed_out(replay_cycle_timestamp,
			replay_cycle_timestamp - 800001, timeout_us));
	EXPECT_FALSE(airspeed_quality::flap_frequency_timed_out(replay_cycle_timestamp,
			replay_cycle_timestamp + 20000, timeout_us));
}

TEST(AirspeedQuality, continuous_asynchronous_flap_stream_preserves_spectral_delivery)
{
	constexpr uint64_t start_timestamp = 1000000;
	constexpr uint64_t quality_interval_us = 20000;
	constexpr uint64_t flap_interval_us = 10000;
	constexpr uint64_t flap_future_offset_us = 5000;
	FlapFreshnessPipelineHarness harness;
	bool became_active = false;
	bool reached_full_window = false;
	uint32_t resets_at_activation = 0;
	uint64_t activation_timestamp = 0;
	uint64_t final_quality_timestamp = 0;

	for (int quality_index = 0; quality_index <= 600; ++quality_index) {
		const uint64_t quality_timestamp = start_timestamp + quality_index * quality_interval_us;
		const uint64_t latest_flap_timestamp = quality_timestamp + flap_future_offset_us;
		harness.publish_flap_frequency(latest_flap_timestamp - flap_interval_us, 3.2f);
		harness.publish_flap_frequency(latest_flap_timestamp, 3.2f);
		harness.update_quality(quality_timestamp, quality_timestamp);
		final_quality_timestamp = quality_timestamp;

		EXPECT_FALSE(harness.timed_out());

		if (harness.flap_active() && !became_active) {
			became_active = true;
			resets_at_activation = harness.no_recent_flap_reset_count();
			activation_timestamp = quality_timestamp;
		}

		if (became_active) {
			EXPECT_TRUE(harness.flap_active());
			EXPECT_EQ(harness.no_recent_flap_reset_count(), resets_at_activation);
		}

		if (harness.window_count() == 200) {
			reached_full_window = true;
		}

		if (reached_full_window) {
			EXPECT_EQ(harness.window_count(), 200);
		}
	}

	ASSERT_TRUE(became_active);
	EXPECT_EQ(activation_timestamp - start_timestamp, 400000u);
	EXPECT_TRUE(reached_full_window);
	EXPECT_EQ(harness.window_count(), 200);
	ASSERT_GT(harness.evaluation_timestamps().size(), 10u);

	for (size_t index = 1; index < harness.evaluation_timestamps().size(); ++index) {
		EXPECT_EQ(harness.evaluation_timestamps()[index] - harness.evaluation_timestamps()[index - 1], 500000u);
	}

	const uint32_t resets_before_disappearance = harness.no_recent_flap_reset_count();
	bool observed_true_timeout = false;

	for (int quality_index = 1; quality_index <= 100; ++quality_index) {
		const uint64_t quality_timestamp = final_quality_timestamp + quality_index * quality_interval_us;
		harness.update_quality(quality_timestamp, quality_timestamp);

		if (harness.timed_out() && !observed_true_timeout) {
			observed_true_timeout = true;
			EXPECT_GT(quality_timestamp - harness.flap_timestamp(), 800000u);
			EXPECT_FALSE(harness.flap_active());
		}
	}

	EXPECT_TRUE(observed_true_timeout);
	EXPECT_FALSE(harness.flap_active());
	EXPECT_EQ(harness.window_count(), 0);
	EXPECT_GT(harness.no_recent_flap_reset_count(), resets_before_disappearance);
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
