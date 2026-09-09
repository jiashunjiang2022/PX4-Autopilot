#include <gtest/gtest.h>

#include "../AirspeedQualityMath.hpp"
#include "../AirspeedQualityDiagnostics.hpp"


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

TEST(AirspeedQuality, monitoring_diagnostic_does_not_claim_an_ekf_observation)
{
	ekf2_airspeed_quality_s diagnostic{};
	diagnostic.airspeed_q = 0.1f;
	airspeed_quality::set_monitoring_diagnostic(diagnostic, 1020000, 0, 1234);
	EXPECT_TRUE(diagnostic.qmon);
	EXPECT_EQ(diagnostic.timestamp_sample, 0ULL);
	EXPECT_EQ(diagnostic.ekf_buffer_timestamp_sample, 0ULL);
	EXPECT_EQ(diagnostic.quality_timestamp_sample, 1020000ULL);
	EXPECT_EQ(diagnostic.quality_age_us, UINT32_MAX);
	EXPECT_EQ(diagnostic.airspeed_source, -1);
	EXPECT_EQ(diagnostic.airspeed_device_id, 0U);
	EXPECT_EQ(diagnostic.quality_source_instance, 0);
	EXPECT_EQ(diagnostic.quality_device_id, 1234U);
	EXPECT_TRUE(std::isnan(diagnostic.eas2tas));
	EXPECT_TRUE(std::isnan(diagnostic.nominal_r_as));
	EXPECT_TRUE(std::isnan(diagnostic.r_as_used));
	EXPECT_FLOAT_EQ(diagnostic.airspeed_q, 0.1f);
}
