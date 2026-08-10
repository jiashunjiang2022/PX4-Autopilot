#include <gtest/gtest.h>

#include "AirspeedQualityInput.hpp"

#include <lib/airspeed/airspeed.h>
#include <uORB/topics/airspeed_quality_input.h>

#include <array>
#include <algorithm>
#include <cmath>
#include <iterator>
#include <vector>

namespace
{

constexpr uint64_t kOutputIntervalUs{20000};
constexpr uint64_t kMaxSourceGapUs{40000};
constexpr float kMinSourceRateHz{48.f};
constexpr float kMaxSourceRateHz{100.f};
constexpr float kRateReconfigureFraction{0.20f};
constexpr uint8_t kRateStableSamples{10};

class QualityInputResamplerHarness
{
public:
	struct Output {
		uint64_t timestamp;
		uint64_t previous_source_timestamp;
		uint64_t current_source_timestamp;
		float value;
		float previous_value;
		float current_value;
		uint32_t rate_reset_counter;
	};

	bool push(uint64_t timestamp, float value)
	{
		if (_rate_previous_timestamp != 0) {
			if (timestamp == _rate_previous_timestamp) {
				_gap_count++;
				reset_input(airspeed_quality_input_s::RESET_REASON_DUPLICATE);
				return false;

			} else if (timestamp < _rate_previous_timestamp) {
				_gap_count++;
				reset_input(airspeed_quality_input_s::RESET_REASON_NON_MONOTONIC);
				return false;
			}

			const uint64_t source_dt_us = timestamp - _rate_previous_timestamp;

			if (source_dt_us > kMaxSourceGapUs) {
				_gap_count++;
				reset_input(airspeed_quality_input_s::RESET_REASON_LONG_GAP);
				_rate_previous_timestamp = timestamp;
				return false;
			}

			const float source_dt_s = source_dt_us * 1e-6f;
			const float instantaneous_rate_hz = 1.f / source_dt_s;
			const float alpha = std::clamp(source_dt_s / (1.f + source_dt_s), 0.f, 1.f);
			_measured_source_rate_hz = std::isfinite(_measured_source_rate_hz)
						   ? _measured_source_rate_hz + alpha * (instantaneous_rate_hz - _measured_source_rate_hz)
						   : instantaneous_rate_hz;
			_rate_sample_count = std::min(static_cast<int>(_rate_sample_count) + 1, 255);
		}

		_rate_previous_timestamp = timestamp;

		if (_rate_sample_count < kRateStableSamples) {
			return false;
		}

		const bool rate_valid = airspeed_quality_input::source_rate_valid(_measured_source_rate_hz,
					     _rate_sample_count, kMinSourceRateHz, kMaxSourceRateHz, kRateStableSamples);

		if (!rate_valid) {
			_rate_invalid_count++;

			if (_rate_valid || _filter_initialized) {
				_rate_reset_counter++;
				reset_filter(airspeed_quality_input_s::RESET_REASON_RATE_INVALID);
			}

			_rate_valid = false;
			_reset_reason = airspeed_quality_input_s::RESET_REASON_RATE_INVALID;
			return false;
		}

		_rate_valid = true;

		if (airspeed_quality_input::source_rate_requires_reconfigure(_measured_source_rate_hz,
				_filter_source_rate_hz, kRateReconfigureFraction)) {
			_rate_reset_counter++;
			reset_filter(airspeed_quality_input_s::RESET_REASON_RATE_CHANGE);
		}

		if (!_filter_initialized) {
			_filter_source_rate_hz = _measured_source_rate_hz;
			_previous_value = value;
			_previous_source_timestamp = timestamp;
			_next_output_timestamp = ((timestamp / kOutputIntervalUs) + 1) * kOutputIntervalUs;
			_filter_initialized = true;
			return false;
		}

		while (airspeed_quality_input::bracketed(_previous_source_timestamp, timestamp,
				_next_output_timestamp)) {
			_outputs.push_back({
				_next_output_timestamp,
				_previous_source_timestamp,
				timestamp,
				airspeed_quality_input::interpolate(_previous_value, value, _previous_source_timestamp,
						timestamp, _next_output_timestamp),
				_previous_value,
				value,
				_rate_reset_counter
			});
			_next_output_timestamp += kOutputIntervalUs;
		}

		_previous_source_timestamp = timestamp;
		_previous_value = value;
		_reset_reason = airspeed_quality_input_s::RESET_REASON_NONE;
		return true;
	}

	bool valid() const { return _rate_valid && _filter_initialized; }
	float measured_source_rate_hz() const { return _measured_source_rate_hz; }
	float filter_source_rate_hz() const { return _filter_source_rate_hz; }
	uint32_t gap_count() const { return _gap_count; }
	uint32_t rate_reset_counter() const { return _rate_reset_counter; }
	uint32_t rate_invalid_count() const { return _rate_invalid_count; }
	uint8_t reset_reason() const { return _reset_reason; }
	const std::vector<Output> &outputs() const { return _outputs; }

private:
	void reset_input(uint8_t reason)
	{
		_rate_previous_timestamp = 0;
		_measured_source_rate_hz = NAN;
		_rate_sample_count = 0;
		_rate_valid = false;
		reset_filter(reason);
	}

	void reset_filter(uint8_t reason)
	{
		_previous_source_timestamp = 0;
		_next_output_timestamp = 0;
		_previous_value = NAN;
		_filter_source_rate_hz = NAN;
		_filter_initialized = false;
		_reset_reason = reason;
	}

	std::vector<Output> _outputs;
	uint64_t _rate_previous_timestamp{0};
	uint64_t _previous_source_timestamp{0};
	uint64_t _next_output_timestamp{0};
	float _previous_value{NAN};
	float _measured_source_rate_hz{NAN};
	float _filter_source_rate_hz{NAN};
	uint32_t _gap_count{0};
	uint32_t _rate_reset_counter{0};
	uint32_t _rate_invalid_count{0};
	uint8_t _rate_sample_count{0};
	uint8_t _reset_reason{airspeed_quality_input_s::RESET_REASON_INITIALIZATION};
	bool _filter_initialized{false};
	bool _rate_valid{false};
};

} // namespace

TEST(AirspeedQualityInput, source_identity_changes_are_explicit)
{
	EXPECT_FALSE(airspeed_quality_input::source_device_valid(0));
	EXPECT_TRUE(airspeed_quality_input::source_device_valid(42));
	EXPECT_FALSE(airspeed_quality_input::source_identity_changed(0, 0, 42, 0));
	EXPECT_FALSE(airspeed_quality_input::source_identity_changed(42, 0, 42, 0));
	EXPECT_TRUE(airspeed_quality_input::source_identity_changed(42, 0, 43, 0));
	EXPECT_TRUE(airspeed_quality_input::source_identity_changed(42, 0, 42, 1));
}

TEST(AirspeedQualityInput, signed_pressure_is_continuous_and_range_checked)
{
	const float positive = calc_IAS_corrected(AIRSPEED_COMPENSATION_MODEL_PITOT, AIRSPEED_SENSOR_MODEL_MEMBRANE,
			       0.2f, 1.5f, 4.f, 101325.f, 15.f);
	const float negative = calc_IAS_corrected(AIRSPEED_COMPENSATION_MODEL_PITOT, AIRSPEED_SENSOR_MODEL_MEMBRANE,
			       0.2f, 1.5f, -4.f, 101325.f, 15.f);
	const float zero = calc_IAS_corrected(AIRSPEED_COMPENSATION_MODEL_PITOT, AIRSPEED_SENSOR_MODEL_MEMBRANE,
			   0.2f, 1.5f, 0.f, 101325.f, 15.f);
	EXPECT_GT(positive, 0.f);
	EXPECT_LT(negative, 0.f);
	EXPECT_FLOAT_EQ(zero, 0.f);
	EXPECT_FLOAT_EQ(positive, -negative);
	EXPECT_TRUE(airspeed_quality_input::pressure_in_range(4.f, 7000.f));
	EXPECT_TRUE(airspeed_quality_input::pressure_in_range(0.f, 7000.f));
	EXPECT_TRUE(airspeed_quality_input::pressure_in_range(-0.01f, 7000.f));
	EXPECT_TRUE(airspeed_quality_input::pressure_in_range(-6894.757f, 7000.f));
	EXPECT_FALSE(airspeed_quality_input::pressure_in_range(7001.f, 7000.f));
	EXPECT_FALSE(airspeed_quality_input::pressure_in_range(NAN, 7000.f));
	EXPECT_FALSE(airspeed_quality_input::pressure_in_range(INFINITY, 7000.f));
}

TEST(AirspeedQualityInput, rate_contract_requires_stability_and_reconfigures_sparingly)
{
	EXPECT_FALSE(airspeed_quality_input::source_rate_valid(50.1f, 9, kMinSourceRateHz, kMaxSourceRateHz, 10));
	EXPECT_TRUE(airspeed_quality_input::source_rate_valid(48.f, 10, kMinSourceRateHz, kMaxSourceRateHz, 10));
	EXPECT_TRUE(airspeed_quality_input::source_rate_valid(50.1f, 10, kMinSourceRateHz, kMaxSourceRateHz, 10));
	EXPECT_TRUE(airspeed_quality_input::source_rate_valid(52.f, 10, kMinSourceRateHz, kMaxSourceRateHz, 10));
	EXPECT_TRUE(airspeed_quality_input::source_rate_valid(59.f, 10, kMinSourceRateHz, kMaxSourceRateHz, 10));
	EXPECT_TRUE(airspeed_quality_input::source_rate_valid(100.f, 10, kMinSourceRateHz, kMaxSourceRateHz, 10));
	EXPECT_FALSE(airspeed_quality_input::source_rate_valid(45.f, 10, kMinSourceRateHz, kMaxSourceRateHz, 10));
	EXPECT_FALSE(airspeed_quality_input::source_rate_valid(101.f, 10, kMinSourceRateHz, kMaxSourceRateHz, 10));
	EXPECT_FALSE(airspeed_quality_input::source_rate_requires_reconfigure(72.f, 60.f, kRateReconfigureFraction));
	EXPECT_TRUE(airspeed_quality_input::source_rate_requires_reconfigure(72.1f, 60.f, kRateReconfigureFraction));
}

TEST(AirspeedQualityInput, timestamp_failures_reset_and_invalidate)
{
	QualityInputResamplerHarness duplicate;
	EXPECT_FALSE(duplicate.push(1000000, 1.f));
	EXPECT_FALSE(duplicate.push(1000000, 2.f));
	EXPECT_FALSE(duplicate.valid());
	EXPECT_EQ(duplicate.reset_reason(), airspeed_quality_input_s::RESET_REASON_DUPLICATE);

	QualityInputResamplerHarness non_monotonic;
	EXPECT_FALSE(non_monotonic.push(1000000, 1.f));
	EXPECT_FALSE(non_monotonic.push(999999, 2.f));
	EXPECT_FALSE(non_monotonic.valid());
	EXPECT_EQ(non_monotonic.reset_reason(), airspeed_quality_input_s::RESET_REASON_NON_MONOTONIC);

	QualityInputResamplerHarness long_gap;
	EXPECT_FALSE(long_gap.push(1000000, 1.f));
	EXPECT_FALSE(long_gap.push(1040001, 2.f));
	EXPECT_FALSE(long_gap.valid());
	EXPECT_EQ(long_gap.reset_reason(), airspeed_quality_input_s::RESET_REASON_LONG_GAP);
}

TEST(AirspeedQualityInput, realistic_ms4525do_timing_is_stable_and_bracketed)
{
	// Median 13.53 ms, P95/max 26.84 ms, and mean rate 59.33 Hz over each deterministic cycle.
	constexpr std::array<uint64_t, 20> source_intervals_us {
		13530, 19230, 13530, 19230, 13530, 26840, 13530, 19230, 13530, 19230,
		13530, 26840, 13530, 19230, 13530, 19230, 13530, 19230, 13530, 13530
	};
	QualityInputResamplerHarness resampler;
	uint64_t source_timestamp = 1000000;
	uint32_t settled_reset_count = 0;

	for (int index = 0; index < 1200; ++index) {
		source_timestamp += source_intervals_us[index % source_intervals_us.size()];
		const float filtered_pressure = source_timestamp * 1e-6f;
		resampler.push(source_timestamp, filtered_pressure);

		if (index == 900) {
			settled_reset_count = resampler.rate_reset_counter();
		}
	}

	EXPECT_TRUE(resampler.valid());
	EXPECT_NEAR(resampler.measured_source_rate_hz(), 59.33f, 1.f);
	EXPECT_LT(std::fabs(resampler.filter_source_rate_hz() - resampler.measured_source_rate_hz()),
		resampler.filter_source_rate_hz() * kRateReconfigureFraction);
	EXPECT_EQ(resampler.gap_count(), 0u);
	EXPECT_EQ(resampler.rate_invalid_count(), 0u);
	EXPECT_EQ(resampler.rate_reset_counter(), settled_reset_count);
	EXPECT_LE(resampler.rate_reset_counter(), 1u);

	const auto &outputs = resampler.outputs();
	ASSERT_GT(outputs.size(), 500u);
	const uint32_t final_reset_count = resampler.rate_reset_counter();
	auto steady_begin = std::find_if(outputs.begin(), outputs.end(), [final_reset_count](const auto &output) {
		return output.rate_reset_counter == final_reset_count;
	});
	ASSERT_NE(steady_begin, outputs.end());
	ASSERT_GT(std::distance(steady_begin, outputs.end()), 500);

	for (size_t index = 0; index < outputs.size(); ++index) {
		const auto &output = outputs[index];
		EXPECT_GE(output.timestamp, output.previous_source_timestamp);
		EXPECT_LE(output.timestamp, output.current_source_timestamp);
		EXPECT_NEAR(output.value, output.timestamp * 1e-6f, 1e-5f);

		if (output.timestamp > output.previous_source_timestamp
		    && output.timestamp < output.current_source_timestamp) {
			EXPECT_GT(output.value, output.previous_value);
			EXPECT_LT(output.value, output.current_value);
		}

		if (index > 0) {
			EXPECT_GT(output.timestamp, outputs[index - 1].timestamp);
		}
	}

	for (auto output = steady_begin + 1; output != outputs.end(); ++output) {
		EXPECT_EQ(output->timestamp - (output - 1)->timestamp, kOutputIntervalUs);
	}

	const float output_rate_hz = 1e6f * static_cast<float>(std::distance(steady_begin, outputs.end()) - 1)
				     / static_cast<float>(outputs.back().timestamp - steady_begin->timestamp);
	EXPECT_FLOAT_EQ(output_rate_hz, 50.f);
}

TEST(AirspeedQualityInput, realistic_skye_timing_is_valid_and_bracketed)
{
	// Normal SKYE jitter is 19.3-21.0 ms; one measured 39.729 ms interval remains below the hard gap limit.
	constexpr std::array<uint64_t, 16> source_intervals_us {
		19320, 20110, 19880, 20379, 19560, 20981, 19917, 19740,
		20060, 19680, 20420, 19830, 20190, 19490, 21000, 19990
	};
	QualityInputResamplerHarness resampler;
	uint64_t source_timestamp = 1000000;
	uint32_t settled_reset_count = 0;
	size_t outputs_before_long_interval = 0;
	size_t outputs_after_long_interval = 0;

	for (int index = 0; index < 1200; ++index) {
		const uint64_t source_interval_us = (index == 602) ? 39729 :
						    source_intervals_us[index % source_intervals_us.size()];

		if (index == 602) {
			outputs_before_long_interval = resampler.outputs().size();
		}

		source_timestamp += source_interval_us;
		resampler.push(source_timestamp, source_timestamp * 1e-6f);

		if (index == 602) {
			outputs_after_long_interval = resampler.outputs().size();
		}

		if (index == 900) {
			settled_reset_count = resampler.rate_reset_counter();
		}
	}

	EXPECT_TRUE(resampler.valid());
	EXPECT_NEAR(resampler.measured_source_rate_hz(), 50.1f, 0.5f);
	EXPECT_EQ(resampler.gap_count(), 0u);
	EXPECT_EQ(resampler.rate_invalid_count(), 0u);
	EXPECT_EQ(resampler.rate_reset_counter(), settled_reset_count);
	EXPECT_LE(resampler.rate_reset_counter(), 1u);
	EXPECT_EQ(outputs_after_long_interval - outputs_before_long_interval, 2u);

	const auto &outputs = resampler.outputs();
	ASSERT_GT(outputs.size(), 1000u);
	uint32_t quality_input_missed_count = 0;

	for (size_t index = 0; index < outputs.size(); ++index) {
		const auto &output = outputs[index];
		EXPECT_GE(output.timestamp, output.previous_source_timestamp);
		EXPECT_LE(output.timestamp, output.current_source_timestamp);
		EXPECT_NEAR(output.value, output.timestamp * 1e-6f, 1e-5f);

		if (output.timestamp > output.previous_source_timestamp
		    && output.timestamp < output.current_source_timestamp) {
			EXPECT_GT(output.value, output.previous_value);
			EXPECT_LT(output.value, output.current_value);
		}

		if (index > 0) {
			const uint64_t output_interval_us = output.timestamp - outputs[index - 1].timestamp;
			EXPECT_EQ(output_interval_us, kOutputIntervalUs);
			quality_input_missed_count += output_interval_us == kOutputIntervalUs ? 0 : 1;
		}
	}

	EXPECT_EQ(quality_input_missed_count, 0u);
}

TEST(AirspeedQualityInput, genuine_source_rate_change_reconfigures)
{
	QualityInputResamplerHarness resampler;
	uint64_t source_timestamp = 1000000;

	for (int index = 0; index < 160; ++index) {
		source_timestamp += 16835;
		resampler.push(source_timestamp, source_timestamp * 1e-6f);
	}

	ASSERT_TRUE(resampler.valid());
	const uint32_t stable_reset_count = resampler.rate_reset_counter();

	// A sustained approximately 83 Hz producer is a genuine >20% source-rate change from 59 Hz.
	for (int index = 0; index < 1000; ++index) {
		source_timestamp += 12000;
		resampler.push(source_timestamp, source_timestamp * 1e-6f);
	}

	EXPECT_TRUE(resampler.valid());
	EXPECT_GT(resampler.measured_source_rate_hz(), 70.f);
	EXPECT_GT(resampler.rate_reset_counter(), stable_reset_count);
	EXPECT_EQ(resampler.rate_invalid_count(), 0u);
}

TEST(AirspeedQualityInput, interpolation_requires_two_real_bracketing_samples)
{
	EXPECT_TRUE(airspeed_quality_input::bracketed(10000, 30000, 20000));
	EXPECT_FLOAT_EQ(airspeed_quality_input::interpolate(-2.f, 2.f, 10000, 30000, 20000), 0.f);
	EXPECT_FALSE(airspeed_quality_input::bracketed(10000, 30000, 40000));
	EXPECT_FALSE(airspeed_quality_input::bracketed(10000, 10000, 10000));
}
