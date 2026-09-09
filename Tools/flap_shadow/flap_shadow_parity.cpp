#include "../../src/modules/flap_aug_shadow/fast_predictor.hpp"
#include "../../src/modules/flap_aug_shadow/slow_engineering_shadow.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>

using namespace flap_aug_shadow;

namespace
{
template<typename T>
bool read(std::ifstream &stream, T &value)
{
	return static_cast<bool>(stream.read(reinterpret_cast<char *>(&value), sizeof(value)));
}

bool header(std::ifstream &stream, const char expected[8], uint32_t &count)
{
	char actual[8] {};
	return static_cast<bool>(stream.read(actual, sizeof(actual)))
	       && std::memcmp(actual, expected, sizeof(actual)) == 0 && read(stream, count);
}

float difference(float actual, float expected)
{
	return std::fabs(actual - expected);
}

bool v2_parity(const char *path, float &roll_error, float &base_error, float &temporal_error)
{
	std::ifstream stream(path, std::ios::binary);
	uint32_t count = 0;

	if (!header(stream, "FASV2G1", count)) { return false; }

	for (uint32_t vector = 0; vector < count; ++vector) {
		FastPredictor predictor;
		FastPredictor::Result result{};
		float last[FastPredictor::BaseFeatureCount] {};

		for (size_t sample = 0; sample < FastPredictor::HistorySamples; ++sample) {
			float features[FastPredictor::BaseFeatureCount] {};

			if (!stream.read(reinterpret_cast<char *>(features), sizeof(features))) { return false; }

			std::memcpy(last, features, sizeof(last));
			result = predictor.update(features, true, true);
		}

		float expected[4] {};

		if (!stream.read(reinterpret_cast<char *>(expected), sizeof(expected))) { return false; }

		roll_error = std::max(roll_error, difference(result.roll, expected[0]));
		base_error = std::max(base_error, difference(FastPredictor::predict_pitch_base(last), expected[1]));
		temporal_error = std::max(temporal_error, difference(result.pitch, expected[3]));

		if (!result.roll_valid || !result.pitch_valid) { return false; }
	}

	return roll_error <= 1e-5f && base_error <= 1e-5f && temporal_error <= 1e-5f;
}

bool v3_model_parity(const char *path, float &maximum_error)
{
	std::ifstream stream(path, std::ios::binary);
	uint32_t count = 0;

	if (!header(stream, "FASV3G1", count)) { return false; }

	for (uint32_t vector = 0; vector < count; ++vector) {
		float features[SlowEngineeringShadow::FeatureCount] {};
		float expected = 0.f;

		if (!stream.read(reinterpret_cast<char *>(features), sizeof(features)) || !read(stream, expected)) { return false; }

		maximum_error = std::max(maximum_error,
				difference(SlowEngineeringShadow::predict_features(features), expected));
	}

	return maximum_error <= 1e-5f;
}

bool phase_parity(const char *path, uint32_t &mismatches, float &confidence_error)
{
	std::ifstream stream(path, std::ios::binary);
	uint32_t count = 0;

	if (!header(stream, "FASPHG1", count)) { return false; }

	PhaseClassifier classifier;

	for (uint32_t row = 0; row < count; ++row) {
		uint8_t reset = 0, valid = 0, extreme = 0, expected_phase = 0;
		float values[6] {};

		if (!read(stream, reset) || !read(stream, valid) || !read(stream, extreme)
		    || !read(stream, expected_phase)
		    || !stream.read(reinterpret_cast<char *>(values), sizeof(values))) { return false; }

		if (reset) { classifier.reset(); }

		const PhaseResult result = classifier.update(values[0], values[1], values[2], values[3], values[4], valid, extreme);
		mismatches += static_cast<uint8_t>(result.phase) != expected_phase;
		confidence_error = std::max(confidence_error, difference(result.confidence, values[5]));
	}

	return mismatches == 0 && confidence_error <= 1e-6f;
}

bool slow_parity(const char *path, float &state_error, float &innovation_error, float &update_error)
{
	std::ifstream stream(path, std::ios::binary);
	uint32_t count = 0;

	if (!header(stream, "FASSLG1", count)) { return false; }

	BoundedSlowState state;

	for (uint32_t row = 0; row < count; ++row) {
		uint8_t reset = 0;
		float values[5] {};

		if (!read(stream, reset) || !stream.read(reinterpret_cast<char *>(values), sizeof(values))) { return false; }

		if (reset) { state.reset(); }

		const BoundedUpdateResult result = state.update(values[0], values[1], true);
		state_error = std::max(state_error, difference(result.state, values[2]));
		innovation_error = std::max(innovation_error, difference(result.innovation, values[3]));
		update_error = std::max(update_error, difference(result.slew_rate / 50.f, values[4]));
	}

	return state_error <= 5e-6f && innovation_error <= 5e-6f && update_error <= 1e-6f;
}
}

int main(int argc, char *argv[])
{
	if (argc != 5) { return 2; }

	float roll_error = 0.f, base_error = 0.f, temporal_error = 0.f, v3_error = 0.f;
	float confidence_error = 0.f, state_error = 0.f, innovation_error = 0.f, update_error = 0.f;
	uint32_t phase_mismatches = 0;
	const bool v2 = v2_parity(argv[1], roll_error, base_error, temporal_error);
	const bool v3 = v3_model_parity(argv[2], v3_error);
	const bool phase = phase_parity(argv[3], phase_mismatches, confidence_error);
	const bool slow = slow_parity(argv[4], state_error, innovation_error, update_error);
	std::printf("ROLL_V2_PARITY=%s max_abs_error=%.9g\n", v2 && roll_error <= 1e-5f ? "PASS" : "FAIL", roll_error);
	std::printf("PITCH_V2_BASE_PARITY=%s max_abs_error=%.9g\n", v2 && base_error <= 1e-5f ? "PASS" : "FAIL", base_error);
	std::printf("PITCH_V2_TEMPORAL_PARITY=%s max_abs_error=%.9g\n", v2 && temporal_error <= 1e-5f ? "PASS" : "FAIL", temporal_error);
	std::printf("V3_MANEUVER_MODEL_PARITY=%s max_abs_error=%.9g\n", v3 ? "PASS" : "FAIL", v3_error);
	std::printf("PHASE_PARITY=%s mismatches=%u\n", phase && phase_mismatches == 0 ? "PASS" : "FAIL", phase_mismatches);
	std::printf("CONFIDENCE_PARITY=%s max_abs_error=%.9g\n", phase && confidence_error <= 1e-6f ? "PASS" : "FAIL", confidence_error);
	std::printf("SLOW_STATE_PARITY=%s state=%.9g innovation=%.9g update=%.9g\n", slow ? "PASS" : "FAIL",
		   state_error, innovation_error, update_error);
	return v2 && v3 && phase && slow ? 0 : 1;
}
