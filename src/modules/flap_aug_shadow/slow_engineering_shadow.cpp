#include "slow_engineering_shadow.hpp"

#include "generated/v3_slow_engineering_model.hpp"

#include <cmath>
#include <cstring>

namespace flap_aug_shadow
{
namespace
{
constexpr float RadToDeg = 57.29577951308232f;
constexpr float SlowAlpha = 0.00199800133f; // 1-exp(-0.02/10)
constexpr float InnovationClip = 0.20f;
constexpr float OutputClip = 0.60f;
constexpr uint16_t EntryConfirmSamples = 10;
constexpr uint16_t ExitConfirmSamples = 15;
constexpr uint16_t MinimumDwellSamples = 25;
constexpr uint16_t MaximumTransitionSamples = 75;

float constrain(float value, float lower, float upper)
{
	return fminf(fmaxf(value, lower), upper);
}

float confidence_for_phase(ManeuverPhase phase)
{
	switch (phase) {
	case ManeuverPhase::StraightOrLowManeuver: return 1.f;
	case ManeuverPhase::SteadyTurn: return 0.4f;
	default: return 0.f;
	}
}
}

static_assert(generated::v3_slow::FeatureCount == SlowEngineeringShadow::FeatureCount);

BoundedUpdateResult BoundedSlowState::update(float residual, float confidence, bool allow_update)
{
	BoundedUpdateResult result{};
	const float raw_innovation = residual - _state;
	result.innovation = std::isfinite(raw_innovation) ? raw_innovation : 0.f;

	if (allow_update && std::isfinite(residual) && std::isfinite(confidence) && confidence > 0.f) {
		result.innovation = constrain(raw_innovation, -InnovationClip, InnovationClip);
		const float delta = SlowAlpha * constrain(confidence, 0.f, 1.f) * result.innovation;
		const float next = constrain(_state + delta, -OutputClip, OutputClip);
		result.slew_rate = (next - _state) * 50.f;
		_state = next;
		result.enabled = true;
	}

	result.state = _state;
	return result;
}

void PhaseClassifier::reset()
{
	_state = ManeuverPhase::StraightOrLowManeuver;
	_dwell = 0;
	_entry_count = 0;
	_exit_count = 0;
}

PhaseResult PhaseClassifier::update(float manual_roll, float roll_sp, float p_sp, float p,
		float ground_track_rate, bool valid, bool extreme)
{
	PhaseResult result{};
	const ManeuverPhase previous = _state;
	const bool finite = std::isfinite(manual_roll) && std::isfinite(roll_sp) && std::isfinite(p_sp)
			    && std::isfinite(p) && std::isfinite(ground_track_rate);

	if (valid && finite) {
		const float components[5] {manual_roll / 0.15f, roll_sp * RadToDeg / 8.f,
			p_sp * RadToDeg / 20.f, p * RadToDeg / 30.f, ground_track_rate / 0.15f};

		for (const float value : components) {
			result.score += fabsf(value) / 5.f;
			result.signed_score += value / 5.f;
		}
	}

	if (!valid || !finite || extreme) {
		_state = ManeuverPhase::ExtremeOrInvalid;
		result.score = 0.f;
		result.signed_score = 0.f;
		_dwell = _entry_count = _exit_count = 0;

	} else {
		if (_state == ManeuverPhase::ExtremeOrInvalid) {
			_state = ManeuverPhase::StraightOrLowManeuver;
		}

		switch (_state) {
		case ManeuverPhase::StraightOrLowManeuver:
			_entry_count = result.score >= 1.f ? _entry_count + 1 : 0;

			if (_entry_count >= EntryConfirmSamples) {
				_state = ManeuverPhase::TurnEntry;
				_dwell = _entry_count = 0;
			}

			break;

		case ManeuverPhase::TurnEntry:
			++_dwell;
			_exit_count = result.score <= 0.55f ? _exit_count + 1 : 0;

			if (_dwell >= MinimumDwellSamples && result.score >= 0.75f) {
				_state = ManeuverPhase::SteadyTurn;
				_dwell = _exit_count = 0;

			} else if (_exit_count >= ExitConfirmSamples || _dwell >= MaximumTransitionSamples) {
				_state = ManeuverPhase::TurnExit;
				_dwell = _exit_count = 0;
			}

			break;

		case ManeuverPhase::SteadyTurn:
			++_dwell;
			_exit_count = result.score <= 0.55f ? _exit_count + 1 : 0;

			if (_exit_count >= ExitConfirmSamples) {
				_state = ManeuverPhase::TurnExit;
				_dwell = _exit_count = 0;
			}

			break;

		case ManeuverPhase::TurnExit:
			++_dwell;
			_entry_count = result.score >= 1.f ? _entry_count + 1 : 0;

			if (_dwell >= MinimumDwellSamples && result.score <= 0.55f) {
				_state = ManeuverPhase::StraightOrLowManeuver;
				_dwell = _entry_count = 0;

			} else if (_entry_count >= EntryConfirmSamples) {
				_state = ManeuverPhase::TurnEntry;
				_dwell = _entry_count = 0;

			} else if (_dwell >= MaximumTransitionSamples) {
				_state = result.score >= 0.75f ? ManeuverPhase::SteadyTurn : ManeuverPhase::StraightOrLowManeuver;
				_dwell = 0;
			}

			break;

		case ManeuverPhase::ExtremeOrInvalid:
			break;
		}
	}

	result.phase = _state;
	result.confidence = confidence_for_phase(_state);
	result.transition = previous != _state;
	return result;
}

void SlowEngineeringShadow::reset()
{
	std::memset(_history, 0, sizeof(_history));
	std::memset(_history_valid, 0, sizeof(_history_valid));
	_next = 0;
	_count = 0;
	_bounded_state.reset();
	_last = {};
}

const float *SlowEngineeringShadow::history_at_offset(size_t offset_from_newest) const
{
	const size_t newest = (_next + HistorySamples - 1) % HistorySamples;
	return _history[(newest + HistorySamples - offset_from_newest) % HistorySamples];
}

void SlowEngineeringShadow::build_features(float (&features)[FeatureCount]) const
{
	const float *current = history_at_offset(0);
	std::memcpy(features, current, CurrentFeatureCount * sizeof(float));
	size_t index = CurrentFeatureCount;
	constexpr size_t control_features[3] {0, 2, 4};
	constexpr size_t body_features[2] {10, 6};
	constexpr size_t flight_features[2] {18, 19};
	constexpr size_t delta_features[5] {0, 2, 4, 10, 6};
	constexpr size_t delta_offsets[3] {5, 10, 20};
	constexpr size_t control_offsets[5] {5, 10, 20, 30, 50};
	constexpr size_t flight_offsets[2] {25, 50};

	for (const size_t source : control_features) {
		for (const size_t offset : control_offsets) {
			features[index++] = history_at_offset(offset)[source];
		}
	}

	for (const size_t source : body_features) {
		for (const size_t offset : control_offsets) {
			features[index++] = history_at_offset(offset)[source];
		}
	}

	for (const size_t source : flight_features) {
		for (const size_t offset : flight_offsets) {
			features[index++] = history_at_offset(offset)[source];
		}
	}

	for (const size_t source : delta_features) {
		for (const size_t offset : delta_offsets) {
			features[index++] = current[source] - history_at_offset(offset)[source];
		}
	}
}

float SlowEngineeringShadow::predict_features(const float (&features)[FeatureCount])
{
	float result = generated::v3_slow::Intercept;

	for (size_t index = 0; index < FeatureCount; ++index) {
		const float standardized = std::isfinite(features[index]) ?
			(features[index] - generated::v3_slow::Mean[index]) / generated::v3_slow::Scale[index] : 0.f;
		result += standardized * generated::v3_slow::Coefficient[index];
	}

	return result;
}

SlowEngineeringShadow::Result SlowEngineeringShadow::update(const float (&current)[CurrentFeatureCount],
		bool model_input_valid, float tail_roll, float confidence, bool allow_update)
{
	std::memcpy(_history[_next], current, sizeof(current));
	_history_valid[_next] = model_input_valid;
	_next = (_next + 1) % HistorySamples;
	_count = _count < HistorySamples ? _count + 1 : HistorySamples;
	bool history_valid = _count == HistorySamples;

	for (size_t index = 0; index < HistorySamples && history_valid; ++index) {
		history_valid = _history_valid[index];
	}

	_last.model_valid = history_valid && std::isfinite(tail_roll);
	_last.update_enabled = false;
	_last.slew_rate = 0.f;

	if (_last.model_valid) {
		float features[FeatureCount] {};
		build_features(features);
		_last.maneuver_hat = predict_features(features);
		_last.residual = tail_roll - _last.maneuver_hat;
		const BoundedUpdateResult bounded = _bounded_state.update(_last.residual, confidence, allow_update);
		_last.innovation = bounded.innovation;
		_last.slew_rate = bounded.slew_rate;
		_last.update_enabled = bounded.enabled;
	}

	_last.slow_hat = _bounded_state.state();
	return _last;
}

} // namespace flap_aug_shadow
