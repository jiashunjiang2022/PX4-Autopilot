/****************************************************************************
 * V3 engineering-only phase/confidence/model Shadow. No actuator interfaces.
 ****************************************************************************/

#pragma once

#include <cstddef>
#include <cstdint>

namespace flap_aug_shadow
{

enum class ManeuverPhase : uint8_t {
	StraightOrLowManeuver = 0,
	TurnEntry = 1,
	SteadyTurn = 2,
	TurnExit = 3,
	ExtremeOrInvalid = 4,
};

struct PhaseResult {
	ManeuverPhase phase{ManeuverPhase::ExtremeOrInvalid};
	float score{0.f};
	float signed_score{0.f};
	float confidence{0.f};
	bool transition{false};
};

struct BoundedUpdateResult {
	float state{0.f};
	float innovation{0.f};
	float slew_rate{0.f};
	bool enabled{false};
};

class BoundedSlowState
{
public:
	BoundedUpdateResult update(float residual, float confidence, bool allow_update);
	void reset() { _state = 0.f; }
	float state() const { return _state; }

private:
	float _state{0.f};
};

class PhaseClassifier
{
public:
	PhaseResult update(float manual_roll, float roll_sp, float p_sp, float p,
			   float ground_track_rate, bool valid, bool extreme);
	void reset();

private:
	ManeuverPhase _state{ManeuverPhase::StraightOrLowManeuver};
	uint16_t _dwell{0};
	uint16_t _entry_count{0};
	uint16_t _exit_count{0};
};

class SlowEngineeringShadow
{
public:
	static constexpr size_t CurrentFeatureCount = 20;
	static constexpr size_t FeatureCount = 64;
	static constexpr size_t HistorySamples = 51;

	struct Result {
		float maneuver_hat{0.f};
		float residual{0.f};
		float slow_hat{0.f};
		float innovation{0.f};
		float slew_rate{0.f};
		bool model_valid{false};
		bool update_enabled{false};
	};

	Result update(const float (&current)[CurrentFeatureCount], bool model_input_valid,
		      float tail_roll, float confidence, bool allow_update);
	void reset();
	float state() const { return _bounded_state.state(); }

	static float predict_features(const float (&features)[FeatureCount]);

private:
	const float *history_at_offset(size_t offset_from_newest) const;
	void build_features(float (&features)[FeatureCount]) const;

	float _history[HistorySamples][CurrentFeatureCount]{};
	bool _history_valid[HistorySamples]{};
	size_t _next{0};
	size_t _count{0};
	BoundedSlowState _bounded_state{};
	Result _last{};
};

} // namespace flap_aug_shadow
