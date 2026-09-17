#pragma once
#include <cmath>
#include <cstdint>

// Physical persistent detector. No pitch, actuator or controller access.
class AdaptiveTailTrimEstimator
{
public:
	struct Config { float tau{5.f}, window{3.f}, std_limit{.025f}, sign_fraction{.9f}; };
	struct Result {
		float b_hat{0.f}, std{0.f}, same_sign_fraction{0.f};
		uint16_t sample_count{0};
		bool gate{false}, valid{false};
	};
	void reset();
	void clearEvidence();
	void observePreEntry(float b, float dt, bool trusted);
	void enterMission(float window_s);
	Result update(float b, float dt, bool valid, bool learning, Config cfg);
private:
	float _samples[200]{};
	float _hat{0.f}, _elapsed{0.f}, _reference{0.f};
	uint16_t _head{0}, _count{0};
	bool _initialized{false};
	Result _result{};
	// Separate bounded pre-entry storage; median scratch is never on the work-queue stack.
	float _entry[200]{}, _scratch[200]{};
	float _entry_elapsed{0.f};
	uint16_t _entry_head{0}, _entry_count{0};
};
