#pragma once
#include "AdaptiveTailTrimCore.hpp"
#include "AdaptiveTailTrimEstimator.hpp"

class AdaptiveTailTrimManeuverGate
{
public:
	// PROVISIONAL SHADOW constants, radians and seconds. Not flight certified.
	static constexpr float Enter = .174532925f, Exit = .087266463f, Hold = 1.f;
	void reset() { _active = false; _exit_elapsed = 0.f; }
	bool update(float phi, float p, float dt, bool valid);
private:
	bool _active{false};
	float _exit_elapsed{0.f};
};

class CausalTailPitchEnvelope
{
public:
	enum Invalid : uint8_t { Stale = 1, Nonfinite = 2, Mapping = 4, Future = 8, Range = 16 };
	struct Result { float raw{0.f}, roll{0.f}, envelope{0.f}; uint8_t invalid{0}; bool valid{false}; };
	static constexpr uint64_t FreshnessUs = 200000;
	void reset() { _envelope = 0.f; }
	Result update(float left, float right, uint64_t stamp, uint64_t now, float dt, bool mapping);
private:
	float _envelope{0.f};
};

// Values in, diagnostics out. Deliberately cannot access real I, S or actuators.
class AdaptiveTailTrimShadow
{
public:
	struct Config {
		float bmax{.08f}, reserve{.03f}, slew{.005f}, roll_reserve{.15f};
		float tau{5.f}, window{3.f}, std_raw{.025f}, sign_fraction{.9f};
		float entry_window{5.f}; // read-only FLAP_B2B_EWIN time semantics
	};
	struct Inputs {
		uint64_t now{0}, actuator_timestamp{0};
		uint32_t epoch{0};
		float dt{0.f}, g{0.f}, i_actual{0.f}, s_actual{0.f}, imax{0.f};
		float phi_sp{0.f}, p_sp{0.f}, left{0.f}, right{0.f};
		bool enabled{false}, armed{false}, landed{true}, control_valid{false};
		bool setpoint_valid{false}, mapping_valid{false}, safety{false};
		bool mission_eligible{false}; // observation remains available outside Mission
	};
	struct Result {
		AdaptiveTailTrimCore::Result transfer{};
		AdaptiveTailTrimEstimator::Result estimate{};
		CausalTailPitchEnvelope::Result pitch{};
		float g{0.f}, i_actual{0.f}, s_actual{0.f}, t_actual{0.f};
		float b_obs{0.f}, target{0.f}, b_shadow{0.f}, offset{0.f}, i_shadow{0.f}, b_total{0.f};
		float actual_tail_trim_torque{0.f}; // literal zero, diagnostic only
		bool enabled{false}, valid{false}, learning{false}, maneuver{false}, reversal{false}, reset{false};
	};
	void configure(Config config);
	void reset(uint32_t epoch = 0);
	Result update(Inputs in);
private:
	static constexpr float GMin = .01f;
	bool validConfig() const;
	Config _config{};
	AdaptiveTailTrimCore _core{{.08f, .005f, GMin, .15f, .15f, 1.1f}};
	AdaptiveTailTrimEstimator _estimator{};
	AdaptiveTailTrimManeuverGate _maneuver{};
	CausalTailPitchEnvelope _pitch{};
	float _offset{0.f}, _target{0.f};
	uint32_t _epoch{0};
	uint64_t _last_time{0};
	bool _enabled{false}, _reversal{false}, _rearm{false};
	bool _mission_previous{false};
};
