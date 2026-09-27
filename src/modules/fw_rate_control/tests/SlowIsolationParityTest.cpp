#include <gtest/gtest.h>
#include <cfloat>
#include <cstring>
#include <cmath>
#include "../BumplessRollITransfer.hpp"
#define BumplessRollITransfer IsolationSlowReference
#include "IsolationSlowReference.hpp"
#undef BumplessRollITransfer
#include "FastControl.hpp"
#include "FastFFPlus.hpp"
#include "FastActuatorMargin.hpp"
#include <uORB/topics/rate_ctrl_terms.h>
#include <uORB/topics/vehicle_status.h>

using matrix::Vector3f;
// Execute actual source output/status fragments, with deterministic I/O adapters.
// This is not a scheduler, hardware, allocator or full-module integration test.
namespace {
using hrt_abstime = uint64_t;
uint64_t fake_time;
uint64_t clock_now() { return fake_time; }
#define hrt_absolute_time clock_now
struct Param { float value; float get() const { return value; } };
struct Publication { void publish(const rate_ctrl_terms_s &) {} };
struct Compression {
	Vector3f gains{1.f, 1.f, 1.f};
	const Vector3f &getGains() const { return gains; }
	void update(const Vector3f &, float) {} // Same externally swept pre-update gains on both sides.
};
struct RemovedDiagnostic {
	bool ffplus_enabled{}, ffplus_active{}, ffplus_config_valid{}, controller_terms_valid{}, actuator_postcheck_failed{};
	float ffplus_delta{}, base_roll_ff_gain{}, effective_roll_ff_gain{}, p_sp_used{}, p_used{}, p_error_used{};
	float roll_p_term{}, roll_d_term{}, roll_ff_term{}, baseline_roll_output{}, baseline_pitch_output{};
	float augmented_roll_output{}, applied_delta_roll{};
};
template<class Slow> struct Harness {
	using BumplessRollITransfer = Slow;
	RateControl _rate_control;
	Slow _bumpless_roll_i_transfer;
	typename Slow::Inputs in{};
	typename Slow::Result _bumpless_roll_i_result{};
	Compression _gain_compression;
	Publication _rate_ctrl_terms_pub;
	Param _param_fw_rr_ff{.8f}, _param_fw_pr_ff{.3f}, _param_fw_yr_ff{.2f};
	Param _param_ffp_en{0.f}, _param_ffp_delta{.025f}, _param_fast_en{0.f};
	Param _param_fast_k{1.f}, _param_fast_max{.005f}, _param_fast_on{.15f}, _param_fast_off{.18f}, _param_fast_slew{.05f};
	Param _param_fw_acro_yaw_en{1.f}, _param_fw_man_y_sc{1.f}, _param_fw_rll_to_yaw_ff{.12f};
	Param _param_flap_slow_en{1.f}, _param_flap_slow_b0{.195f}, _param_flap_b2b_hr{.5f};
	vehicle_status_s _vehicle_status{};
	struct { bool flag_control_attitude_enabled{true}, flag_control_manual_enabled{false}; } _vcontrol_mode;
	struct { float yaw{}; } _manual_control_setpoint;
	struct { bool reset_integral{}; } _rates_sp;
	struct { float xyz[3]{}; } _vehicle_torque_setpoint;
	bool _landed{}, _roll_i_reset_this_cycle{}, _b2b_total_clipped{};
	float _airspeed_scaling{1.f}, _b2b_g_current{}, _b2b_roll_baseline{};
	FastControl _fast_control; // Immutable reference only; never compiled into production.
	uint64_t _fast_last_decision{};
	Harness(bool enabled, int mode) {
		_param_flap_slow_en.value = enabled;
		_vehicle_status.arming_state = vehicle_status_s::ARMING_STATE_ARMED;
		_vehicle_status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION;
		_rate_control.setPidGains(Vector3f(.05f,.03f,.02f), Vector3f(.1f,.05f,.03f), Vector3f(.01f,.01f,.01f));
		_rate_control.setIntegratorLimit(Vector3f(.2f,.2f,.2f));
		in.enabled = enabled; in.comparator_mode = mode; in.eligible = true; in.config_valid = true; in.rates_enabled = true;
		in.dt = .02f; in.imax_raw = .2f; in.cap_raw = .03f; in.slew_raw_per_s = .02f; in.safety_slew_raw_per_s = .2f;
		in.adapt_enabled = true; in.hold_cap_raw = .1f; in.residual_reserve_raw = .05f;
		in.adapt_tau_s = 5.f; in.adapt_slew_raw_per_s = .01f; in.entry_window_s = 5.f;
		in.gate_window_s = 3.f; in.gate_std_raw = .025f; in.gate_same_sign_fraction = .9f;
		in.headroom_release_ratio = .5f;
		_bumpless_roll_i_transfer.synchronizeReset(_rate_control.rollIntegralResetEpoch());
	}
	bool verify_flap_slow_configuration() const { return true; }
	bool verify_fast_actuator_margin_configuration() const { return true; }
	void resetIntegralAndTransfer() {
		_rate_control.resetIntegral();
		_bumpless_roll_i_transfer.synchronizeReset(_rate_control.rollIntegralResetEpoch());
		_roll_i_reset_this_cycle = true;
	}
	struct Sample { rate_ctrl_status_s status; float raw; float torque; };
	Sample step(int k, bool reference) {
		fake_time = 1000000 + uint64_t(k) * 20000;
		_roll_i_reset_this_cycle = false;
		_airspeed_scaling = .7f + float(k % 13) * .05f;
		_gain_compression.gains = Vector3f(.5f + float(k % 17) * .025f, .8f, .9f);
		const float airspeed_scale_squared = _airspeed_scaling * _airspeed_scaling;
		_b2b_g_current = _gain_compression.getGains()(0) * airspeed_scale_squared;
		in.g_current = _b2b_g_current;
		in.eligible = k % 1000 < 850;
		in.pilot_abort = k % 1000 == 850;
		in.hard_reset = k % 1000 == 990;
		_bumpless_roll_i_result = _bumpless_roll_i_transfer.update(in, _rate_control);
		if (_bumpless_roll_i_result.reset_required) {
			_rate_control.resetIntegral(0);
			_bumpless_roll_i_transfer.synchronizeReset(_rate_control.rollIntegralResetEpoch());
			_roll_i_reset_this_cycle = true;
		}
		float raw = 0.f;
		{
		const float transferred_roll_i_raw = _bumpless_roll_i_transfer.transferredRaw();
		_rate_control.setRollITransferContext(transferred_roll_i_raw,
			_bumpless_roll_i_result.headroom_release_ratio_effective, std::fabs(transferred_roll_i_raw) > FLT_EPSILON);
		_rate_control.setPositiveSaturationFlag(0, k % 11 == 0);
		_rate_control.setNegativeSaturationFlag(0, k % 13 == 0);
		const Vector3f rates(.1f, .03f, -.02f), angular_accel(.01f, -.02f, .03f);
		const Vector3f body_rates_setpoint((k % 1000 < 550 ? .8f : -.5f), .1f, .05f);
		const Vector3f trim(.03f, -.02f, .01f);
		const float dt = in.dt;
		struct { uint64_t timestamp_sample; } angular_velocity{fake_time};
		RemovedDiagnostic fc{};
		FastControl::Decision fast_decision{};
		FastActuatorMargin::Result fast_margin{};
		bool fast_actuator_config_valid = false, apply_fast_roll = false;
		float fast_scale = 0.f, fast_base = 0.f, fast_augmented = 0.f, fast_roll_output = 0.f;
		const bool is_fixed_wing = true, fast_estimator_reset = false, pilot_abort_to_stabilized = in.pilot_abort;
		if (reference) {
#include "BaseOutput.inc"
			raw = angular_acceleration_setpoint(0);
#include "BaseYaw.inc"
		} else {
#include "IsolatedOutput.inc"
			raw = angular_acceleration_setpoint(0);
#include "IsolatedYaw.inc"
		}
		// Explicitly observe reference disabled actuation and FF behavior.
		if (reference) { EXPECT_FLOAT_EQ(fast_decision.final, 0.f); EXPECT_FALSE(apply_fast_roll); }
		(void)fast_augmented;
		}
		if (reference) {
#include "BaseStatus.inc"
			return {rate_ctrl_status, raw, _vehicle_torque_setpoint.xyz[0]};
		} else {
#include "IsolatedStatus.inc"
			return {rate_ctrl_status, raw, _vehicle_torque_setpoint.xyz[0]};
		}
	}
};

void trajectory(bool enabled, int mode) {
	Harness<BumplessRollITransfer> now(enabled, mode);
	Harness<IsolationSlowReference> old(enabled, mode);
	bool saw_transfer = false, saw_lpf = false;
	for (int k = 0; k < 5000; ++k) {
		SCOPED_TRACE(k);
		const auto a = now.step(k, false);
		const auto b = old.step(k, true);
		EXPECT_EQ(a.raw, b.raw);
		EXPECT_EQ(a.torque, b.torque);
		// Both value-initialized messages: compare every status field, including native I/S/T.
		EXPECT_EQ(0, std::memcmp(&a.status, &b.status, sizeof(a.status)));
		saw_transfer |= fabsf(a.status.flap_b2b_transferred_raw) > 1e-5f;
		saw_lpf |= now._bumpless_roll_i_result.comparator_active;
	}
	EXPECT_EQ(saw_transfer, enabled);
	EXPECT_EQ(saw_lpf, enabled && mode == 1);
}
}
TEST(SlowIsolationParity, NativeBaseline) { trajectory(false, 0); }
TEST(SlowIsolationParity, OriginalSlow) { trajectory(true, 0); }
TEST(SlowIsolationParity, PairedLpf) { trajectory(true, 1); }
