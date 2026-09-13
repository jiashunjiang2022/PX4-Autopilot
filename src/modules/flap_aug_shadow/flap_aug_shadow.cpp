#include "flap_aug_shadow.hpp"
#include "shadow_safety.hpp"

#include <matrix/math.hpp>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>

#include <cmath>

using namespace time_literals;

namespace flap_aug_shadow
{
namespace
{
constexpr hrt_abstime FastAge = 60_ms;
constexpr hrt_abstime ManualAge = 250_ms;
constexpr hrt_abstime IntegratorAge = 300_ms;
constexpr hrt_abstime AirspeedAge = 250_ms;
constexpr hrt_abstime BatteryAge = 250_ms;
constexpr hrt_abstime WindAge = 120_ms;
constexpr hrt_abstime StatusAge = 1_s;
constexpr hrt_abstime LongInvalidTime = 2_s;
constexpr float TrackRateAlpha = 0.0392105608f; // 1-exp(-0.02/0.5)
constexpr uint32_t V4ImplementationVersion = 1;
// Diagnostic storage sanity bound only; this is not a control or actuator authorization bound.
// FNV-1a short ID for the frozen V4 semantic specification package.
constexpr uint32_t V4ArtifactHashShort = 0x93d9738au;

float wrap_pi(float value)
{
	return atan2f(sinf(value), cosf(value));
}

bool finite_quaternion(const float (&q)[4])
{
	return std::isfinite(q[0]) && std::isfinite(q[1]) && std::isfinite(q[2]) && std::isfinite(q[3]);
}
}

float CausalLowpassDerivative::update(float input, bool valid)
{
	if (!_initialized) {
		_replacement = valid && std::isfinite(input) ? input : 0.f;
		_initialized = true;
	}

	const float used = valid && std::isfinite(input) ? input : _replacement;
	constexpr float b0 = 0.14532388f;
	constexpr float b1 = 0.29064777f;
	constexpr float b2 = 0.14532388f;
	constexpr float a1 = -0.67102909f;
	constexpr float a2 = 0.25232463f;
	const float filtered = b0 * used + b1 * _x1 + b2 * _x2 - a1 * _y1 - a2 * _y2;
	const float derivative = filtered - _previous_filtered;
	_x2 = _x1;
	_x1 = used;
	_y2 = _y1;
	_y1 = filtered;
	_previous_filtered = filtered;
	_output_valid = valid && std::isfinite(input);
	return derivative * 50.f;
}

void CausalLowpassDerivative::reset()
{
	*this = {};
}

FlapAugShadow::FlapAugShadow() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

bool FlapAugShadow::init()
{
	updateParams();
	ScheduleOnInterval(20_ms);
	return true;
}

bool FlapAugShadow::fresh(hrt_abstime now, hrt_abstime timestamp, hrt_abstime maximum_age) const
{
	return timestamp != 0 && now >= timestamp && now - timestamp <= maximum_age;
}

void FlapAugShadow::update_subscriptions()
{
	_actuator_motors_sub.update(&_actuator_motors);
	_actuator_servos_sub.update(&_actuator_servos);
	_airspeed_sub.update(&_airspeed);
	_battery_sub.update(&_battery);
	_flap_frequency_sub.update(&_flap_frequency);
	_manual_sub.update(&_manual);
	_rate_status_sub.update(&_rate_status);
	_rate_terms_sub.update(&_rate_terms);
	_sensor_accel_sub.update(&_sensor_accel);
	_angular_velocity_sub.update(&_angular_velocity);
	_attitude_sub.update(&_attitude);
	_attitude_setpoint_sub.update(&_attitude_setpoint);
	_land_sub.update(&_land);
	_local_position_sub.update(&_local_position);
	_rates_setpoint_sub.update(&_rates_setpoint);
	_torque_setpoint_sub.update(&_torque_setpoint);
	_vehicle_status_sub.update(&_vehicle_status);
	_wind_sub.update(&_wind);

	if (_parameter_update_sub.updated()) {
		parameter_update_s update{};
		_parameter_update_sub.copy(&update);
		updateParams();
	}
}

bool FlapAugShadow::verify_v4_configuration() const
{
	constexpr float Tolerance = 1e-4f;
	auto near = [](float value, float expected) {
		return std::isfinite(value) && fabsf(value - expected) <= Tolerance;
	};

	// Frozen three-surface identity: left elevon, right elevon, rudder.
	return _param_ca_sv_cs_count.get() == 3
	       && _param_ca_sv_cs0_type.get() == 5
	       && near(_param_ca_sv_cs0_trq_r.get(), -0.55f)
	       && near(_param_ca_sv_cs0_trq_p.get(), 1.f)
	       && near(_param_ca_sv_cs0_trq_y.get(), 0.f)
	       && _param_ca_sv_cs1_type.get() == 6
	       && near(_param_ca_sv_cs1_trq_r.get(), 0.55f)
	       && near(_param_ca_sv_cs1_trq_p.get(), 1.f)
	       && near(_param_ca_sv_cs1_trq_y.get(), 0.f)
	       && _param_ca_sv_cs2_type.get() == 4
	       && near(_param_ca_sv_cs2_trq_r.get(), 0.f)
	       && near(_param_ca_sv_cs2_trq_p.get(), 0.f)
	       && near(_param_ca_sv_cs2_trq_y.get(), 1.f)
	       && _param_pwm_main_func1.get() == 201
	       && _param_pwm_main_func2.get() == 202
	       && _param_pwm_main_func5.get() == 203
	       && _param_pwm_main_rev.get() == 17;
}

void FlapAugShadow::update_legacy_integrator(hrt_abstime now)
{
	const uint64_t bucket = now / 200_ms;

	if (bucket != _legacy_integrator_bucket) {
		_legacy_integrator_bucket = bucket;

		if (fresh(now, _rate_status.timestamp, 250_ms)
		    && std::isfinite(_rate_status.rollspeed_integ)
		    && std::isfinite(_rate_status.pitchspeed_integ)
		    && std::isfinite(_rate_status.yawspeed_integ)) {
			_legacy_integrator[0] = _rate_status.rollspeed_integ;
			_legacy_integrator[1] = _rate_status.pitchspeed_integ;
			_legacy_integrator[2] = _rate_status.yawspeed_integ;
			_legacy_integrator_timestamp = bucket * 200_ms;
		}
	}
}

bool FlapAugShadow::build_base_features(hrt_abstime now,
		float (&features)[FastPredictor::BaseFeatureCount], bool &roll_valid, bool &pitch_valid,
		float &roll, float &pitch, float &yaw, float &roll_sp, float &pitch_sp,
		float &ground_track_rate, bool &rtk_valid)
{
	for (float &value : features) {
		value = NAN;
	}

	const bool attitude_valid = fresh(now, _attitude.timestamp, FastAge) && finite_quaternion(_attitude.q);
	const bool attitude_sp_valid = fresh(now, _attitude_setpoint.timestamp, FastAge)
				       && finite_quaternion(_attitude_setpoint.q_d);
	const bool rate_valid = fresh(now, _angular_velocity.timestamp, FastAge)
				&& std::isfinite(_angular_velocity.xyz[0])
				&& std::isfinite(_angular_velocity.xyz[1])
				&& std::isfinite(_angular_velocity.xyz[2]);
	const bool rate_sp_valid = fresh(now, _rates_setpoint.timestamp, FastAge)
				   && std::isfinite(_rates_setpoint.roll)
				   && std::isfinite(_rates_setpoint.pitch)
				   && std::isfinite(_rates_setpoint.yaw);

	if (attitude_valid) {
		const matrix::Eulerf euler{matrix::Quatf{_attitude.q}};
		roll = euler.phi();
		pitch = euler.theta();
		yaw = euler.psi();
		features[0] = roll;
		features[1] = pitch;
		features[2] = sinf(yaw);
		features[3] = cosf(yaw);
	}

	if (attitude_sp_valid) {
		const matrix::Eulerf euler_sp{matrix::Quatf{_attitude_setpoint.q_d}};
		roll_sp = euler_sp.phi();
		pitch_sp = euler_sp.theta();
		features[7] = roll_sp;
		features[8] = pitch_sp;
	}

	if (rate_valid) {
		features[4] = _angular_velocity.xyz[0];
		features[5] = _angular_velocity.xyz[1];
		features[6] = _angular_velocity.xyz[2];
	}

	if (rate_sp_valid) {
		features[9] = _rates_setpoint.roll;
		features[10] = _rates_setpoint.pitch;
		features[11] = _rates_setpoint.yaw;
	}

	if (attitude_valid && attitude_sp_valid) {
		features[12] = wrap_pi(roll_sp - roll);
		features[13] = pitch_sp - pitch;
	}

	if (rate_valid && rate_sp_valid) {
		features[14] = _rates_setpoint.roll - _angular_velocity.xyz[0];
		features[15] = _rates_setpoint.pitch - _angular_velocity.xyz[1];
		features[16] = _rates_setpoint.yaw - _angular_velocity.xyz[2];
	}

	for (size_t axis = 0; axis < 3; ++axis) {
		features[17 + axis] = _rate_derivative[axis].update(_angular_velocity.xyz[axis], rate_valid);
	}

	const bool accel_valid = fresh(now, _sensor_accel.timestamp, FastAge)
				 && std::isfinite(_sensor_accel.x) && std::isfinite(_sensor_accel.y)
				 && std::isfinite(_sensor_accel.z);
	features[20] = accel_valid ? _sensor_accel.x : NAN;
	features[21] = accel_valid ? _sensor_accel.y : NAN;
	features[22] = accel_valid ? _sensor_accel.z : NAN;

	update_legacy_integrator(now);
	const float roll_imax = _param_fw_rr_imax.get();
	const float pitch_imax = _param_fw_pr_imax.get();
	const bool integrator_valid = fresh(now, _legacy_integrator_timestamp, IntegratorAge)
				      && std::isfinite(roll_imax) && roll_imax > 1e-6f
				      && std::isfinite(pitch_imax) && pitch_imax > 1e-6f;

	if (integrator_valid) {
		features[23] = _legacy_integrator[0];
		features[24] = _legacy_integrator[1];
		features[25] = _legacy_integrator[2];
		features[26] = fabsf(_legacy_integrator[0]) / roll_imax;
		features[27] = fabsf(_legacy_integrator[1]) / pitch_imax;
	}

	const bool servos_valid = fresh(now, _actuator_servos.timestamp, FastAge)
				  && std::isfinite(_actuator_servos.control[0])
				  && std::isfinite(_actuator_servos.control[1])
				  && std::isfinite(_actuator_servos.control[2]);

	if (servos_valid) {
		features[28] = _actuator_servos.control[0];
		features[29] = _actuator_servos.control[1];
		features[30] = _actuator_servos.control[2];
		features[31] = (-features[28] + features[29]) * 0.5f;
		features[32] = (features[28] + features[29]) * 0.5f;
	}

	const bool flap_valid = fresh(now, _flap_frequency.timestamp, FastAge)
				&& std::isfinite(_flap_frequency.frequency_hz);
	features[33] = flap_valid ? _flap_frequency.frequency_hz : NAN;
	features[34] = _flap_derivative.update(_flap_frequency.frequency_hz, flap_valid);

	const bool propulsion_valid = fresh(now, _actuator_motors.timestamp, FastAge)
				      && std::isfinite(_actuator_motors.control[0]);
	features[35] = propulsion_valid ? _actuator_motors.control[0] : NAN;
	features[36] = _propulsion_derivative.update(_actuator_motors.control[0], propulsion_valid);
	features[37] = propulsion_valid ? _actuator_motors.control[0] * _actuator_motors.control[0] : NAN;

	const bool airspeed_valid = fresh(now, _airspeed.timestamp, AirspeedAge)
				    && std::isfinite(_airspeed.calibrated_airspeed_m_s);
	features[38] = airspeed_valid ? _airspeed.calibrated_airspeed_m_s : NAN;

	const bool local_valid = fresh(now, _local_position.timestamp, FastAge)
				 && std::isfinite(_local_position.vx) && std::isfinite(_local_position.vy)
				 && std::isfinite(_local_position.vz) && std::isfinite(_local_position.z)
				 && std::isfinite(_local_position.ref_alt);

	if (local_valid) {
		features[39] = hypotf(_local_position.vx, _local_position.vy);
		features[40] = -_local_position.vz;
		features[41] = _local_position.ref_alt - _local_position.z;
	}

	const bool battery_valid = fresh(now, _battery.timestamp, BatteryAge)
				   && std::isfinite(_battery.voltage_v) && std::isfinite(_battery.current_a);
	features[42] = battery_valid ? _battery.voltage_v : NAN;
	features[43] = battery_valid ? _battery.current_a : NAN;
	const bool wind_valid = fresh(now, _wind.timestamp, WindAge)
				&& std::isfinite(_wind.windspeed_north) && std::isfinite(_wind.windspeed_east);

	if (wind_valid) {
		features[44] = _wind.windspeed_north;
		features[45] = _wind.windspeed_east;
		features[46] = hypotf(_wind.windspeed_north, _wind.windspeed_east);
	}

	rtk_valid = local_valid && _local_position.xy_valid && _local_position.v_xy_valid
		    && _local_position.v_z_valid && _local_position.heading_good_for_control;
	ground_track_rate = NAN;

	if (rtk_valid) {
		const float track = atan2f(_local_position.vy, _local_position.vx);

		if (_previous_track_valid) {
			const float delta = wrap_pi(track - _previous_track);
			_unwrapped_track += delta;
			const float raw_rate = delta / 0.02f;

			if (_track_rate_initialized) {
				_track_rate_state += TrackRateAlpha * (raw_rate - _track_rate_state);

			} else {
				_track_rate_state = raw_rate;
				_track_rate_initialized = true;
			}
		}

		_previous_track = track;
		_previous_track_valid = true;
		ground_track_rate = _track_rate_initialized ? _track_rate_state : NAN;

	} else {
		_previous_track_valid = false;
	}

	roll_valid = attitude_valid && attitude_sp_valid && rate_valid && rate_sp_valid
		     && integrator_valid && servos_valid;
	pitch_valid = roll_valid && accel_valid && flap_valid && propulsion_valid && airspeed_valid
		      && local_valid && battery_valid && wind_valid
		      && _rate_derivative[0].valid() && _rate_derivative[1].valid() && _rate_derivative[2].valid()
		      && _flap_derivative.valid() && _propulsion_derivative.valid();
	return roll_valid || pitch_valid;
}

void FlapAugShadow::reset_estimators()
{
	_fast.reset();
	_phase.reset();
	_slow.reset();
	for (CausalLowpassDerivative &filter : _rate_derivative) {
		filter.reset();
	}
	_flap_derivative.reset();
	_propulsion_derivative.reset();
	_legacy_integrator_timestamp = 0;
	_legacy_integrator_bucket = UINT64_MAX;
	_previous_track_valid = false;
	_track_rate_initialized = false;
	_track_rate_state = 0.f;
	_invalid_since = 0;
	_v4_state.reset();
}

void FlapAugShadow::Run()
{
	const hrt_abstime total_start = hrt_absolute_time();
	update_subscriptions();
	const hrt_abstime now = hrt_absolute_time();
	float base[FastPredictor::BaseFeatureCount] {};
	float roll = NAN;
	float pitch = NAN;
	float yaw = NAN;
	float roll_sp = NAN;
	float pitch_sp = NAN;
	float ground_track_rate = NAN;
	bool roll_input_valid = false;
	bool pitch_input_valid = false;
	bool rtk_valid = false;
	build_base_features(now, base, roll_input_valid, pitch_input_valid, roll, pitch, yaw,
			    roll_sp, pitch_sp, ground_track_rate, rtk_valid);

	const hrt_abstime fast_start = hrt_absolute_time();
	const FastPredictor::Result fast = _fast.update(base, roll_input_valid, pitch_input_valid);
	const uint32_t fast_compute_us = static_cast<uint32_t>(hrt_absolute_time() - fast_start);
	const hrt_abstime slow_start = hrt_absolute_time();

	const bool status_valid = fresh(now, _vehicle_status.timestamp, StatusAge);
	const bool land_valid = fresh(now, _land.timestamp, StatusAge);
	const bool armed = status_valid && _vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
	const bool failsafe = !status_valid || _vehicle_status.failsafe;
	const bool airborne = land_valid && !_land.landed;
	const bool supported_mode = status_valid && (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_STAB
				    || _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION);
	const bool extreme = std::isfinite(roll) && std::isfinite(_angular_velocity.xyz[0])
			     && (fabsf(roll) > 0.785398163f || fabsf(_angular_velocity.xyz[0]) > 2.09439510f);
	const bool manual_valid = fresh(now, _manual.timestamp, ManualAge) && _manual.valid
				  && std::isfinite(_manual.roll) && std::isfinite(_manual.pitch);
	const bool terms_valid = fresh(now, _rate_terms.timestamp, FastAge)
				 && std::isfinite(_rate_terms.p_term[0]) && std::isfinite(_rate_terms.i_term[0])
				 && std::isfinite(_rate_terms.ff_term[0]) && std::isfinite(_rate_terms.output[0]);
	const bool roll_i_diagnostic_valid = fresh(now, _rate_status.timestamp, IntegratorAge)
					     && std::isfinite(_rate_status.rollspeed_error)
					     && std::isfinite(_rate_status.rollspeed_integ_delta_raw)
					     && std::isfinite(_rate_status.rollspeed_integ_delta_pre_imax)
					     && std::isfinite(_rate_status.rollspeed_integ_shadow_no_imax)
					     && std::isfinite(_rate_status.rollspeed_integ_raw_drive_accum);
	const bool slow_status_valid = fresh(now, _rate_status.timestamp, IntegratorAge)
				       && std::isfinite(_rate_status.flap_slow_applied_torque);
	const float actual_slow_injection = slow_status_valid ? _rate_status.flap_slow_applied_torque : 0.f;
	const bool phase_input_valid = armed && airborne && supported_mode && manual_valid
				       && fresh(now, _attitude.timestamp, FastAge)
				       && fresh(now, _attitude_setpoint.timestamp, FastAge)
				       && fresh(now, _angular_velocity.timestamp, FastAge)
				       && fresh(now, _rates_setpoint.timestamp, FastAge)
				       && std::isfinite(roll) && std::isfinite(roll_sp)
				       && std::isfinite(_angular_velocity.xyz[0])
				       && std::isfinite(_rates_setpoint.roll)
				       && rtk_valid && std::isfinite(ground_track_rate);
	const PhaseResult phase = _phase.update(_manual.roll, roll_sp, _rates_setpoint.roll,
					      _angular_velocity.xyz[0], ground_track_rate,
					      phase_input_valid, extreme);

	float v3_current[SlowEngineeringShadow::CurrentFeatureCount] {
		_manual.roll, _manual.pitch, roll_sp, pitch_sp, _rates_setpoint.roll, _rates_setpoint.pitch,
		roll, pitch, sinf(yaw), cosf(yaw), _angular_velocity.xyz[0], _angular_velocity.xyz[1],
		_angular_velocity.xyz[2], base[32], _rate_terms.p_term[0], _rate_terms.i_term[0],
		_rate_terms.ff_term[0], _rate_terms.output[0], base[38], base[33]
	};
	// The explicit checks below mirror the 20 current V3 feature sources.
	bool v3_model_input_valid = manual_valid && terms_valid;

	for (const float value : v3_current) {
		v3_model_input_valid = v3_model_input_valid && std::isfinite(value);
	}

	const float tail_margin = std::isfinite(base[28]) && std::isfinite(base[29]) ?
				  1.f - fmaxf(fabsf(base[28]), fabsf(base[29])) : NAN;
	const bool reset_now = !armed || failsafe || extreme;

	if (reset_now) {
		reset_estimators();

	} else if (!(v3_model_input_valid && phase_input_valid)) {
		if (_invalid_since == 0) {
			_invalid_since = now;

		} else if (now - _invalid_since > LongInvalidTime) {
			_slow.reset();
		}

	} else {
		_invalid_since = 0;
	}

	const bool allow_slow_update = !reset_now && supported_mode && airborne && phase_input_valid
				       && std::isfinite(tail_margin) && tail_margin > 0.2f;
	const SlowEngineeringShadow::Result slow = _slow.update(v3_current,
		v3_model_input_valid && armed && !failsafe, base[31], phase.confidence, allow_slow_update);
	const bool slow_valid = slow.model_valid && allow_slow_update
				&& phase.phase != ManeuverPhase::ExtremeOrInvalid;
	const uint32_t slow_compute_us = static_cast<uint32_t>(hrt_absolute_time() - slow_start);

	flap_aug_shadow_s message{};
	message.timestamp = now;
	message.implementation_version = 3;
	message.v2_hash_short = 0x3a71c671u;
	message.v3_hash_short = 0xf46195fau;
	message.input_valid = roll_input_valid && v3_model_input_valid && phase_input_valid;
	message.fast_roll_valid = fast.roll_valid;
	message.fast_pitch_valid = fast.pitch_valid;
	message.v3_model_available = true;
	message.history_ready = slow.history_ready;
	message.maneuver_prediction_valid = slow.model_valid;
	message.slow_residual_valid = slow.residual_valid;
	message.slow_valid = slow_valid;
	message.rtk_valid = rtk_valid;
	message.flight_mode = status_valid ? _vehicle_status.nav_state : UINT8_MAX;
	message.maneuver_phase = static_cast<uint8_t>(phase.phase);
	message.phase_transition = phase.transition;
	message.maneuver_score = phase.score;
	message.slow_confidence = phase.confidence;
	message.tail_roll = std::isfinite(base[31]) ? base[31] : 0.f;
	message.tail_pitch = std::isfinite(base[32]) ? base[32] : 0.f;
	message.roll_i = terms_valid ? _rate_terms.i_term[0] : 0.f;
	message.roll_i_norm = std::isfinite(base[26]) ? base[26] : 0.f;
	message.tail_margin = std::isfinite(tail_margin) ? tail_margin : 0.f;
	message.roll_i_diagnostic_valid = roll_i_diagnostic_valid;
	message.roll_i_update_enabled = roll_i_diagnostic_valid && _rate_status.rollspeed_integ_update_enabled;
	message.roll_rate_error = roll_i_diagnostic_valid ? _rate_status.rollspeed_error : 0.f;
	message.roll_i_delta_raw = roll_i_diagnostic_valid ? _rate_status.rollspeed_integ_delta_raw : 0.f;
	message.roll_i_delta_pre_imax = roll_i_diagnostic_valid ? _rate_status.rollspeed_integ_delta_pre_imax : 0.f;
	message.roll_i_shadow_no_imax = roll_i_diagnostic_valid ? _rate_status.rollspeed_integ_shadow_no_imax : 0.f;
	message.roll_i_raw_drive_accum = roll_i_diagnostic_valid ? _rate_status.rollspeed_integ_raw_drive_accum : 0.f;
	message.fast_roll_pred = fast.roll;
	message.fast_pitch_pred = fast.pitch;
	message.maneuver_hat = slow.maneuver_hat;
	message.slow_residual = slow.residual;
	message.slow_hat = slow.slow_hat;
	message.slow_innovation = slow.innovation;
	message.slow_slew_rate = slow.slew_rate;
	message.slow_update_enabled = slow.update_enabled;
	message.requested_slow_aug = 0.f;
	message.requested_fast_roll_aug = 0.f;
	message.requested_fast_pitch_aug = 0.f;
	message.projected_slow_aug = 0.f;
	message.projected_fast_roll_aug = 0.f;
	message.projected_fast_pitch_aug = 0.f;
	message.projection_active = false;
		message.actual_slow_injection = actual_slow_injection;
	message.actual_fast_roll_injection = ActualInjection::FastRoll;
	message.actual_fast_pitch_injection = ActualInjection::FastPitch;
	message.fast_compute_us = fast_compute_us;
	message.slow_compute_us = slow_compute_us;
	message.total_compute_us = static_cast<uint32_t>(hrt_absolute_time() - total_start);
	_shadow_pub.publish(message);

	// V4 is intentionally a separate diagnostic state. Its update law is not frozen in the
	// architecture package, so delta_b remains zero; configuration validity is runtime guarded.
	const V4ShadowResult v4 = _v4_state.update(_param_flap_v4_enable.get(), _param_flap_v4_b_prior.get(),
									   verify_v4_configuration(), base[28], base[29], base[30],
									   _torque_setpoint.xyz[0], slow.model_valid, slow.maneuver_hat,
										   actual_slow_injection, ActualInjection::FastRoll,
									   ActualInjection::FastPitch);

	flap_aug_v4_shadow_s v4_message{};
	v4_message.timestamp = now;
	v4_message.v4_implementation_version = V4ImplementationVersion;
	v4_message.v4_artifact_hash_short = V4ArtifactHashShort;
	v4_message.v4_enabled = v4.enabled;
	v4_message.config_mapping_valid = v4.config_mapping_valid;
	v4_message.tail_roll_signal_valid = v4.tail_roll_signal_valid;
	v4_message.b_prior = v4.b_prior;
	v4_message.b_prior_valid = v4.b_prior_valid;
	v4_message.delta_b_shadow = v4.delta_b_shadow;
	v4_message.u_trim_candidate_shadow = v4.u_trim_candidate_shadow;
	// This is the realized PRE_REVERSAL differential-tail coordinate verified for the aircraft.
	v4_message.u_controller_eq = v4.u_controller_eq;
	v4_message.u_controller_eq_valid = v4.u_controller_eq_valid;
	v4_message.tail_roll_realized = v4.tail_roll_realized;
	v4_message.tail_pitch_realized = v4.tail_pitch_realized;
	v4_message.tail_yaw_realized = v4.tail_yaw_realized;
	v4_message.roll_torque_equiv_prealloc = v4.roll_torque_equiv_prealloc;
	v4_message.allocation_consistency_error = v4.allocation_consistency_error;
	v4_message.zero_aug_mapping_valid = v4.zero_aug_mapping_valid;
	v4_message.nonzero_aug_mapping_valid = v4.nonzero_aug_mapping_valid;
	v4_message.u_aug_eq_shadow = v4.u_aug_eq_shadow;
	v4_message.maneuver_hat = v4.maneuver_hat;
	v4_message.maneuver_hat_valid = v4.maneuver_hat_valid;
	v4_message.r_total_proxy_shadow = v4.r_total_proxy_shadow;
	v4_message.r_total_proxy_valid = v4.r_total_proxy_valid;
	v4_message.innovation_shadow = v4.innovation_shadow;
	v4_message.total_mapping_valid = v4.total_mapping_valid;
	v4_message.delta_update_implemented = v4.delta_update_implemented;
	v4_message.maneuver_phase = static_cast<uint8_t>(phase.phase);
	v4_message.slow_confidence = std::isfinite(phase.confidence) ? phase.confidence : 0.f;
	v4_message.history_ready = slow.history_ready;
	v4_message.v3_model_valid = slow.model_valid;
	v4_message.residual_valid = slow.residual_valid;
	v4_message.phase_valid = phase_input_valid;
	v4_message.actual_slow_injection = actual_slow_injection;
	v4_message.actual_fast_roll_injection = ActualInjection::FastRoll;
	v4_message.actual_fast_pitch_injection = ActualInjection::FastPitch;
	_v4_shadow_pub.publish(v4_message);
}

int FlapAugShadow::task_spawn(int argc, char *argv[])
{
	FlapAugShadow *instance = new FlapAugShadow();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;

	if (instance->init()) {
		return PX4_OK;
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;
	return PX4_ERROR;
}

int FlapAugShadow::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FlapAugShadow::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Read-only 50 Hz Frozen V2 and V3 engineering Shadow diagnostics.
It has no actuator publication or injection path.
)DESCR_STR");
	PRINT_MODULE_USAGE_NAME("flap_aug_shadow", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

} // namespace flap_aug_shadow

extern "C" __EXPORT int flap_aug_shadow_main(int argc, char *argv[])
{
	return flap_aug_shadow::FlapAugShadow::main(argc, argv);
}
