/****************************************************************************
 *
 *   Copyright (c) 2013-2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "FixedwingRateControl.hpp"

using namespace time_literals;
using namespace matrix;

using math::constrain;
using math::interpolate;
using math::radians;

FixedwingRateControl::FixedwingRateControl(bool vtol) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_actuator_controls_status_pub(vtol ? ORB_ID(actuator_controls_status_1) : ORB_ID(actuator_controls_status_0)),
	_vehicle_thrust_setpoint_pub(vtol ? ORB_ID(vehicle_thrust_setpoint_virtual_fw) : ORB_ID(vehicle_thrust_setpoint)),
	_vehicle_torque_setpoint_pub(vtol ? ORB_ID(vehicle_torque_setpoint_virtual_fw) : ORB_ID(vehicle_torque_setpoint)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	_handle_param_vt_fw_difthr_en = param_find("VT_FW_DIFTHR_EN");

	/* fetch initial parameter values */
	parameters_update();

	_rate_ctrl_status_pub.advertise();
	// PITCH_DIAG_BEGIN
	_pitch_diag_inner_pub.advertise();
	// PITCH_DIAG_END
}

FixedwingRateControl::~FixedwingRateControl()
{
	perf_free(_loop_perf);
}

bool
FixedwingRateControl::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

int
FixedwingRateControl::parameters_update()
{
	const Vector3f rate_p = Vector3f(_param_fw_rr_p.get(), _param_fw_pr_p.get(), _param_fw_yr_p.get());
	const Vector3f rate_i = Vector3f(_param_fw_rr_i.get(), _param_fw_pr_i.get(), _param_fw_yr_i.get());
	const Vector3f rate_d = Vector3f(_param_fw_rr_d.get(), _param_fw_pr_d.get(), _param_fw_yr_d.get());

	_rate_control.setPidGains(rate_p, rate_i, rate_d);

	_rate_control.setIntegratorLimit(
		Vector3f(_param_fw_rr_imax.get(), _param_fw_pr_imax.get(), _param_fw_yr_imax.get()));

	if (_handle_param_vt_fw_difthr_en != PARAM_INVALID) {
		param_get(_handle_param_vt_fw_difthr_en, &_param_vt_fw_difthr_en);
	}


	return PX4_OK;
}

void
FixedwingRateControl::vehicle_manual_poll()
{
	if (_vcontrol_mode.flag_control_manual_enabled && _in_fw_or_transition_wo_tailsitter_transition) {

		// Always copy the new manual setpoint, even if it wasn't updated, to fill the actuators with valid values
		if (_manual_control_setpoint_sub.copy(&_manual_control_setpoint)) {

			if (_vcontrol_mode.flag_control_rates_enabled &&
			    !_vcontrol_mode.flag_control_attitude_enabled) {

				// RATE mode we need to generate the rate setpoint from manual user inputs

				if (_vehicle_status.is_vtol_tailsitter && _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {
					// the rate_sp must always be published in body (hover) frame
					_rates_sp.roll = _manual_control_setpoint.yaw * radians(_param_fw_acro_z_max.get());
					_rates_sp.yaw = -_manual_control_setpoint.roll * radians(_param_fw_acro_x_max.get());

				} else {
					_rates_sp.roll = _manual_control_setpoint.roll * radians(_param_fw_acro_x_max.get());
					_rates_sp.yaw = _manual_control_setpoint.yaw * radians(_param_fw_acro_z_max.get());
				}

				_rates_sp.timestamp = hrt_absolute_time();
				_rates_sp.pitch = -_manual_control_setpoint.pitch * radians(_param_fw_acro_y_max.get());
				_rates_sp.thrust_body[0] = (_manual_control_setpoint.throttle + 1.f) * .5f;

				_rate_sp_pub.publish(_rates_sp);

			} else {
				// Manual/direct control, filled in FW-frame. Note that setpoints will get transformed to body frame prior publishing.
				const float airspeed_scaling_sq = _airspeed_scaling * _airspeed_scaling;

				_vehicle_torque_setpoint.xyz[0] = math::constrain(_manual_control_setpoint.roll * _param_fw_man_r_sc.get() +
								  _param_trim_roll.get() * airspeed_scaling_sq, -1.f, 1.f);
				_vehicle_torque_setpoint.xyz[1] = math::constrain(-_manual_control_setpoint.pitch * _param_fw_man_p_sc.get() +
								  _param_trim_pitch.get() * airspeed_scaling_sq, -1.f, 1.f);
				_vehicle_torque_setpoint.xyz[2] = math::constrain(_manual_control_setpoint.yaw * _param_fw_man_y_sc.get() +
								  _param_trim_yaw.get() * airspeed_scaling_sq, -1.f, 1.f);

				_vehicle_thrust_setpoint.xyz[0] = math::constrain((_manual_control_setpoint.throttle + 1.f) * .5f, 0.f, 1.f);
			}
		}
	}
}

void
FixedwingRateControl::vehicle_land_detected_poll()
{
	if (_vehicle_land_detected_sub.updated()) {
		vehicle_land_detected_s vehicle_land_detected {};

		if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
			_landed = vehicle_land_detected.landed;
		}
	}
}

float FixedwingRateControl::get_airspeed_and_update_scaling(float dt)
{
	_airspeed_validated_sub.update();
	const bool airspeed_valid = PX4_ISFINITE(_airspeed_validated_sub.get().calibrated_airspeed_m_s)
				    && (hrt_elapsed_time(&_airspeed_validated_sub.get().timestamp) < 1_s);

	// if no airspeed measurement is available out best guess is to use the trim airspeed
	float airspeed = _param_fw_airspd_trim.get();

	if (_param_fw_use_airspd.get() && airspeed_valid) {
		/* prevent numerical drama by requiring 0.5 m/s minimal speed */
		airspeed = math::max(0.5f, _airspeed_validated_sub.get().calibrated_airspeed_m_s);

		if (dt > 1.f) {
			_airspeed_filter_for_torque_scaling.reset(airspeed);

		} else {
			airspeed = _airspeed_filter_for_torque_scaling.update(airspeed, dt);
		}

	} else {
		// VTOL: if we have no airspeed available and we are in hover mode then assume the lowest airspeed possible
		// this assumption is good as long as the vehicle is not hovering in a headwind which is much larger
		// than the stall airspeed
		if (_vehicle_status.is_vtol && _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
		    && !_vehicle_status.in_transition_mode) {
			airspeed = _param_fw_airspd_stall.get();
		}
	}

	/*
	 * For scaling our actuators using anything less than the stall
	 * speed doesn't make any sense - its the strongest reasonable deflection we
	 * want to do in flight and it's the baseline a human pilot would choose.
	 *
	 * Forcing the scaling to this value allows reasonable handheld tests.
	 */


	if (_param_fw_arsp_scale_en.get()) {
		const float min_airspeed = math::max(_param_fw_airspd_stall.get(), 0.1f);
		const float airspeed_constrained = math::max(airspeed, min_airspeed);
		_airspeed_scaling = _param_fw_airspd_trim.get() / airspeed_constrained;

	} else {
		_airspeed_scaling = 1.0f;
	}

	return airspeed;
}

void FixedwingRateControl::Run()
{
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// only run controller if angular velocity changed
	if (_vehicle_angular_velocity_sub.updated() || (hrt_elapsed_time(&_last_run) > 20_ms)) { //TODO rate!

		// only update parameters if they changed
		bool params_updated = _parameter_update_sub.updated();

		// check for parameter updates
		if (params_updated) {
			// clear update
			parameter_update_s pupdate;
			_parameter_update_sub.copy(&pupdate);

			// update parameters from storage
			updateParams();
			parameters_update();
		}

// PITCH_FF_SHAPER_BEGIN
		const hrt_abstime pff_previous_sample = _last_run;
		double pff_measured_dt = NAN;
// PITCH_FF_SHAPER_END
		float dt = 0.f;

		static constexpr float DT_MIN = 0.002f;
		static constexpr float DT_MAX = 0.04f;

		vehicle_angular_velocity_s vehicle_angular_velocity{};
// ROLL_TBR_BEGIN
		const hrt_abstime tbr_previous_sample = _last_run;
		float tbr_measured_dt = NAN;
		roll_tbr::Result tbr_result{};
		_roll_tbr_status = {};
		_roll_tbr_status.controller_update_counter = ++_roll_tbr_update_counter;
		_roll_tbr_status.p_sp_used = _roll_tbr_status.p_used = NAN;
		_roll_tbr_status.native_roll_output = NAN;
		_roll_tbr_status.native_roll_p_term = _roll_tbr_status.native_roll_i_term = NAN;
		_roll_tbr_status.native_roll_d_term = _roll_tbr_status.native_roll_ff_term = NAN;
// ROLL_TBR_END

		if (_vehicle_angular_velocity_sub.copy(&vehicle_angular_velocity)) {
// ROLL_TBR_BEGIN
			if (tbr_previous_sample != 0 && vehicle_angular_velocity.timestamp_sample > tbr_previous_sample) {
				tbr_measured_dt = (vehicle_angular_velocity.timestamp_sample - tbr_previous_sample) * 1e-6f;
			}
// ROLL_TBR_END
// PITCH_FF_SHAPER_BEGIN
			if (pff_previous_sample != 0 && vehicle_angular_velocity.timestamp_sample > pff_previous_sample) {
				pff_measured_dt = (vehicle_angular_velocity.timestamp_sample - pff_previous_sample) * 1e-6;
			}
// PITCH_FF_SHAPER_END
			dt = math::constrain((vehicle_angular_velocity.timestamp_sample - _last_run) * 1e-6f, DT_MIN, DT_MAX);
			_last_run = vehicle_angular_velocity.timestamp_sample;
		}

		if (dt < DT_MIN || dt > DT_MAX) {
			const hrt_abstime time_now_us = hrt_absolute_time();
			dt = math::constrain((time_now_us - _last_run) * 1e-6f, DT_MIN, DT_MAX);
			_last_run = time_now_us;
		}

		vehicle_angular_velocity_s angular_velocity{};
		_vehicle_angular_velocity_sub.copy(&angular_velocity);

		Vector3f rates(angular_velocity.xyz);
		Vector3f angular_accel{angular_velocity.xyz_derivative};

		// Tailsitter: rotate setpoint from hover to fixed-wing frame (controller is in fixed-wing frame, interface in hover)
		if (_vehicle_status.is_vtol_tailsitter) {
			rates = Vector3f(-angular_velocity.xyz[2], angular_velocity.xyz[1], angular_velocity.xyz[0]);
			angular_accel = Vector3f(-angular_velocity.xyz_derivative[2], angular_velocity.xyz_derivative[1],
						 angular_velocity.xyz_derivative[0]);
		}

		// vehicle status update must be before the vehicle_control_mode poll, otherwise rate sp are not published during whole transition
		_vehicle_status_sub.update(&_vehicle_status);
		const bool is_in_transition_except_tailsitter = _vehicle_status.in_transition_mode
				&& !_vehicle_status.is_vtol_tailsitter;
		const bool is_fixed_wing = _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING;
		_in_fw_or_transition_wo_tailsitter_transition =  is_fixed_wing || is_in_transition_except_tailsitter;

		_vehicle_control_mode_sub.update(&_vcontrol_mode);

		vehicle_land_detected_poll();

		vehicle_manual_poll();
		vehicle_land_detected_poll();

		/* if we are in rotary wing mode, do nothing */
		if (_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING && !_vehicle_status.is_vtol) {
// ROLL_TBR_BEGIN
			_roll_tbr.reset();
// ROLL_TBR_END
// PITCH_FF_SHAPER_BEGIN
			_pitch_ff_shaper.reset();
// PITCH_FF_SHAPER_END
			perf_end(_loop_perf);
			return;
		}

// PITCH_FF_SHAPER_BEGIN
		if (!_vcontrol_mode.flag_control_rates_enabled) {
			_pitch_ff_shaper.reset();
		}
// PITCH_FF_SHAPER_END
		if (_vcontrol_mode.flag_control_rates_enabled) {

			const float airspeed = get_airspeed_and_update_scaling(dt);

			/* reset integrals where needed */
			if (_rates_sp.reset_integral) {
				_rate_control.resetIntegral();
			}

			// Reset integrators if the aircraft is on ground or not in a state where the fw attitude controller is run
			if (_landed || !_in_fw_or_transition_wo_tailsitter_transition) {

				_gain_compression.reset();
				_rate_control.resetIntegral();
			}

			// Update saturation status from control allocation feedback
			// TODO: send the unallocated value directly for better anti-windup
			Vector3<bool> diffthr_enabled(
				_param_vt_fw_difthr_en & static_cast<int32_t>(VTOLFixedWingDifferentialThrustEnabledBit::ROLL_BIT),
				_param_vt_fw_difthr_en & static_cast<int32_t>(VTOLFixedWingDifferentialThrustEnabledBit::PITCH_BIT),
				_param_vt_fw_difthr_en & static_cast<int32_t>(VTOLFixedWingDifferentialThrustEnabledBit::YAW_BIT)
			);

			if (_vehicle_status.is_vtol_tailsitter) {
				// Swap roll and yaw
				diffthr_enabled.swapRows(0, 2);
			}

			// saturation handling for axis controlled by differential thrust (VTOL only)
			control_allocator_status_s control_allocator_status;

			// Set saturation flags for VTOL differential thrust feature
			// If differential thrust is enabled in an axis, assume it's the only torque authority and only update saturation using matrix 0 allocating the motors.
			if (_control_allocator_status_subs[0].update(&control_allocator_status)) {
				for (size_t i = 0; i < 3; i++) {
					if (diffthr_enabled(i)) {
						_rate_control.setPositiveSaturationFlag(i, control_allocator_status.unallocated_torque[i] > FLT_EPSILON);
						_rate_control.setNegativeSaturationFlag(i, control_allocator_status.unallocated_torque[i] < -FLT_EPSILON);
					}
				}
			}

			// Set saturation flags for control surface controlled axes
			if (_control_allocator_status_subs[_vehicle_status.is_vtol ? 1 : 0].update(&control_allocator_status)) {
				for (size_t i = 0; i < 3; i++) {
					if (!diffthr_enabled(i)) {
						_rate_control.setPositiveSaturationFlag(i, control_allocator_status.unallocated_torque[i] > FLT_EPSILON);
						_rate_control.setNegativeSaturationFlag(i, control_allocator_status.unallocated_torque[i] < -FLT_EPSILON);
					}
				}
			}

			/* bi-linear interpolation over airspeed for actuator trim scheduling */
			Vector3f trim(_param_trim_roll.get(), _param_trim_pitch.get(), _param_trim_yaw.get());
			trim *= _airspeed_scaling * _airspeed_scaling;

			if (airspeed < _param_fw_airspd_trim.get()) {
				trim(0) += interpolate(airspeed, _param_fw_airspd_min.get(), _param_fw_airspd_trim.get(),
						       _param_fw_dtrim_r_vmin.get(),
						       0.0f);
				trim(1) += interpolate(airspeed, _param_fw_airspd_min.get(), _param_fw_airspd_trim.get(),
						       _param_fw_dtrim_p_vmin.get(),
						       0.0f);
				trim(2) += interpolate(airspeed, _param_fw_airspd_min.get(), _param_fw_airspd_trim.get(),
						       _param_fw_dtrim_y_vmin.get(),
						       0.0f);

			} else {
				trim(0) += interpolate(airspeed, _param_fw_airspd_trim.get(), _param_fw_airspd_max.get(), 0.0f,
						       _param_fw_dtrim_r_vmax.get());
				trim(1) += interpolate(airspeed, _param_fw_airspd_trim.get(), _param_fw_airspd_max.get(), 0.0f,
						       _param_fw_dtrim_p_vmax.get());
				trim(2) += interpolate(airspeed, _param_fw_airspd_trim.get(), _param_fw_airspd_max.get(), 0.0f,
						       _param_fw_dtrim_y_vmax.get());
			}

			if (_vcontrol_mode.flag_control_rates_enabled) {
				_rates_sp_sub.update(&_rates_sp);

				Vector3f body_rates_setpoint = Vector3f(_rates_sp.roll, _rates_sp.pitch, _rates_sp.yaw);
				// PITCH_DIAG_BEGIN
				_pitch_diag_inner = {};
				_pitch_diag_inner.controller_dt = dt;
				_pitch_diag_inner.rate_sp_source_timestamp = _rates_sp.timestamp;
				_pitch_diag_inner.q_sp_used = body_rates_setpoint(1);
				_pitch_diag_inner.q_used = rates(1);
				_pitch_diag_inner.q_error = body_rates_setpoint(1) - rates(1);
				_pitch_diag_inner.q_dot_used = angular_accel(1);
				_pitch_diag_inner.airspeed_scaling = _airspeed_scaling;
				_pitch_diag_inner.airspeed_scaling_sq = _airspeed_scaling * _airspeed_scaling;
				_pitch_diag_inner.fw_pr_p = _param_fw_pr_p.get();
				_pitch_diag_inner.fw_pr_i = _param_fw_pr_i.get();
				_pitch_diag_inner.fw_pr_d = _param_fw_pr_d.get();
				_pitch_diag_inner.fw_pr_ff = _param_fw_pr_ff.get();
				_pitch_diag_inner.fw_pr_imax = _param_fw_pr_imax.get();
				_pitch_diag_inner.reset_integral = _rates_sp.reset_integral;
				_pitch_diag_inner.rates_enabled = _vcontrol_mode.flag_control_rates_enabled;
				_pitch_diag_inner.attitude_enabled = _vcontrol_mode.flag_control_attitude_enabled;
				_pitch_diag_inner.manual_enabled = _vcontrol_mode.flag_control_manual_enabled;
				_pitch_diag_inner.fixed_wing_or_transition = _in_fw_or_transition_wo_tailsitter_transition;
				_pitch_diag_inner.controller_active = true;
				_pitch_diag_inner.fixed_wing_active = is_fixed_wing;
				_pitch_diag_inner.mission_active = _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION;
				_pitch_diag_inner.landed = _landed;
				_pitch_diag_inner.nav_state = _vehicle_status.nav_state;
				// PITCH_DIAG_END

				// Tailsitter: rotate setpoint from hover to fixed-wing frame (controller is in fixed-wing frame, interface in hover)
				if (_vehicle_status.is_vtol_tailsitter) {
					body_rates_setpoint = Vector3f(-_rates_sp.yaw, _rates_sp.pitch, _rates_sp.roll);
				}

				const Vector3f gain_ff(_param_fw_rr_ff.get(), _param_fw_pr_ff.get(), _param_fw_yr_ff.get());
				const Vector3f scaled_gain_ff = gain_ff / _airspeed_scaling;
				_rate_control.setFeedForwardGain(scaled_gain_ff);
				// PITCH_DIAG_BEGIN
				_pitch_diag_inner.effective_ff_gain = scaled_gain_ff(1);
				_pitch_diag_inner.pitch_ff_term = scaled_gain_ff(1) * body_rates_setpoint(1);
				// PITCH_DIAG_END

				// Run attitude RATE controllers which need the desired attitudes from above, add trim.
// ROLL_TBR_BEGIN
				rate_ctrl_status_s tbr_integral_before{};
				_rate_control.getRateControlStatus(tbr_integral_before);
				_roll_tbr_status.p_sp_used = body_rates_setpoint(0);
				_roll_tbr_status.p_used = rates(0);
				_roll_tbr_status.native_roll_p_term = _param_fw_rr_p.get() * (body_rates_setpoint(0) - rates(0));
				_roll_tbr_status.native_roll_i_term = tbr_integral_before.rollspeed_integ;
				_roll_tbr_status.native_roll_d_term = -_param_fw_rr_d.get() * angular_accel(0);
				_roll_tbr_status.native_roll_ff_term = scaled_gain_ff(0) * body_rates_setpoint(0);
// ROLL_TBR_END
				// PITCH_DIAG_BEGIN
				rate_ctrl_status_s pitch_diag_integral_before{};
				_rate_control.getRateControlStatus(pitch_diag_integral_before);
				_pitch_diag_inner.pitch_p_term = _param_fw_pr_p.get() * (body_rates_setpoint(1) - rates(1));
				_pitch_diag_inner.pitch_i_term_before = pitch_diag_integral_before.pitchspeed_integ;
				_pitch_diag_inner.pitch_d_term = -_param_fw_pr_d.get() * angular_accel(1);
				_pitch_diag_inner.i_at_limit_before = fabsf(pitch_diag_integral_before.pitchspeed_integ) >=
							_param_fw_pr_imax.get() - 1e-6f;
				// PITCH_DIAG_END
				const Vector3f angular_acceleration_setpoint = _rate_control.update(rates, body_rates_setpoint, angular_accel, dt, _landed);
				// PITCH_DIAG_BEGIN
				rate_ctrl_status_s pitch_diag_integral_after{};
				_rate_control.getRateControlStatus(pitch_diag_integral_after);
				_pitch_diag_inner.pitch_i_term_after = pitch_diag_integral_after.pitchspeed_integ;
				_pitch_diag_inner.pitch_i_delta = pitch_diag_integral_after.pitchspeed_integ - pitch_diag_integral_before.pitchspeed_integ;
				_pitch_diag_inner.pitch_rate_control_sum = angular_acceleration_setpoint(1);
				_pitch_diag_inner.i_at_limit_after = fabsf(pitch_diag_integral_after.pitchspeed_integ) >=
							_param_fw_pr_imax.get() - 1e-6f;
				// PITCH_DIAG_END

				Vector3f control_u = _gain_compression.getGains().emult(angular_acceleration_setpoint * _airspeed_scaling * _airspeed_scaling);
				// PITCH_DIAG_BEGIN
				_pitch_diag_inner.gain_compression_pitch = _gain_compression.getGains()(1);
				_pitch_diag_inner.pitch_control_after_scaling = control_u(1);
				// PITCH_DIAG_END

// PITCH_FF_SHAPER_BEGIN
				_pitch_ff_status = {};
				_pitch_ff_status.gain_compression_pitch = _gain_compression.getGains()(1);
				_pitch_ff_status.effective_ff_gain = scaled_gain_ff(1);
				_pitch_ff_status.airspeed_scaling_sq = _airspeed_scaling * _airspeed_scaling;
				_pitch_ff_status.native_pitch_control = control_u(1);
// PITCH_FF_SHAPER_END
				_gain_compression.update(control_u, dt);
// PITCH_FF_SHAPER_BEGIN
				// Only the final Pitch output changes. Compression adaptation above sees Native command.
				_wing_phase_sub.update(&_wing_phase);
				const hrt_abstime pff_now = hrt_absolute_time();
				pitch_ff_phase_shaper::Input pff_input{};
				pff_input.enable = _param_flap_pff_en.get() == 1;
				pff_input.armed = _vcontrol_mode.flag_armed;
				pff_input.supported = is_fixed_wing && !_vehicle_status.is_vtol
					&& !_vehicle_status.is_vtol_tailsitter && !_vehicle_status.in_transition_mode;
				pff_input.mission = _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION;
				pff_input.landed = _landed;
				pff_input.controller_active = _vcontrol_mode.flag_control_rates_enabled && _vcontrol_mode.flag_control_auto_enabled;
				pff_input.phase_valid = _wing_phase.phase_valid;
				pff_input.phase_fresh = pitch_ff_phase_shaper::phaseFresh(_wing_phase.timestamp, pff_now);
				pff_input.phase_rad = _wing_phase.phase_rad;
				pff_input.flap_frequency_hz = _wing_phase.flap_frequency_hz;
				pff_input.q_sp = body_rates_setpoint(1);
				// Reject faulty raw intervals rather than hiding them behind Native dt clamping.
				pff_input.dt = std::isfinite(pff_measured_dt) && pff_measured_dt > 0. && pff_measured_dt <= .1 ? dt : NAN;
				pff_input.numeric_inputs_valid = rates.isAllFinite() && body_rates_setpoint.isAllFinite()
					&& angular_accel.isAllFinite() && control_u.isAllFinite() && trim.isAllFinite();
				// A generous numerical containment bound, derived from existing Native Pitch rate limits.
				pff_input.amplitude_bound = 4. * static_cast<double>(math::radians(math::max(_param_pff_pitch_limit_pos.get(),
					_param_pff_pitch_limit_neg.get())));
				const pitch_ff_phase_shaper::Config pff_config{_param_flap_pff_alpha.get(), _param_flap_pff_adapt.get()};
				auto pff_result = _pitch_ff_shaper.update(pff_config, pff_input);
				auto pff_mapping = pitch_ff_phase_shaper::mapPitchOutput(pff_result, _pitch_ff_status.effective_ff_gain,
					_pitch_ff_status.gain_compression_pitch, _pitch_ff_status.airspeed_scaling_sq, control_u(1), trim(1));
				if (!pff_mapping.valid) {
					_pitch_ff_shaper.reset();
					pff_result = {};
					pff_result.reset_reason = pitch_ff_phase_shaper::ResetReason::NONFINITE_STATE;
					pff_mapping = {};
					pff_mapping.shaped_pitch_control = control_u(1);
				}
				// Avoid even a float addition/cast on the default-off Native path.
				if (std::fabs(pff_mapping.normalized_correction) > 0.) {
					control_u(1) = static_cast<float>(pff_mapping.shaped_pitch_control);
				}
// PITCH_FF_SHAPER_END

				// Special case yaw in Acro: if the parameter FW_ACRO_YAW_EN is not set then don't rate-control yaw
				if (!_vcontrol_mode.flag_control_attitude_enabled && _vcontrol_mode.flag_control_manual_enabled
				    && !_param_fw_acro_yaw_en.get()) {
					control_u(2) = _manual_control_setpoint.yaw * _param_fw_man_y_sc.get();
					_rate_control.resetIntegral(2);
				}

				if (control_u.isAllFinite()) {
					matrix::constrain(control_u + trim, -1.f, 1.f).copyTo(_vehicle_torque_setpoint.xyz);

				} else {
					_rate_control.resetIntegral();
					trim.copyTo(_vehicle_torque_setpoint.xyz);
				}
// PITCH_FF_SHAPER_BEGIN
				_pitch_ff_status.timestamp = hrt_absolute_time();
				_pitch_ff_status.timestamp_sample = angular_velocity.timestamp_sample;
				_pitch_ff_status.rate_sp_source_timestamp = _rates_sp.timestamp;
				_pitch_ff_status.wing_phase_timestamp = _wing_phase.timestamp;
				_pitch_ff_status.update_counter = ++_pitch_ff_update_counter;
				_pitch_ff_status.controller_dt = dt;
				_pitch_ff_status.measured_dt = pff_measured_dt;
				_pitch_ff_status.phase_rad = _wing_phase.phase_rad;
				_pitch_ff_status.phase_age = _wing_phase.timestamp != 0 && pff_now >= _wing_phase.timestamp ?
					(pff_now - _wing_phase.timestamp) * 1e-6f : NAN;
				_pitch_ff_status.flap_frequency_hz = _wing_phase.flap_frequency_hz;
				_pitch_ff_status.q_sp_used = body_rates_setpoint(1);
				_pitch_ff_status.coeff_cos_1f = pff_result.coeff_cos_1f;
				_pitch_ff_status.coeff_sin_1f = pff_result.coeff_sin_1f;
				_pitch_ff_status.coeff_cos_2f_internal = pff_result.coeff_cos_2f_internal;
				_pitch_ff_status.coeff_sin_2f_internal = pff_result.coeff_sin_2f_internal;
				_pitch_ff_status.baseline = pff_result.baseline;
				_pitch_ff_status.fade = pff_result.fade;
				_pitch_ff_status.warm_cycles = pff_result.warm_cycles;
				_pitch_ff_status.qhat_1f = pff_result.qhat_1f;
				_pitch_ff_status.qhat_2f_internal = pff_result.qhat_2f_internal;
				_pitch_ff_status.qhat_1f_reference = pff_result.qhat_1f_reference;
				_pitch_ff_status.qhat_2f_reference_internal = pff_result.qhat_2f_reference_internal;
				_pitch_ff_status.qhat_1f_applied = pff_result.qhat_1f_applied;
				_pitch_ff_status.estimation_error = pff_result.estimation_error;
				_pitch_ff_status.alpha_effective = pff_result.alpha_effective;
				_pitch_ff_status.active = pff_result.active;
				_pitch_ff_status.warmup_complete = pff_result.warmup_complete;
				_pitch_ff_status.fading = pff_result.fading;
				_pitch_ff_status.alpha_command = _param_flap_pff_alpha.get();
				_pitch_ff_status.native_ff_term = scaled_gain_ff(1) * body_rates_setpoint(1);
				_pitch_ff_status.ff_delta_rate_control = pff_mapping.ff_delta_rate_control;
				_pitch_ff_status.shaped_ff_term = _pitch_ff_status.native_ff_term + _pitch_ff_status.ff_delta_rate_control;
				_pitch_ff_status.normalized_correction = pff_mapping.normalized_correction;
				_pitch_ff_status.shaped_pitch_control = control_u(1);
				_pitch_ff_status.native_pitch_preclip = _pitch_ff_status.native_pitch_control + trim(1);
				_pitch_ff_status.shaped_pitch_preclip = control_u(1) + trim(1);
				_pitch_ff_status.shaped_pitch_final = _vehicle_torque_setpoint.xyz[1];
				_pitch_ff_status.enabled = pff_input.enable;
				_pitch_ff_status.armed = pff_input.armed;
				_pitch_ff_status.phase_valid = pff_input.phase_valid;
				_pitch_ff_status.phase_fresh = pff_input.phase_fresh;
				_pitch_ff_status.frequency_valid = std::isfinite(pff_input.flap_frequency_hz) && pff_input.flap_frequency_hz > .5;
				_pitch_ff_status.fixed_wing_active = pff_input.supported;
				_pitch_ff_status.mission_active = pff_input.mission;
				_pitch_ff_status.landed = pff_input.landed;
				_pitch_ff_status.controller_active = pff_input.controller_active;
				_pitch_ff_status.reset_reason = static_cast<uint8_t>(pff_result.reset_reason);
				_pitch_ff_status_pub.publish(_pitch_ff_status);
				// Restore Native scratch input for existing diagnostics and Roll TBR validity checks.
				control_u(1) = _pitch_ff_status.native_pitch_control;
// PITCH_FF_SHAPER_END
				// PITCH_DIAG_BEGIN
				_pitch_diag_inner.pitch_trim = trim(1);
				_pitch_diag_inner.pitch_output_preclip = control_u(1) + trim(1);
				_pitch_diag_inner.pitch_output_final = _vehicle_torque_setpoint.xyz[1];
				_pitch_diag_inner.output_clip = fabsf(_pitch_diag_inner.pitch_output_preclip) > 1.f;
				_pitch_diag_inner.timestamp_sample = angular_velocity.timestamp_sample;
				_pitch_diag_inner.inner_update_counter = ++_pitch_diag_inner_update_counter;
				_pitch_diag_inner.timestamp = hrt_absolute_time();
				_pitch_diag_inner_pub.publish(_pitch_diag_inner);
				// PITCH_DIAG_END

				/* throttle passed through if it is finite */
// ROLL_TBR_BEGIN
				roll_tbr::Input tbr_input{};
				tbr_input.enable = _param_flap_tbr_en.get();
				tbr_input.armed = _vcontrol_mode.flag_armed;
				tbr_input.supported = is_fixed_wing && !_vehicle_status.is_vtol
						      && !_vehicle_status.is_vtol_tailsitter && !_vehicle_status.in_transition_mode;
				tbr_input.mission = _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION;
				tbr_input.landed = _landed;
				tbr_input.controller_active = _vcontrol_mode.flag_control_rates_enabled && _vcontrol_mode.flag_control_auto_enabled;
				tbr_input.inputs_valid = rates.isAllFinite() && body_rates_setpoint.isAllFinite()
							 && angular_accel.isAllFinite() && control_u.isAllFinite() && trim.isAllFinite();
				tbr_input.integral_reset = _rates_sp.reset_integral;
				tbr_input.dt = dt;
				tbr_input.measured_dt = tbr_measured_dt;
				tbr_input.setpoint = body_rates_setpoint(0);
				tbr_input.rate = rates(0);
				const roll_tbr::Config tbr_config{_param_flap_tbr_fc.get(), _param_flap_tbr_k.get(),
								 _param_flap_tbr_max.get(), _param_flap_tbr_slew.get()};
				tbr_result = _roll_tbr.update(tbr_config, tbr_input);
				_roll_tbr_status.native_roll_output = control_u(0) + trim(0);
				_roll_tbr_status.native_roll_clipped = _vehicle_torque_setpoint.xyz[0];

				if (tbr_result.active && fabsf(tbr_result.output) > 0.f) {
					_vehicle_torque_setpoint.xyz[0] = math::constrain(control_u(0) + trim(0) + tbr_result.output, -1.f, 1.f);
				}

				_roll_tbr_status.controller_clip = fabsf(_roll_tbr_status.native_roll_output + tbr_result.output) > 1.f;
// ROLL_TBR_END
				_vehicle_thrust_setpoint.xyz[0] = PX4_ISFINITE(_rates_sp.thrust_body[0]) ? _rates_sp.thrust_body[0] : 0.0f;

				/* scale effort by battery status */
				if (_param_fw_bat_scale_en.get() && _vehicle_thrust_setpoint.xyz[0] > 0.1f) {

					if (_battery_status_sub.updated()) {
						battery_status_s battery_status{};

						if (_battery_status_sub.copy(&battery_status) && battery_status.connected && battery_status.scale > 0.f) {
							_battery_scale = battery_status.scale;
						}
					}

					_vehicle_thrust_setpoint.xyz[0] *= _battery_scale;
				}
			}

			// publish rate controller status
			rate_ctrl_status_s rate_ctrl_status{};
			_rate_control.getRateControlStatus(rate_ctrl_status);
			rate_ctrl_status.timestamp = hrt_absolute_time();

			_rate_ctrl_status_pub.publish(rate_ctrl_status);

		} else {
			// full manual
// ROLL_TBR_BEGIN
			_roll_tbr.reset();
// ROLL_TBR_END
			_gain_compression.reset();
			_rate_control.resetIntegral();
		}

		// Add feed-forward from roll control output to yaw control output
// ROLL_TBR_BEGIN
		if (!_vcontrol_mode.flag_control_rates_enabled) {
			_roll_tbr_status.native_roll_output = _vehicle_torque_setpoint.xyz[0];
			_roll_tbr_status.native_roll_clipped = _vehicle_torque_setpoint.xyz[0];
		}

		_roll_tbr_status.timestamp = hrt_absolute_time();
		_roll_tbr_status.timestamp_sample = angular_velocity.timestamp_sample;
		_roll_tbr_status.controller_dt = dt;
		_roll_tbr_status.measured_dt = tbr_measured_dt;
		_roll_tbr_status.mission_active = _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION;
		_roll_tbr_status.tbr_enabled = _param_flap_tbr_en.get() == 1;
		_roll_tbr_status.tbr_active = tbr_result.active;
		const roll_tbr::Config tbr_log_config{_param_flap_tbr_fc.get(), _param_flap_tbr_k.get(),
						    _param_flap_tbr_max.get(), _param_flap_tbr_slew.get()};
		_roll_tbr_status.config_valid = roll_tbr::RollTbr::validConfig(tbr_log_config, dt)
					  && (_param_flap_tbr_en.get() == 0 || _param_flap_tbr_en.get() == 1);
		_roll_tbr_status.reset_reason = static_cast<uint8_t>(tbr_result.reset_reason);
		_roll_tbr_status.p_error_raw = _roll_tbr_status.p_sp_used - _roll_tbr_status.p_used;
		_roll_tbr_status.p_error_filtered = tbr_result.filtered;
		_roll_tbr_status.tbr_raw = tbr_result.raw;
		_roll_tbr_status.tbr_after_cap = tbr_result.capped;
		_roll_tbr_status.tbr_final = tbr_result.output;
		_roll_tbr_status.cap_active = tbr_result.cap_active;
		_roll_tbr_status.slew_active = tbr_result.slew_active;
		_roll_tbr_status.final_roll_output = _vehicle_torque_setpoint.xyz[0];
		_roll_tbr_status.native_pitch_output = _vehicle_torque_setpoint.xyz[1];

		if (_roll_tbr_status.timestamp - _roll_tbr_last_publish >= 10_ms) {
			_roll_tbr_status_pub.publish(_roll_tbr_status);
			_roll_tbr_last_publish = _roll_tbr_status.timestamp;
		}
// ROLL_TBR_END
		// This can be used to counteract the adverse yaw effect when rolling the plane
		_vehicle_torque_setpoint.xyz[2] = math::constrain(_vehicle_torque_setpoint.xyz[2] + _param_fw_rll_to_yaw_ff.get() *
						  _vehicle_torque_setpoint.xyz[0], -1.f, 1.f);

		// Tailsitter: rotate back to body frame from airspeed frame
		if (_vehicle_status.is_vtol_tailsitter) {
			const float helper = _vehicle_torque_setpoint.xyz[0];
			_vehicle_torque_setpoint.xyz[0] = _vehicle_torque_setpoint.xyz[2];
			_vehicle_torque_setpoint.xyz[2] = -helper;
		}

		/* Only publish if any of the proper modes are enabled */
		if (_vcontrol_mode.flag_control_rates_enabled ||
		    _vcontrol_mode.flag_control_attitude_enabled ||
		    _vcontrol_mode.flag_control_manual_enabled) {
			{
				_vehicle_thrust_setpoint.timestamp = hrt_absolute_time();
				_vehicle_thrust_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
				_vehicle_thrust_setpoint_pub.publish(_vehicle_thrust_setpoint);

				_vehicle_torque_setpoint.timestamp = hrt_absolute_time();
				_vehicle_torque_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
				_vehicle_torque_setpoint_pub.publish(_vehicle_torque_setpoint);
			}
		}

		updateActuatorControlsStatus(dt);

		// Manual flaps/spoilers control, also active in VTOL Hover. Is handled and published in FW Position controller/VTOL module if Auto.
		if (_vcontrol_mode.flag_control_manual_enabled) {

			// Flaps control
			float flaps_control = 0.f; // default to no flaps

			/* map flaps by default to manual if valid */
			if (PX4_ISFINITE(_manual_control_setpoint.flaps)) {
				flaps_control = math::max(_manual_control_setpoint.flaps, 0.f); // do not consider negative switch settings
			}

			normalized_unsigned_setpoint_s flaps_setpoint;
			flaps_setpoint.timestamp = hrt_absolute_time();
			flaps_setpoint.normalized_setpoint = flaps_control;
			_flaps_setpoint_pub.publish(flaps_setpoint);

			// Spoilers control
			float spoilers_control = 0.f; // default to no spoilers

			switch (_param_fw_spoilers_man.get()) {
			case 0:
				break;

			case 1:
				// do not consider negative switch settings
				spoilers_control = PX4_ISFINITE(_manual_control_setpoint.flaps) ? math::max(_manual_control_setpoint.flaps, 0.f) : 0.f;
				break;

			case 2:
				// do not consider negative switch settings
				spoilers_control = PX4_ISFINITE(_manual_control_setpoint.aux1) ? math::max(_manual_control_setpoint.aux1, 0.f) : 0.f;
				break;
			}

			normalized_unsigned_setpoint_s spoilers_setpoint;
			spoilers_setpoint.timestamp = hrt_absolute_time();
			spoilers_setpoint.normalized_setpoint = spoilers_control;
			_spoilers_setpoint_pub.publish(spoilers_setpoint);
		}
	}

	// backup schedule
	ScheduleDelayed(20_ms);

	perf_end(_loop_perf);
}

void FixedwingRateControl::updateActuatorControlsStatus(float dt)
{
	for (int i = 0; i < 3; i++) {

		// We assume that the attitude is actuated by control surfaces
		// consuming power only when they move
		const float control_signal = _vehicle_torque_setpoint.xyz[i] - _control_prev[i];
		_control_prev[i] = _vehicle_torque_setpoint.xyz[i];

		_control_energy[i] += control_signal * control_signal * dt;
	}

	_energy_integration_time += dt;

	if (_energy_integration_time > 500e-3f) {

		actuator_controls_status_s status;
		status.timestamp = _vehicle_torque_setpoint.timestamp;

		for (int i = 0; i < 3; i++) {
			status.control_power[i] = _control_energy[i] / _energy_integration_time;
			_control_energy[i] = 0.f;
		}

		_actuator_controls_status_pub.publish(status);
		_energy_integration_time = 0.f;
	}
}

int FixedwingRateControl::task_spawn(int argc, char *argv[])
{
	bool vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			vtol = true;
		}
	}

	FixedwingRateControl *instance = new FixedwingRateControl(vtol);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int FixedwingRateControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FixedwingRateControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
fw_rate_control is the fixed-wing rate controller.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("fw_rate_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int fw_rate_control_main(int argc, char *argv[])
{
	return FixedwingRateControl::main(argc, argv);
}
