/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

#include "RpmPid.hpp"

#include <cmath>

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>

static inline float wrap360(float deg)
{
	float out = fmodf(deg, 360.f);
	if (out < 0.f) {
		out += 360.f;
	}
	return out;
}

static inline float wrap180(float deg)
{
	float out = fmodf(deg + 180.f, 360.f);
	if (out < 0.f) {
		out += 360.f;
	}

	return out - 180.f;
}

static inline float smoothstep01(float x)
{
	x = math::constrain(x, 0.f, 1.f);
	return x * x * (3.f - 2.f * x);
}

static inline float downstroke_weight(float phase_deg, float blend_deg)
{
	const float phase = wrap360(phase_deg);
	const float b = math::constrain(blend_deg, 0.f, 45.f);

	if (b <= FLT_EPSILON) {
		return (phase > 90.f && phase < 270.f) ? 1.f : 0.f;
	}

	if (phase < (90.f - b)) {
		return 0.f;
	}

	if (phase < (90.f + b)) {
		return smoothstep01((phase - (90.f - b)) / (2.f * b));
	}

	if (phase < (270.f - b)) {
		return 1.f;
	}

	if (phase < (270.f + b)) {
		return 1.f - smoothstep01((phase - (270.f - b)) / (2.f * b));
	}

	return 0.f;
}

RpmPid::RpmPid() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
	updateParams();
}

bool RpmPid::init()
{
	if (!_rpm_sub.registerCallback()) {
		PX4_ERR("rpm callback registration failed");
		return false;
	}

	return true;
}

void RpmPid::updateParams()
{
	ModuleParams::updateParams();

	_flap_f_min = _param_flap_f_min.get();
	_flap_f_max = _param_flap_f_max.get();
	_flap_ratio = _param_flap_ratio.get();
	_kp = _param_flap_kp.get();
	_ki = _param_flap_ki.get();
	_kd = _param_flap_kd.get();
	_i_max = _param_flap_i_max.get();
	_phase_ff_en = (_param_flap_phase_en.get() != 0);
	_phase_ff_amp = _param_flap_phase_amp.get();
	_phase_ff_duty = _param_flap_phase_duty.get();
	_phase_ff_shift_deg = _param_flap_phase_shift.get();
	_fm_mode = _param_flap_fm_mode.get();
	_fm_delta_hz = _param_flap_fm_delta.get();
	_sc_delta_cmd = _param_flap_sc_delta.get();
	_sc_delta_slew = _param_flap_sc_slew.get();
	_sc_blend_deg = _param_flap_sc_blend.get();
	_sc_fmax_mult = _param_flap_sc_fmax_m.get();
	_sc_phase_k_hz_per_deg = _param_flap_sc_ph_k.get();
	_sc_i_limit_a = _param_flap_sc_ilim_a.get();
	_sc_recover_tau_s = _param_flap_sc_rec_tau.get();
	_sc_u_slew = _param_flap_sc_u_slew.get();

	// basic sanity
	if (_flap_f_max < _flap_f_min) {
		_flap_f_max = _flap_f_min;
	}

	_phase_ff_amp = math::constrain(_phase_ff_amp, 0.f, 0.2f);
	_phase_ff_duty = math::constrain(_phase_ff_duty, 0.1f, 0.9f);
	_phase_ff_shift_deg = math::constrain(_phase_ff_shift_deg, -180.f, 180.f);
	_fm_mode = math::constrain(_fm_mode, (int32_t)0, (int32_t)3);
	_fm_delta_hz = math::constrain(_fm_delta_hz, 0.f, 10.f);
	_sc_delta_cmd = math::constrain(_sc_delta_cmd, 0.1f, 0.9f);
	_sc_delta_slew = math::constrain(_sc_delta_slew, 0.01f, 10.f);
	_sc_blend_deg = math::constrain(_sc_blend_deg, 0.f, 45.f);
	_sc_fmax_mult = math::constrain(_sc_fmax_mult, 1.f, 5.f);
	_sc_phase_k_hz_per_deg = math::constrain(_sc_phase_k_hz_per_deg, 0.f, 0.05f);
	_sc_i_limit_a = math::constrain(_sc_i_limit_a, 0.f, 200.f);
	_sc_recover_tau_s = math::constrain(_sc_recover_tau_s, 0.01f, 5.f);
	_sc_u_slew = math::constrain(_sc_u_slew, 0.f, 20.f);

	if (!PX4_ISFINITE(_sc_delta_slewed)) {
		_sc_delta_slewed = _sc_delta_cmd;
	}

	if (!PX4_ISFINITE(_sc_delta_applied)) {
		_sc_delta_applied = _sc_delta_cmd;
	}
}

void RpmPid::Run()
{
	if (should_exit()) {
		_rpm_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	// parameter update
	if (_parameter_update_sub.updated()) {
		parameter_update_s p{};
		_parameter_update_sub.copy(&p);
		updateParams();
	}

	rpm_s rpm{};

	while (_rpm_sub.update(&rpm)) {
		const hrt_abstime now = hrt_absolute_time();
		_wing_phase_sub.update(&_wing_phase);
		_esc_status_sub.update(&_esc_status);
		_battery_status_sub.update(&_battery_status);

		if (_last_run == 0) {
			_last_run = now;
			_integral = 0.f;
			_prev_error = 0.f;
			_last_u_out = NAN;
			continue;
		}

		const float dt = (now - _last_run) * 1e-6f;
		_last_run = now;

		if (dt <= 0.f || dt > 0.5f) {
			continue;
		}

		vehicle_status_s status{};
		_vehicle_status_sub.copy(&status);

		// Default output: invalid (do not use), selection happens downstream
		float u_out = NAN;
		float u_ref = NAN;
		float u_pid = NAN;
		float f_eff_sp = NAN;
		float f_inst_sp = NAN;
		float rpm_sp = NAN;
		float rpm_meas = NAN;
		float omega_ref = NAN;
		float omega_meas = NAN;
		float phase_deg_used = NAN;
		float phase_ref_deg = NAN;
		float phase_err_deg = NAN;
		float f_up_cmd_hz = NAN;
		float f_down_cmd_hz = NAN;
		bool in_downstroke = false;
		float wave_cmd = NAN;
		float wave_meas = NAN;
		float i_meas_a = NAN;
		float v_meas_v = NAN;
		float p_meas_w = NAN;
		uint8_t current_source = flap_control_status_s::CURRENT_SOURCE_NONE;
		uint16_t saturation_flags = 0;

		const bool phase_valid = (now - _wing_phase.timestamp) < 200000 && PX4_ISFINITE(_wing_phase.phase_deg);

		if (phase_valid) {
			phase_deg_used = wrap360(_wing_phase.phase_deg + _phase_ff_shift_deg);

			if (!_cycle_tracking_valid) {
				_cycle_tracking_valid = true;
				_last_hall_count = _wing_phase.hall_pulse_count;
				_cycle_start_ts = _wing_phase.timestamp;
				_last_cycle_sample_ts = _wing_phase.timestamp;
				_down_time_s = 0.f;
				_cycle_time_s = 0.f;

			} else {
				if (_wing_phase.timestamp > _last_cycle_sample_ts) {
					const float dt_phase = (_wing_phase.timestamp - _last_cycle_sample_ts) * 1e-6f;

					if (dt_phase > 0.f && dt_phase < 0.2f) {
						_cycle_time_s += dt_phase;

						if (phase_deg_used > 90.f && phase_deg_used < 270.f) {
							_down_time_s += dt_phase;
						}
					}

					_last_cycle_sample_ts = _wing_phase.timestamp;
				}

				if (_wing_phase.hall_pulse_count != _last_hall_count) {
					if (_cycle_time_s > 1e-4f) {
						_delta_meas = math::constrain(_down_time_s / _cycle_time_s, 0.f, 1.f);
					}

					_cycle_start_ts = _wing_phase.timestamp;
					_cycle_time_s = 0.f;
					_down_time_s = 0.f;
					_last_hall_count = _wing_phase.hall_pulse_count;
				}
			}

		} else {
			saturation_flags |= FLAG_PHASE_INVALID;
			_cycle_tracking_valid = false;
		}

		bool have_electrical = false;

		if ((_esc_status.timestamp > 0) && ((now - _esc_status.timestamp) < 200000) && (_esc_status.esc_count > 0)) {
			const int count = math::min(static_cast<int>(_esc_status.esc_count), static_cast<int>(esc_status_s::CONNECTED_ESC_MAX));
			float current_max = -1.f;
			float voltage_at_max = NAN;

			for (int i = 0; i < count; i++) {
				if (!(_esc_status.esc_online_flags & (1u << i))) {
					continue;
				}

				const float current = _esc_status.esc[i].esc_current;

				if (PX4_ISFINITE(current) && current >= 0.f && current > current_max) {
					current_max = current;
					voltage_at_max = _esc_status.esc[i].esc_voltage;
				}
			}

			if (current_max >= 0.f) {
				i_meas_a = current_max;
				v_meas_v = voltage_at_max;
				current_source = flap_control_status_s::CURRENT_SOURCE_ESC;
				have_electrical = true;
			}
		}

		if (!have_electrical
		    && (_battery_status.timestamp > 0)
		    && ((now - _battery_status.timestamp) < 500000)
		    && _battery_status.connected
		    && PX4_ISFINITE(_battery_status.current_a)
		    && _battery_status.current_a >= 0.f) {
			i_meas_a = _battery_status.current_a;
			v_meas_v = _battery_status.voltage_v;
			current_source = flap_control_status_s::CURRENT_SOURCE_BATTERY;
			have_electrical = true;
		}

		if (have_electrical && PX4_ISFINITE(i_meas_a) && PX4_ISFINITE(v_meas_v)) {
			p_meas_w = i_meas_a * v_meas_v;
		}

		const bool current_limit_active = (_sc_i_limit_a > FLT_EPSILON) && have_electrical
						  && PX4_ISFINITE(i_meas_a) && (i_meas_a > _sc_i_limit_a);

		if (current_limit_active) {
			saturation_flags |= FLAG_CURRENT_LIMIT;
		}

		if (status.arming_state != vehicle_status_s::ARMING_STATE_ARMED) {
			// reset PID when not controlling
			_integral = 0.f;
			_prev_error = 0.f;
			_last_u_out = NAN;

		} else {
			// Reference throttle from the normal flight stack (actuator_motors instance 0).
			// This allows using frequency control in both manual and auto modes without disabling the allocator.
			actuator_motors_s motors_ref{};
			_actuator_motors_sub.copy(&motors_ref);

			const float u_ref_in = motors_ref.control[0];
			u_ref = u_ref_in;

			if (PX4_ISFINITE(u_ref)) {
				u_ref = math::constrain(u_ref, 0.f, 1.f);
			} else {
				u_ref = NAN;
			}

			if (!PX4_ISFINITE(u_ref)) {
				// No valid reference command: don't override
				u_out = NAN;
				_integral = 0.f;
				_prev_error = 0.f;
				_last_u_out = NAN;

			} else if (!PX4_ISFINITE(rpm.rpm_estimate)) {
				// No valid feedback: don't override
				u_out = NAN;
				_integral = 0.f;
				_prev_error = 0.f;
				_last_u_out = NAN;
				saturation_flags |= FLAG_FEEDBACK_INVALID;

			} else {
				// Use u_ref as thrust demand proxy and map to desired flapping frequency.
				f_eff_sp = _flap_f_min + u_ref * (_flap_f_max - _flap_f_min);
				float f_sp = f_eff_sp;

				// Piecewise frequency modulation: upper/lower half-cycle use different frequencies,
				// while keeping the overall period constant.
				if (_fm_mode == 2) {
					if (phase_valid && PX4_ISFINITE(f_sp)) {
						float delta = _fm_delta_hz;

						// Ensure delta is feasible (f_sp - 2*delta > 0).
						if (f_sp <= 2.f * delta) {
							delta = 0.49f * f_sp;
						}

						const float denom = f_sp - 2.f * delta;

						if (denom > 0.001f) {
							const float sigma = f_sp * delta / denom;
							const float phase_deg = wrap360(_wing_phase.phase_deg + _phase_ff_shift_deg);
							const bool upper_half = (phase_deg < 180.f);
							f_sp = upper_half ? (f_sp - delta) : (f_sp + sigma);
						}
					}

				} else if (_fm_mode == 3) {
					const float delta_step = _sc_delta_slew * dt;
					_sc_delta_slewed += math::constrain(_sc_delta_cmd - _sc_delta_slewed, -delta_step, delta_step);
					_sc_delta_slewed = math::constrain(_sc_delta_slewed, 0.1f, 0.9f);

					const float delta_target = current_limit_active ? 0.5f : _sc_delta_slewed;
					const float alpha_delta = math::constrain(dt / _sc_recover_tau_s, 0.f, 1.f);
					_sc_delta_applied += (delta_target - _sc_delta_applied) * alpha_delta;
					_sc_delta_applied = math::constrain(_sc_delta_applied, 0.1f, 0.9f);

					if (current_limit_active && fabsf(_sc_delta_applied - _sc_delta_slewed) > 0.005f) {
						saturation_flags |= FLAG_DELTA_REDUCED;
					}

					if (phase_valid && PX4_ISFINITE(f_eff_sp)) {
						const float f_down = f_eff_sp / (2.f * _sc_delta_applied);
						const float f_up = f_eff_sp / (2.f * (1.f - _sc_delta_applied));
						const float w_down = downstroke_weight(phase_deg_used, _sc_blend_deg);
						const float f_low = f_eff_sp / _sc_fmax_mult;
						const float f_high = f_eff_sp * _sc_fmax_mult;
						const bool phase_in_downstroke = (phase_deg_used > 90.f && phase_deg_used < 270.f);

						f_sp = math::constrain(f_up + (f_down - f_up) * w_down, f_low, f_high);
						f_up_cmd_hz = f_up;
						f_down_cmd_hz = f_down;
						in_downstroke = phase_in_downstroke;
						wave_meas = cosf(phase_deg_used * (M_PI_F / 180.f));

						if (!PX4_ISFINITE(_phase_ref_deg)) {
							_phase_ref_deg = phase_deg_used;

						} else {
							_phase_ref_deg = wrap360(_phase_ref_deg + 360.f * f_sp * dt);
						}

						phase_ref_deg = _phase_ref_deg;
						phase_err_deg = wrap180(_phase_ref_deg - phase_deg_used);
						wave_cmd = cosf(phase_ref_deg * (M_PI_F / 180.f));

						if (_sc_phase_k_hz_per_deg > FLT_EPSILON) {
							f_sp += _sc_phase_k_hz_per_deg * phase_err_deg;
							f_sp = math::constrain(f_sp, f_low, f_high);
						}

					} else {
						_phase_ref_deg = NAN;
						f_sp = f_eff_sp;
					}
				}

				if (_fm_mode != 3) {
					_phase_ref_deg = NAN;
				}

				f_inst_sp = f_sp;
				rpm_sp = f_sp * _flap_ratio * 60.f;
				rpm_meas = rpm.rpm_estimate;
				omega_ref = f_sp * (2.f * M_PI_F);
				omega_meas = rpm_meas * (2.f * M_PI_F / 60.f);

				const float error = rpm_sp - rpm_meas;

				const float d = (error - _prev_error) / dt;
				_prev_error = error;

				// integral term (limit in RPM error-integral units)
				const float integral_before = _integral;
				_integral += error * dt;
				_integral = math::constrain(_integral, -_i_max, _i_max);

				// Inner-loop correction around the normal flight stack output
				const float delta_u = _kp * error + _ki * _integral + _kd * d;
				u_pid = delta_u;
				const float u_unsat = u_ref + delta_u;
				u_out = math::constrain(u_unsat, 0.f, 1.f);

				if ((u_unsat > 1.f && error > 0.f) || (u_unsat < 0.f && error < 0.f)) {
					_integral = integral_before;
				}

				// Phase-based shaping feedforward: smooth cosine modulation (downstroke 90..270 positive).
				// This is applied on the motor command (not on the setpoint) to bias within-cycle torque
				// while keeping the average reference from the flight stack.
				if (_fm_mode == 1 && _phase_ff_en && (_phase_ff_amp > 0.f)) {
					if (phase_valid) {
						const float deg = wrap360(_wing_phase.phase_deg + _phase_ff_shift_deg);
						const float rad = (deg - 180.f) * (M_PI_F / 180.f);
						float u_phase = _phase_ff_amp * cosf(rad);

						// Asymmetric up/downstroke shaping via duty ratio.
						const float down_scale = math::constrain(_phase_ff_duty / 0.5f, 0.f, 2.f);
						const float up_scale = math::constrain((1.f - _phase_ff_duty) / 0.5f, 0.f, 2.f);

						if (u_phase >= 0.f) {
							u_phase *= down_scale;

						} else {
							u_phase *= up_scale;
						}

						// Remove DC bias introduced by asymmetric scaling to keep mean ~0.
						const float bias = _phase_ff_amp * (down_scale - up_scale) / M_PI_F;
						u_phase -= bias;

						u_out = math::constrain(u_out + u_phase, 0.f, 1.f);
					}
				}

				if (_fm_mode == 3 && _sc_u_slew > FLT_EPSILON && PX4_ISFINITE(_last_u_out)) {
					const float max_du = _sc_u_slew * dt;
					u_out = math::constrain(u_out, _last_u_out - max_du, _last_u_out + max_du);
				}

				if (u_out <= 0.001f) {
					saturation_flags |= FLAG_OUT_SAT_LOW;
				}

				if (u_out >= 0.999f) {
					saturation_flags |= FLAG_OUT_SAT_HIGH;
				}

				_last_u_out = u_out;
			}
		}

		if (!PX4_ISFINITE(u_out)) {
			_last_u_out = NAN;
		}

		flap_motor_setpoint_s out{};
		out.timestamp = now;
		out.timestamp_sample = rpm.timestamp;
		out.thrust = u_out;
		_flap_motor_setpoint_pub.publish(out);

		flap_control_status_s control_status{};
		control_status.timestamp = now;
		control_status.timestamp_sample = rpm.timestamp;
		control_status.mode = static_cast<uint8_t>(_fm_mode);
		control_status.f_eff_sp_hz = f_eff_sp;
		control_status.f_inst_sp_hz = f_inst_sp;
		control_status.rpm_sp = rpm_sp;
		control_status.rpm_meas = rpm_meas;
		control_status.omega_ref_rad_s = omega_ref;
		control_status.omega_meas_rad_s = omega_meas;
		control_status.phase_deg = phase_deg_used;
		control_status.phase_ref_deg = phase_ref_deg;
		control_status.phase_err_deg = phase_err_deg;
		control_status.delta_cmd = _sc_delta_cmd;
		control_status.delta_slewed = _sc_delta_slewed;
		control_status.delta_applied = _sc_delta_applied;
		control_status.delta_meas = _delta_meas;
		control_status.f_up_cmd_hz = f_up_cmd_hz;
		control_status.f_down_cmd_hz = f_down_cmd_hz;
		control_status.in_downstroke = in_downstroke;
		control_status.wave_cmd = wave_cmd;
		control_status.wave_meas = wave_meas;
		control_status.u_ref = u_ref;
		control_status.u_pid = u_pid;
		control_status.u_out = u_out;
		control_status.current_source = current_source;
		control_status.i_meas_a = i_meas_a;
		control_status.v_meas_v = v_meas_v;
		control_status.p_meas_w = p_meas_w;
		control_status.i_limit_a = _sc_i_limit_a;
		control_status.saturation_flags = saturation_flags;
		_flap_control_status_pub.publish(control_status);
	}
}

int RpmPid::task_spawn(int argc, char *argv[])
{
	RpmPid *instance = new RpmPid();

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

int RpmPid::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int RpmPid::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
PID controller that closes the loop from flapping frequency (commanded by throttle)
to motor RPM measured by the AS5600 encoder.

NOTE: this controller is experimental and has not yet been validated on the actual flapping-wing hardware.

- The controller publishes flap_motor_setpoint.thrust (0..1) and does not directly drive PWM outputs.
  Use a mixer/output function to select between the normal Motor1 output and flap_motor_setpoint.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("rpm_pid", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int rpm_pid_main(int argc, char *argv[])
{
	return RpmPid::main(argc, argv);
}
