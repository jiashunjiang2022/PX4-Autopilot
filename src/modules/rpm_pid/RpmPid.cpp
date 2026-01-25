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

	// basic sanity
	if (_flap_f_max < _flap_f_min) {
		_flap_f_max = _flap_f_min;
	}

	_phase_ff_amp = math::constrain(_phase_ff_amp, 0.f, 0.2f);
	_phase_ff_duty = math::constrain(_phase_ff_duty, 0.1f, 0.9f);
	_phase_ff_shift_deg = math::constrain(_phase_ff_shift_deg, -180.f, 180.f);
	_fm_mode = math::constrain(_fm_mode, 0, 2);
	_fm_delta_hz = math::constrain(_fm_delta_hz, 0.f, 10.f);
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

		if (_last_run == 0) {
			_last_run = now;
			_integral = 0.f;
			_prev_error = 0.f;
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

		if (status.arming_state != vehicle_status_s::ARMING_STATE_ARMED) {
			// reset PID when not controlling
			_integral = 0.f;
			_prev_error = 0.f;

		} else {
			// Reference throttle from the normal flight stack (actuator_motors instance 0).
			// This allows using frequency control in both manual and auto modes without disabling the allocator.
			actuator_motors_s motors_ref{};
			_actuator_motors_sub.copy(&motors_ref);

			const float u_ref_in = motors_ref.control[0];
			float u_ref = u_ref_in;

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

			} else if (!PX4_ISFINITE(rpm.rpm_estimate)) {
				// No valid feedback: don't override
				u_out = NAN;
				_integral = 0.f;
				_prev_error = 0.f;

			} else {
				// Use u_ref as thrust demand proxy and map to desired flapping frequency.
				float f_sp = _flap_f_min + u_ref * (_flap_f_max - _flap_f_min);

				// Piecewise frequency modulation: upper/lower half-cycle use different frequencies,
				// while keeping the overall period constant.
				if (_fm_mode == 2) {
					const bool phase_valid = (now - _wing_phase.timestamp) < 200000 && PX4_ISFINITE(_wing_phase.phase_deg);

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
				}

				const float rpm_sp = f_sp * _flap_ratio * 60.f;
				const float rpm_meas = rpm.rpm_estimate;

				const float error = rpm_sp - rpm_meas;

				const float d = (error - _prev_error) / dt;
				_prev_error = error;

				// integral term (limit in RPM error-integral units)
				_integral += error * dt;
				_integral = math::constrain(_integral, -_i_max, _i_max);

				// Inner-loop correction around the normal flight stack output
				const float delta_u = _kp * error + _ki * _integral + _kd * d;
				u_out = math::constrain(u_ref + delta_u, 0.f, 1.f);

				// Phase-based shaping feedforward: smooth cosine modulation (downstroke 90..270 positive).
				// This is applied on the motor command (not on the setpoint) to bias within-cycle torque
				// while keeping the average reference from the flight stack.
				if (_fm_mode == 1 && _phase_ff_en && (_phase_ff_amp > 0.f)) {
					const bool phase_valid = (now - _wing_phase.timestamp) < 200000 && PX4_ISFINITE(_wing_phase.phase_deg);

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
			}
		}

		flap_motor_setpoint_s out{};
		out.timestamp = now;
		out.timestamp_sample = rpm.timestamp;
		out.thrust = u_out;
		_flap_motor_setpoint_pub.publish(out);
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
