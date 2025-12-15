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

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>

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

	// basic sanity
	if (_flap_f_max < _flap_f_min) {
		_flap_f_max = _flap_f_min;
	}

	// glide params bounds
	_param_glide_tol.set(math::constrain(_param_glide_tol.get(), 1.f, 20.f));
	_param_glide_thr.set(math::constrain(_param_glide_thr.get(), 0.f, 0.2f));
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
	wing_phase_s phase{};

	if (_wing_phase_sub.update(&phase)) {
		_last_phase_deg = phase.phase_deg;
		_last_phase_ts = phase.timestamp;
	}

	while (_rpm_sub.update(&rpm)) {
		const hrt_abstime now = hrt_absolute_time();

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

		manual_control_setpoint_s mc{};
		_manual_control_setpoint_sub.copy(&mc);

		vehicle_status_s status{};
		_vehicle_status_sub.copy(&status);

		// Default output: invalid (do not use), selection happens downstream
		float u_out = NAN;

		if (status.arming_state != vehicle_status_s::ARMING_STATE_ARMED) {
			// reset PID when not controlling
			_integral = 0.f;
			_prev_error = 0.f;
			_glide_state = GlideState::Idle;

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
				const float f_sp = _flap_f_min + u_ref * (_flap_f_max - _flap_f_min);
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
			}

			// If manual control is not valid, disable glide logic (and keep override behavior unchanged).
			if (!mc.valid) {
				_glide_state = GlideState::Idle;
				goto publish;
			}

			// glide stop logic
			const int ch = _param_glide_ch.get();
			auto aux_high = [](float v) { return fabsf(v) > 0.5f; };
			bool glide_enable = (ch == 0);

			if (ch == 1) glide_enable = aux_high(mc.aux1);
			else if (ch == 2) glide_enable = aux_high(mc.aux2);
			else if (ch == 3) glide_enable = aux_high(mc.aux3);
			else if (ch == 4) glide_enable = aux_high(mc.aux4);
			else if (ch == 5) glide_enable = aux_high(mc.aux5);
			else if (ch == 6) glide_enable = aux_high(mc.aux6);
			// ch 7-18: not mapped, remain disabled

			const bool phase_valid = (now - _last_phase_ts) < 200_ms && PX4_ISFINITE(_last_phase_deg);

			if (glide_enable && PX4_ISFINITE(u_ref_in) && u_ref_in <= _param_glide_thr.get() && phase_valid) {
				if (_glide_state == GlideState::Idle) {
					// pick nearest 0 or 180
					const float deg = _last_phase_deg;
					const float dist0 = fabsf(matrix::wrap_pi(math::radians(deg))); // to 0
					const float dist180 = fabsf(matrix::wrap_pi(math::radians(deg - 180.f))); // to 180
					_glide_target_deg = (dist0 <= dist180) ? 0.f : 180.f;
					_glide_hold = math::max(u_out, _param_glide_hold.get());
					_glide_start = now;
					_glide_state = GlideState::Waiting;
				}
			} else if (PX4_ISFINITE(u_ref_in) && u_ref_in > _param_glide_thr.get()) {
				_glide_state = GlideState::Idle;
			}

			if (_glide_state == GlideState::Waiting) {
				const float tol = _param_glide_tol.get();
				bool done = false;

				if (phase_valid) {
					float diff = fabsf(matrix::wrap_2pi(math::radians(_last_phase_deg - _glide_target_deg)));
					diff = math::degrees(diff);
					if (diff <= tol || diff >= 360.f - tol) {
						done = true;
					}
				}

				const float to_sec = _param_glide_to.get();

				if ((to_sec > 0.f) && (now - _glide_start) > to_sec * 1e6f) {
					done = true;
				}

				if (done) {
					u_out = 0.f;
					_glide_state = GlideState::Idle;

				} else {
					u_out = _glide_hold;
				}
			}
		}

publish:
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
