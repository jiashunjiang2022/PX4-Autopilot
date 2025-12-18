/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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

#pragma once

#include "FunctionMotors.hpp"
#include "FunctionProviderBase.hpp"

#include <drivers/drv_hrt.h>
#include <lib/mathlib/mathlib.h>
#include <parameters/param.h>
#include <uORB/topics/flap_motor_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/wing_phase.h>

/**
 * Function: Flap_Motor1
 *
 * Selects the motor command source at runtime:
 * - aux1 low  -> normal Motor1 from actuator_motors.control[0]
 * - aux1 high -> flap_motor_setpoint.thrust
 *
 * Additionally supports "glide stop": when enabled via FLAP_GLIDE_CH (recommended: AUX2) and RC throttle is low,
 * keep spinning until wing_phase reaches 0 or 180 deg (nearest), then stop (output disarmed PWM).
 *
 * Both inputs are expected in normalized thrust units [0, 1] (NaN = disarmed).
 */
class FunctionFlapMotor : public FunctionProviderBase
{
public:
	FunctionFlapMotor(const Context &context) :
		_actuator_motors0(&context.work_item, ORB_ID(actuator_motors)),
		_thrust_factor(context.thrust_factor)
	{
		for (int i = 0; i < actuator_motors_s::NUM_CONTROLS; ++i) {
			_motors0.control[i] = NAN;
		}
		_fused_value = NAN;
		_latest_sample_timestamp = 0;

		// Parameter handles (global param system)
		_param_glide_thr_h = param_find("FLAP_GLIDE_THR");
		_param_glide_hold_h = param_find("FLAP_GLIDE_HOLD");
		_param_glide_tol_h = param_find("FLAP_GLIDE_TOL");
		_param_glide_to_h = param_find("FLAP_GLIDE_TO");
		_param_glide_ch_h = param_find("FLAP_GLIDE_CH");

		updateParams();
	}

	static FunctionProviderBase *allocate(const Context &context) { return new FunctionFlapMotor(context); }

	void update() override
	{
		if (_parameter_update_sub.updated()) {
			parameter_update_s p{};
			_parameter_update_sub.copy(&p);
			updateParams();
		}

		// Selection input
		_manual_control_sub.update(&_manual_control_setpoint);

		// Normal motor command source
		_actuator_motors0.update(&_motors0);

		// Flap RPM/frequency controller output
		_flap_motor_setpoint_sub.update(&_flap_motor_setpoint);

		// Wing phase for glide stop
		_wing_phase_sub.update(&_wing_phase);

		const hrt_abstime now = hrt_absolute_time();
		const bool freq_selected = _manual_control_setpoint.valid && (_manual_control_setpoint.aux1 > 0.5f);

		float selected_thrust = _motors0.control[0];
		uint32_t reversible_flags = _motors0.reversible_flags;
		_latest_sample_timestamp = _motors0.timestamp_sample;

		const bool flap_fresh = (now - _flap_motor_setpoint.timestamp) < 200000; // 200 ms

		if (freq_selected && flap_fresh && PX4_ISFINITE(_flap_motor_setpoint.thrust)) {
			selected_thrust = _flap_motor_setpoint.thrust;
			reversible_flags = 0;
			_latest_sample_timestamp = _flap_motor_setpoint.timestamp_sample;
		}

		// Glide stop state machine. Applies regardless of frequency mode selection.
		applyGlideStop(now, selected_thrust);

		if (!PX4_ISFINITE(selected_thrust)) {
			_latest_sample_timestamp = now;
		}

		float values[1] {selected_thrust};
		FunctionMotors::updateValues(reversible_flags, _thrust_factor, values, 1);
		_fused_value = values[0];
	}

	float value(OutputFunction func) override
	{
		if (func == OutputFunction::Flap_Motor1) {
			return _fused_value;
		}

		return NAN;
	}

	uORB::SubscriptionCallbackWorkItem *subscriptionCallback() override { return &_actuator_motors0; }

	bool getLatestSampleTimestamp(hrt_abstime &t) const override { t = _latest_sample_timestamp; return t != 0; }

private:
	enum class GlideState {
		Idle,
		Waiting,
		Stopped
	};

	void updateParams()
	{
		// Use sane defaults even if the param doesn't exist for some reason.
		float glide_thr = 0.03f;
		float glide_hold = 0.05f;
		float glide_tol = 5.f;
		float glide_to = 1.f;
		int32_t glide_ch = 0;

		if (_param_glide_thr_h != PARAM_INVALID) { (void)param_get(_param_glide_thr_h, &glide_thr); }
		if (_param_glide_hold_h != PARAM_INVALID) { (void)param_get(_param_glide_hold_h, &glide_hold); }
		if (_param_glide_tol_h != PARAM_INVALID) { (void)param_get(_param_glide_tol_h, &glide_tol); }
		if (_param_glide_to_h != PARAM_INVALID) { (void)param_get(_param_glide_to_h, &glide_to); }
		if (_param_glide_ch_h != PARAM_INVALID) { (void)param_get(_param_glide_ch_h, &glide_ch); }

		_glide_thr = math::constrain(glide_thr, 0.f, 0.2f);
		_glide_hold = math::constrain(glide_hold, 0.f, 0.5f);
		_glide_tol_deg = math::constrain(glide_tol, 1.f, 20.f);
		_glide_timeout_s = math::constrain(glide_to, 0.1f, 5.f);
		_glide_ch = math::constrain(glide_ch, (int32_t)0, (int32_t)6);
	}

	static inline float auxValueFromChannel(const manual_control_setpoint_s &mc, int ch)
	{
		switch (ch) {
		case 1: return mc.aux1;
		case 2: return mc.aux2;
		case 3: return mc.aux3;
		case 4: return mc.aux4;
		case 5: return mc.aux5;
		case 6: return mc.aux6;
		default: return NAN;
		}
	}

	bool glideSwitchOn() const
	{
		if (_glide_ch == 0) {
			return false;
		}

		if (!_manual_control_setpoint.valid) {
			return false;
		}

		const float v = auxValueFromChannel(_manual_control_setpoint, _glide_ch);
		return PX4_ISFINITE(v) && (v > 0.5f);
	}

	bool rcThrottleLow() const
	{
		if (!_manual_control_setpoint.valid || !PX4_ISFINITE(_manual_control_setpoint.throttle)) {
			return false;
		}

		// manual_control_setpoint.throttle is [-1, 1]. Convert to [0, 1].
		const float throttle_norm = math::constrain((_manual_control_setpoint.throttle + 1.f) * 0.5f, 0.f, 1.f);
		return throttle_norm <= _glide_thr;
	}

	static inline float wrapDeg360(float deg)
	{
		float out = fmodf(deg, 360.f);
		if (out < 0.f) {
			out += 360.f;
		}
		return out;
	}

	static inline float angularDistanceDeg(float a_deg, float b_deg)
	{
		const float a = wrapDeg360(a_deg);
		const float b = wrapDeg360(b_deg);
		float d = fabsf(a - b);
		if (d > 180.f) {
			d = 360.f - d;
		}
		return d;
	}

	void selectGlideTargetIfNeeded(const hrt_abstime now)
	{
		if (_glide_target_set) {
			return;
		}

		const bool phase_valid = (now - _wing_phase.timestamp) < 200000 && PX4_ISFINITE(_wing_phase.phase_deg);

		if (!phase_valid) {
			return;
		}

		const float deg = wrapDeg360(_wing_phase.phase_deg);
		const float dist0 = angularDistanceDeg(deg, 0.f);
		const float dist180 = angularDistanceDeg(deg, 180.f);
		_glide_target_deg = (dist0 <= dist180) ? 0.f : 180.f;
		_glide_target_set = true;
	}

	void applyGlideStop(const hrt_abstime now, float &selected_thrust)
	{
		const bool glide_on = glideSwitchOn();
		const bool throttle_low = rcThrottleLow();

		// Glide logic is only active while glide switch is ON and RC throttle is low.
		// Any throttle > threshold immediately exits the glide state machine, allowing repeated glide cycles.
		if (!glide_on || !throttle_low) {
			_glide_state = GlideState::Idle;
			_glide_target_set = false;
			return;
		}

		const bool phase_valid = (now - _wing_phase.timestamp) < 200000 && PX4_ISFINITE(_wing_phase.phase_deg);

		switch (_glide_state) {
		case GlideState::Idle:
			_glide_state = GlideState::Waiting;
			_glide_start = now;
			_glide_target_set = false;

			// Keep a minimum command while waiting. If upstream is NaN, use 0.
			{
				const float base = PX4_ISFINITE(selected_thrust) ? selected_thrust : 0.f;
				_glide_hold_effective = math::max(base, _glide_hold);
			}

			selectGlideTargetIfNeeded(now);
			break;

		case GlideState::Waiting: {
				selectGlideTargetIfNeeded(now);

				bool done = false;

				if (phase_valid && _glide_target_set) {
					const float diff = angularDistanceDeg(_wing_phase.phase_deg, _glide_target_deg);
					if (diff <= _glide_tol_deg) {
						done = true;
					}
				}

				if (!done && _glide_timeout_s > 0.f) {
					if ((now - _glide_start) > (hrt_abstime)(_glide_timeout_s * 1e6f)) {
						done = true;
					}
				}

				if (done) {
					_glide_state = GlideState::Stopped;
					// Stop motor: output disarmed PWM (MixingOutput maps NaN -> PWM_MAIN_DIS1).
					selected_thrust = NAN;

				} else {
					// Keep spinning until target is reached.
					const float base = PX4_ISFINITE(selected_thrust) ? selected_thrust : 0.f;
					selected_thrust = math::max(base, _glide_hold_effective);
				}
			}
			break;

		case GlideState::Stopped:
			// Keep stopped as long as glide switch remains on and throttle remains low.
			selected_thrust = NAN;
			break;
		}
	}

	uORB::SubscriptionCallbackWorkItem _actuator_motors0;
	actuator_motors_s _motors0{};

	uORB::Subscription _flap_motor_setpoint_sub{ORB_ID(flap_motor_setpoint)};
	flap_motor_setpoint_s _flap_motor_setpoint{};

	uORB::Subscription _manual_control_sub{ORB_ID(manual_control_setpoint)};
	manual_control_setpoint_s _manual_control_setpoint{};

	uORB::Subscription _wing_phase_sub{ORB_ID(wing_phase)};
	wing_phase_s _wing_phase{};

	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};

	const float &_thrust_factor;

	// Glide stop state
	GlideState _glide_state{GlideState::Idle};
	hrt_abstime _glide_start{0};
	float _glide_target_deg{0.f};
	bool _glide_target_set{false};
	float _glide_hold_effective{0.f};

	// Glide parameters (cached)
	float _glide_thr{0.03f};
	float _glide_hold{0.05f};
	float _glide_tol_deg{5.f};
	float _glide_timeout_s{1.f};
	int _glide_ch{0};

	// Parameter handles
	param_t _param_glide_thr_h{PARAM_INVALID};
	param_t _param_glide_hold_h{PARAM_INVALID};
	param_t _param_glide_tol_h{PARAM_INVALID};
	param_t _param_glide_to_h{PARAM_INVALID};
	param_t _param_glide_ch_h{PARAM_INVALID};

	float _fused_value{NAN};
	hrt_abstime _latest_sample_timestamp{0};
};
