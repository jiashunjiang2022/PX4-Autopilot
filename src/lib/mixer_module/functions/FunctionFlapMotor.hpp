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
#include <uORB/topics/flap_motor_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>

/**
 * Function: Flap_Motor1
 *
 * Selects the motor command source at runtime:
 * - aux1 low  -> normal Motor1 from actuator_motors.control[0]
 * - aux1 high -> flap_motor_setpoint.thrust
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
	}

	static FunctionProviderBase *allocate(const Context &context) { return new FunctionFlapMotor(context); }

	void update() override
	{
		// Selection input
		_manual_control_sub.update(&_manual_control_setpoint);

		// Normal motor command source
		_actuator_motors0.update(&_motors0);

		// Flap RPM/frequency controller output
		_flap_motor_setpoint_sub.update(&_flap_motor_setpoint);

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
	uORB::SubscriptionCallbackWorkItem _actuator_motors0;
	actuator_motors_s _motors0{};

	uORB::Subscription _flap_motor_setpoint_sub{ORB_ID(flap_motor_setpoint)};
	flap_motor_setpoint_s _flap_motor_setpoint{};

	uORB::Subscription _manual_control_sub{ORB_ID(manual_control_setpoint)};
	manual_control_setpoint_s _manual_control_setpoint{};

	const float &_thrust_factor;

	float _fused_value{NAN};
	hrt_abstime _latest_sample_timestamp{0};
};
