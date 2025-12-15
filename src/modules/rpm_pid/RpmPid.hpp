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

/**
 * @file RpmPid.hpp
 *
 * Flapping wing RPM / frequency controller.
 *
 * - Uses AS5600 driver RPM estimate as feedback
 * - Throttle stick sets desired flapping frequency (Hz)
 * - Gear ratio parameter converts flapping frequency to motor RPM setpoint
 * - Uses actuator_motors.control[0] (normal flight stack) as the reference command
 * - Publishes flap_motor_setpoint.thrust (0..1, NaN=invalid). A mixer/output function selects whether to use it.
 */

#pragma once

#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/flap_motor_setpoint.h>
#include <uORB/topics/wing_phase.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/rpm.h>
#include <uORB/topics/vehicle_status.h>

using namespace time_literals;

class RpmPid : public ModuleBase<RpmPid>, public ModuleParams, public px4::WorkItem
{
public:
	RpmPid();
	~RpmPid() override = default;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;
	void updateParams() override;

	uORB::SubscriptionCallbackWorkItem _rpm_sub{this, ORB_ID(rpm)};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _actuator_motors_sub{ORB_ID(actuator_motors)};
	uORB::Subscription _wing_phase_sub{ORB_ID(wing_phase)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Publication<flap_motor_setpoint_s> _flap_motor_setpoint_pub{ORB_ID(flap_motor_setpoint)};

	hrt_abstime _last_run{0};

	float _integral{0.f};
	float _prev_error{0.f};
	float _last_phase_deg{NAN};
	hrt_abstime _last_phase_ts{0};

	enum class GlideState {
		Idle,
		Waiting
	};

	GlideState _glide_state{GlideState::Idle};
	float _glide_target_deg{0.f};
	float _glide_hold{0.f};
	hrt_abstime _glide_start{0};

	// Cached params
	float _flap_f_min{0.f};
	float _flap_f_max{5.f};
	float _flap_ratio{7.5f};
	float _kp{0.001f};
	float _ki{0.f};
	float _kd{0.f};
	float _i_max{500.f};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::FLAP_F_MIN>) _param_flap_f_min,
		(ParamFloat<px4::params::FLAP_F_MAX>) _param_flap_f_max,
		(ParamFloat<px4::params::FLAP_RATIO>)  _param_flap_ratio,
		(ParamFloat<px4::params::FLAP_KP>)     _param_flap_kp,
		(ParamFloat<px4::params::FLAP_KI>)     _param_flap_ki,
		(ParamFloat<px4::params::FLAP_KD>)     _param_flap_kd,
		(ParamFloat<px4::params::FLAP_I_MAX>)  _param_flap_i_max,
		(ParamFloat<px4::params::FLAP_GLIDE_THR>)  _param_glide_thr,
		(ParamFloat<px4::params::FLAP_GLIDE_HOLD>) _param_glide_hold,
		(ParamFloat<px4::params::FLAP_GLIDE_TOL>)  _param_glide_tol,
		(ParamFloat<px4::params::FLAP_GLIDE_TO>)   _param_glide_to,
		(ParamInt<px4::params::FLAP_GLIDE_CH>)     _param_glide_ch
	)
};
