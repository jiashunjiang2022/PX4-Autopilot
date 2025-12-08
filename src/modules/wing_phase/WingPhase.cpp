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

/**
 * @file WingPhase.cpp
 *
 * Compute gear B phase (0..360 deg) from encoder count (gear A) + hall index (gear B zero).
 *
 * - Assumes encoder PPR = 4096 on gear A.
 * - Gear ratio R = FLAP_RATIO (default 7.5): A 转 R 圈 -> B 转 1 圈
 * - Hall index pulse zeroes the phase (reset offset to current total_count).
 * - phase_deg = ((total_count - reset_offset) % (PPR*R)) * 360 / (PPR*R)
 */

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/encoder_count.h>
#include <uORB/topics/hall_event.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/wing_phase.h>

using namespace time_literals;

class WingPhase : public ModuleBase<WingPhase>, public ModuleParams, public px4::WorkItem
{
public:
	WingPhase();
	~WingPhase() override = default;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;
	void updateParams() override;

	static constexpr int32_t kPpr = 4096; // encoder PPR on gear A

	uORB::SubscriptionCallbackWorkItem _encoder_sub{this, ORB_ID(encoder_count)};
	uORB::Subscription _hall_sub{ORB_ID(hall_event)};
	uORB::SubscriptionInterval _param_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Publication<wing_phase_s> _phase_pub{ORB_ID(wing_phase)};

	int64_t _reset_offset{0};
	uint32_t _hall_pulses{0};
	uint32_t _hall_pulses_prev{0};
	float _period_cnt{kPpr * 7.5f}; // PPR * gear ratio

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::FLAP_RATIO>) _param_flap_ratio
	)
};

WingPhase::WingPhase() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
}

bool WingPhase::init()
{
	updateParams();

	if (!_encoder_sub.registerCallback()) {
		PX4_ERR("encoder callback reg failed");
		return false;
	}

	return true;
}

void WingPhase::updateParams()
{
	ModuleParams::updateParams();
	_period_cnt = kPpr * _param_flap_ratio.get(); // counts per B revolution
}

void WingPhase::Run()
{
	if (should_exit()) {
		_encoder_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	// update params
	if (_param_update_sub.updated()) {
		parameter_update_s p{};
		_param_update_sub.copy(&p);
		updateParams();
	}

	// process hall events (reset offset)
	hall_event_s hall{};

	while (_hall_sub.update(&hall)) {
		_hall_pulses = hall.pulse_count;
	}

	encoder_count_s enc{};

	while (_encoder_sub.update(&enc)) {
		if (_period_cnt <= 0.f) {
			continue;
		}

		// on new hall pulse, reset offset to current encoder count modulo period
		if (_hall_pulses != _hall_pulses_prev) {
			_reset_offset = enc.total_count % static_cast<int64_t>(_period_cnt);
			_hall_pulses_prev = _hall_pulses;
		}

		int64_t reset = _reset_offset;
		const int64_t period = static_cast<int64_t>(_period_cnt);

		int64_t delta = (enc.total_count - reset) % period;

		if (delta < 0) {
			delta += period;
		}

		const float phase_deg = static_cast<float>(delta) * (360.f / _period_cnt);

		wing_phase_s msg{};
		msg.timestamp = enc.timestamp;
		msg.phase_deg = phase_deg;
		msg.total_count = enc.total_count;
		msg.hall_pulse_count = _hall_pulses;
		_phase_pub.publish(msg);
	}
}

int WingPhase::task_spawn(int argc, char *argv[])
{
	WingPhase *instance = new WingPhase();

	if (!instance) {
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

int WingPhase::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int WingPhase::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
Compute gear B phase using encoder counts (gear A) and hall index (gear B zero).

Assumes encoder PPR=4096; gear ratio from FLAP_RATIO (default 7.5): A 转 R 圈 -> B 转 1 圈.
Phase formula:
  delta = (total_count - reset_offset) mod (PPR*R)
  phase_deg = delta * 360 / (PPR*R)
Hall pulse resets reset_offset.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("wing_phase", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int wing_phase_main(int argc, char *argv[])
{
	return WingPhase::main(argc, argv);
}
