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
 * @file HallGPIO.cpp
 *
 * Simple EXTI-based Hall sensor pulse capture.
 *
 * - repurposes the selected PWM pin as GPIO input with interrupt
 * - each rising edge is one pulse; compute dt -> rpm and publish uORB rpm
 * - default pulses-per-rev = 1 (one pulse per mechanical revolution)
 */

#include <board_config.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <drivers/drv_hrt.h>
#include <inttypes.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/hall_event.h>

#include <px4_platform_common/atomic.h>
#include <px4_arch/io_timer.h>
#include <px4_arch/micro_hal.h>

class HallGPIO : public ModuleBase<HallGPIO>, public px4::ScheduledWorkItem
{
public:
	HallGPIO();
	~HallGPIO() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	int print_status() override;

	bool init(uint32_t pulses_per_rev);

private:
	void Run() override;
	static int gpio_interrupt(int irq, void *context, void *arg);

	uint32_t _pulses_per_rev{1};
	px4::atomic<hrt_abstime> _last_ts{0};
	px4::atomic<uint32_t> _pulse_count{0};
	bool _last_state{false};

	uORB::Publication<hall_event_s> _hall_pub{ORB_ID(hall_event)};
};

HallGPIO::HallGPIO() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{}

HallGPIO::~HallGPIO()
{
	px4_arch_gpiosetevent(GPIO_HALL_IN, false, false, false, nullptr, nullptr);
}

bool HallGPIO::init(uint32_t pulses_per_rev)
{
	_pulses_per_rev = pulses_per_rev > 0 ? pulses_per_rev : 1;

	px4_arch_configgpio(GPIO_HALL_IN);
	const bool level = px4_arch_gpioread(GPIO_HALL_IN);
	PX4_INFO("GPIO_HALL_IN configured, initial level=%d, pulses_per_rev=%" PRIu32, level ? 1 : 0, _pulses_per_rev);

	// trigger on both edges to be robust to idle level
	const int ret = px4_arch_gpiosetevent(GPIO_HALL_IN, true, true, true, &HallGPIO::gpio_interrupt, this);

	if (ret != OK) {
		PX4_ERR("gpio event register failed (%d)", ret);
		return false;
	}

	PX4_INFO("EXTI registered on GPIO_HALL_IN");

	_last_state = px4_arch_gpioread(GPIO_HALL_IN);
	ScheduleOnInterval(5000); // 5 ms polling fallback
	return true;
}

int HallGPIO::gpio_interrupt(int, void *, void *arg)
{
	auto *inst = static_cast<HallGPIO *>(arg);

	const hrt_abstime now = hrt_absolute_time();
	inst->_last_ts.store(now);
	inst->_pulse_count.fetch_add(1);
	inst->ScheduleNow();

	return 0;
}

void HallGPIO::Run()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	// Polling edge detection (also covers the case where EXTI missed)
	const bool state = px4_arch_gpioread(GPIO_HALL_IN);

	if (!_last_state && state) {
		const hrt_abstime now = hrt_absolute_time();
		_last_ts.store(now);
		_pulse_count.fetch_add(1);
	}

	_last_state = state;

	const hrt_abstime now = _last_ts.load();

	if (now == 0) {
		return;
	}

	// Publish hall event
	hall_event_s msg{};
	msg.timestamp = now;
	msg.pulse_count = _pulse_count.load();
	_hall_pub.publish(msg);
}

int HallGPIO::task_spawn(int argc, char *argv[])
{
	int pulses_per_rev = 1;
	int myoptind = 1;
	int ch;

	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "p:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'p':
			pulses_per_rev = atoi(myoptarg);
			break;
		}
	}

	auto *instance = new HallGPIO;

	if (!instance) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;

	if (instance->init(pulses_per_rev)) {
		return PX4_OK;
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;
	return PX4_ERROR;
}

int HallGPIO::custom_command(int, char *[])
{
	return print_usage("unknown command");
}

int HallGPIO::print_status()
{
	PX4_INFO("pulses: %" PRIu32, _pulse_count.load());
	const hrt_abstime last = _last_ts.load();
	PX4_INFO("last_ts: %" PRIu64 " us", static_cast<uint64_t>(last));
	return 0;
}

int HallGPIO::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
Hall sensor using GPIO EXTI on PWM9 (PI6 / M9.2).

- repurposes PWM9 as input with interrupt
- each rising edge = one pulse
- publishes uORB rpm (rpm_raw, rpm_estimate)

Usage:
  hall_gpio start [-p <pulses_per_rev>]
  hall_gpio stop
  hall_gpio status
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("hall_gpio", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_INT('p', 1, 1, 100, "Pulses per mechanical revolution", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int hall_gpio_main(int argc, char *argv[])
{
	return HallGPIO::main(argc, argv);
}
