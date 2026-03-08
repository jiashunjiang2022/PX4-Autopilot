/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#include <drivers/drv_hrt.h>
#include <drivers/drv_sensor.h>
#include <lib/drivers/device/Device.hpp>
#include <math.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/uORB.h>
#include <uORB/topics/differential_pressure.h>
#include <uORB/topics/flap_frequency.h>
#include <uORB/topics/parameter_update.h>

using namespace time_literals;

class AirspeedInjector : public ModuleBase<AirspeedInjector>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	AirspeedInjector() :
		ModuleParams(nullptr),
		ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
	{
	}

	~AirspeedInjector() override
	{
		ScheduleClear();

		if (_diff_pressure_pub != nullptr) {
			orb_unadvertise(_diff_pressure_pub);
			_diff_pressure_pub = nullptr;
		}
	}

	static int task_spawn(int argc, char *argv[])
	{
#ifndef CONFIG_ARCH_BOARD_PX4_SITL
		PX4_ERR("SITL only");
		return PX4_ERROR;
#else
		AirspeedInjector *instance = new AirspeedInjector();

		if (instance == nullptr) {
			PX4_ERR("alloc failed");
			return PX4_ERROR;
		}

		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (!instance->init()) {
			delete instance;
			_object.store(nullptr);
			_task_id = -1;
			return PX4_ERROR;
		}

		return PX4_OK;
#endif
	}

	static int custom_command(int argc, char *argv[])
	{
		return print_usage("unknown command");
	}

	static int print_usage(const char *reason = nullptr)
	{
		if (reason != nullptr) {
			PX4_WARN("%s", reason);
		}

		PRINT_MODULE_DESCRIPTION(
			R"DESCR_STR(
### Description
SITL-only differential pressure injector for airspeed contamination experiments.
Publishes `differential_pressure` at 10 Hz with optional narrowband and spike disturbances.
)DESCR_STR");

		PRINT_MODULE_USAGE_NAME("airspeed_injector", "simulation");
		PRINT_MODULE_USAGE_COMMAND("start");
		PRINT_MODULE_USAGE_COMMAND("stop");
		PRINT_MODULE_USAGE_COMMAND("status");
		return 0;
	}

	int print_status() override
	{
		PX4_INFO("running: yes");
		PX4_INFO("published samples: %" PRIu32, _published_samples);
		PX4_INFO("last dp: %.3f Pa", (double)_last_dp_pa);
		PX4_INFO("EN=%d INST=%d BASE=%.3f FLAP_HZ=%.3f NB_AMP=%.3f SPIKE_AMP=%.3f SPIKE_PERIOD=%.3f",
			 _param_aspd_inj_en.get(),
			 _param_aspd_inj_inst.get(),
			 (double)_param_aspd_inj_base.get(),
			 (double)_param_aspd_inj_flap_hz.get(),
			 (double)_param_aspd_inj_nb_amp.get(),
			 (double)_param_aspd_inj_spk_amp.get(),
			 (double)_param_aspd_inj_spk_per.get());
		return 0;
	}

private:
	bool init()
	{
		// Run at 100 Hz so flap_frequency can be published at 100 Hz.
		ScheduleOnInterval(10_ms);
		return true;
	}

	void Run() override
	{
		if (should_exit()) {
			exit_and_cleanup();
			return;
		}

		if (_parameter_update_sub.updated()) {
			parameter_update_s update{};

			if (_parameter_update_sub.copy(&update)) {
				updateParams();

				if ((_diff_pressure_pub != nullptr) && (_param_aspd_inj_inst.get() != _pub_instance)) {
					orb_unadvertise(_diff_pressure_pub);
					_diff_pressure_pub = nullptr;
					_pub_instance = -1;
				}
			}
		}

		if (_param_aspd_inj_en.get() != 1) {
			return;
		}

		const hrt_abstime now = hrt_absolute_time();
		publish_flap_frequency(now);

		if (!ensure_publisher()) {
			return;
		}

		const float t_s = now * 1e-6f;
		const float base_pa = _param_aspd_inj_base.get();
		const float flap_hz = _param_aspd_inj_flap_hz.get();
		const float nb_amp = _param_aspd_inj_nb_amp.get();
		const float spike_amp = _param_aspd_inj_spk_amp.get();
		const float spike_period_s = _param_aspd_inj_spk_per.get();

		float dp_pa = base_pa;

		if (PX4_ISFINITE(nb_amp) && PX4_ISFINITE(flap_hz) && fabsf(nb_amp) > FLT_EPSILON && flap_hz > FLT_EPSILON) {
			dp_pa += nb_amp * sinf(2.f * M_PI_F * flap_hz * t_s);
		}

		if (PX4_ISFINITE(spike_amp) && PX4_ISFINITE(spike_period_s) && (spike_amp > 0.f) && (spike_period_s > FLT_EPSILON)) {
			const uint64_t spike_period_us = static_cast<uint64_t>(spike_period_s * 1e6f);

			if (spike_period_us > 0) {
				if ((_next_spike_time_us == 0) || (now < (_next_spike_time_us - spike_period_us))) {
					_next_spike_time_us = now + spike_period_us;
				}

				if (now >= _next_spike_time_us) {
					dp_pa += spike_amp;

					do {
						_next_spike_time_us += spike_period_us;

					} while (now >= _next_spike_time_us);
				}
			}

		} else {
			_next_spike_time_us = 0;
		}

		if (!PX4_ISFINITE(dp_pa)) {
			dp_pa = base_pa;
		}

		differential_pressure_s report{};
		report.timestamp_sample = now;
		report.timestamp = now;

		device::Device::DeviceId device_id{};
		device_id.devid_s.bus_type = device::Device::DeviceBusType_SIMULATION;
		device_id.devid_s.bus = 1;
		device_id.devid_s.address = 5;
		device_id.devid_s.devtype = DRV_DIFF_PRESS_DEVTYPE_SIM;
		report.device_id = device_id.devid;

		report.differential_pressure_pa = dp_pa;
		report.temperature = NAN;
		report.error_count = 0;

		// Keep differential_pressure at 10 Hz.
		if ((now - _last_dp_pub_us) >= 100_ms) {
			if (orb_publish(ORB_ID(differential_pressure), _diff_pressure_pub, &report) != PX4_OK) {
				return;
			}

			_last_dp_pub_us = now;
			_last_dp_pa = dp_pa;
			_published_samples++;
		}
	}

	bool ensure_publisher()
	{
		if (_diff_pressure_pub != nullptr) {
			return true;
		}

		int desired_instance = _param_aspd_inj_inst.get();

		if (desired_instance < 0) {
			desired_instance = 0;
		}
		int pub_instance = desired_instance;
		const int primary_exists = orb_exists(ORB_ID(differential_pressure), 0);

		if ((desired_instance > 0) && (primary_exists != PX4_OK)) {
			PX4_WARN("waiting for differential_pressure instance 0 before advertising instance %d", desired_instance);
			return false;
		}

		_diff_pressure_pub = orb_advertise_multi(ORB_ID(differential_pressure), nullptr, &pub_instance);

		if (_diff_pressure_pub == nullptr) {
			PX4_WARN("advertise failed");
			return false;
		}

		_pub_instance = pub_instance;

		if (_pub_instance != desired_instance) {
			PX4_WARN("got instance %d, expected %d; not publishing for safety", _pub_instance, desired_instance);
			orb_unadvertise(_diff_pressure_pub);
			_diff_pressure_pub = nullptr;
			_pub_instance = -1;
			return false;
		}

		return true;
	}

	void publish_flap_frequency(const hrt_abstime now)
	{
		flap_frequency_s flap_frequency{};
		flap_frequency.timestamp = now;
		flap_frequency.frequency_hz = _param_aspd_inj_flap_hz.get();
		flap_frequency.phase_rad = NAN;

		if (PX4_ISFINITE(flap_frequency.frequency_hz) && flap_frequency.frequency_hz > FLT_EPSILON) {
			float phase = 2.f * M_PI_F * flap_frequency.frequency_hz * (now * 1e-6f);
			phase = fmodf(phase, 2.f * M_PI_F);

			if (phase < 0.f) {
				phase += 2.f * M_PI_F;
			}

			flap_frequency.phase_rad = phase;
		}

		_flap_frequency_pub.publish(flap_frequency);
	}

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uint32_t _published_samples{0};
	float _last_dp_pa{NAN};
	uint64_t _next_spike_time_us{0};
	hrt_abstime _last_dp_pub_us{0};
	orb_advert_t _diff_pressure_pub{nullptr};
	int _pub_instance{-1};
	uORB::Publication<flap_frequency_s> _flap_frequency_pub{ORB_ID(flap_frequency)};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::ASPD_INJ_EN>) _param_aspd_inj_en,
		(ParamInt<px4::params::ASPD_INJ_INST>) _param_aspd_inj_inst,
		(ParamFloat<px4::params::ASPD_INJ_BASE>) _param_aspd_inj_base,
		(ParamFloat<px4::params::ASPD_INJ_FLAP_HZ>) _param_aspd_inj_flap_hz,
		(ParamFloat<px4::params::ASPD_INJ_NB_AMP>) _param_aspd_inj_nb_amp,
		(ParamFloat<px4::params::ASPD_INJ_SPK_AMP>) _param_aspd_inj_spk_amp,
		(ParamFloat<px4::params::ASPD_INJ_SPK_PER>) _param_aspd_inj_spk_per
	)
};

extern "C" __EXPORT int airspeed_injector_main(int argc, char *argv[])
{
	return AirspeedInjector::main(argc, argv);
}
