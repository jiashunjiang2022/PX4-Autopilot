/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * @file topic_listener.hpp
 *
 */

#pragma once

#include <px4_platform_common/defines.h>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/app.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <uORB/uORB.h>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/flap_motor_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/rpm.h>
#include <parameters/param.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <float.h>

static inline bool listener_try_copy_topic(const orb_id_t &id, int &sub, void *dst, size_t dst_size,
		uint32_t wait_timeout_us)
{
	(void)dst_size;

	if (sub < 0) {
		sub = orb_subscribe(id);
		if (sub < 0) {
			return false;
		}
	}

	// First try immediately
	if (orb_copy(id, sub, dst) == PX4_OK) {
		return true;
	}

	// If no sample available yet, wait briefly for the next update
	const hrt_abstime start = hrt_absolute_time();

	while ((hrt_absolute_time() - start) < wait_timeout_us) {
		bool updated = false;
		(void)orb_check(sub, &updated);

		if (updated) {
			if (orb_copy(id, sub, dst) == PX4_OK) {
				return true;
			}
		}

		px4_usleep(1000);
	}

	return false;
}

inline int listener_print_topic(const orb_id_t &orb_id, int subscription)
{
	static constexpr int max_size = 512;
	alignas(8) char container[max_size];

	if (orb_id->o_size > max_size) {
		PX4_ERR("topic %s too large (%i > %i)", orb_id->o_name, orb_id->o_size, max_size);
		return -1;
	}

	int ret = orb_copy(orb_id, subscription, &container);

	if (ret == PX4_OK) {
		orb_print_message_internal(orb_id, &container, true);

		// Derived fields (not part of uORB message): show flapping/gear frequency from motor RPM
		// using FLAP_RATIO (motor revs per flap).
		if (strcmp(orb_id->o_name, "rpm") == 0) {
			rpm_s rpm{};
			memcpy(&rpm, container, sizeof(rpm));

			static bool printed_selector_once = false;

			static param_t flap_ratio_handle = PARAM_INVALID;
			static param_t flap_f_min_handle = PARAM_INVALID;
			static param_t flap_f_max_handle = PARAM_INVALID;
			float flap_ratio = 0.f;
			float flap_f_min = 0.f;
			float flap_f_max = 0.f;

			if (flap_ratio_handle == PARAM_INVALID) {
				flap_ratio_handle = param_find("FLAP_RATIO");
			}

			if (flap_ratio_handle != PARAM_INVALID) {
				(void)param_get(flap_ratio_handle, &flap_ratio);
			}

			if (flap_f_min_handle == PARAM_INVALID) {
				flap_f_min_handle = param_find("FLAP_F_MIN");
			}

			if (flap_f_max_handle == PARAM_INVALID) {
				flap_f_max_handle = param_find("FLAP_F_MAX");
			}

			if (flap_f_min_handle != PARAM_INVALID) {
				(void)param_get(flap_f_min_handle, &flap_f_min);
			}

			if (flap_f_max_handle != PARAM_INVALID) {
				(void)param_get(flap_f_max_handle, &flap_f_max);
			}

			if (PX4_ISFINITE(rpm.rpm_estimate) && PX4_ISFINITE(flap_ratio) && (flap_ratio > FLT_EPSILON)) {
				const float frequency_hz = rpm.rpm_estimate / (flap_ratio * 60.f);
				PX4_INFO_RAW("frequency_hz: %.3f (rpm_estimate=%.3f, FLAP_RATIO=%.3f)\n",
					     (double)frequency_hz, (double)rpm.rpm_estimate, (double)flap_ratio);
			}

			// Show whether frequency PID override is effectively active and what the current target frequency is.
			// Only print this once per `listener rpm` run (avoid duplicating for multi-instance rpm).
			// "active" here matches the mixer logic: aux1 requests override and flap_motor_setpoint is fresh.
			if (printed_selector_once) {
				return ret;
			}

			printed_selector_once = true;

			static int manual_control_sub = -1;
			static int actuator_motors_sub = -1;
			static int flap_motor_setpoint_sub = -1;

			manual_control_setpoint_s manual{};
			const bool have_manual = listener_try_copy_topic(ORB_ID(manual_control_setpoint), manual_control_sub,
						      &manual, sizeof(manual), 50000);

			actuator_motors_s motors{};
			const bool have_motors = listener_try_copy_topic(ORB_ID(actuator_motors), actuator_motors_sub,
						      &motors, sizeof(motors), 50000);

			flap_motor_setpoint_s flap_sp{};
			const bool have_flap_sp = listener_try_copy_topic(ORB_ID(flap_motor_setpoint), flap_motor_setpoint_sub,
						       &flap_sp, sizeof(flap_sp), 50000);

			float aux1 = 0.f;
			bool aux1_request = false;

			if (have_manual) {
				aux1 = manual.aux1;
				aux1_request = PX4_ISFINITE(aux1) && (aux1 > 0.5f);
			}

			double flap_sp_age_ms = -1.0;
			bool flap_sp_fresh = false;

			if (have_flap_sp && (flap_sp.timestamp > 0)) {
				const hrt_abstime age_us = hrt_elapsed_time(&flap_sp.timestamp);
				flap_sp_age_ms = (double)age_us / 1000.0;
				flap_sp_fresh = (age_us < 200000); // 200 ms
			}

			const bool freq_pid_active = aux1_request && flap_sp_fresh;

			PX4_INFO_RAW("freq_pid_active: %d (aux1=%.3f request=%d, flap_sp_age_ms=%.1f fresh=%d)\n",
				     (int)freq_pid_active, (double)aux1, (int)aux1_request, flap_sp_age_ms, (int)flap_sp_fresh);

			if (have_motors && PX4_ISFINITE(motors.control[0])
			    && PX4_ISFINITE(flap_f_min) && PX4_ISFINITE(flap_f_max) && (flap_f_max >= flap_f_min)) {
				float u_ref = motors.control[0];

				if (u_ref < 0.f) { u_ref = 0.f; }

				if (u_ref > 1.f) { u_ref = 1.f; }

				const float target_hz = flap_f_min + u_ref * (flap_f_max - flap_f_min);
				PX4_INFO_RAW("target_hz: %.3f (u_ref=%.3f, FLAP_F_MIN=%.3f, FLAP_F_MAX=%.3f)\n",
					     (double)target_hz, (double)motors.control[0], (double)flap_f_min, (double)flap_f_max);
			}
		}
	}

	return ret;
}

void listener(const orb_id_t &id, unsigned num_msgs, int topic_instance,
	      unsigned topic_interval);
