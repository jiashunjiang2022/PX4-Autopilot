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
 * @file AS5600.hpp
 *
 * Driver for AS5600 I2C magnetic rotary encoder.
 */

#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <drivers/device/i2c.h>
#include <drivers/drv_hrt.h>
#include <mathlib/mathlib.h>
#include <parameters/param.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/encoder_count.h>
#include <uORB/topics/debug_vect.h>
#include <uORB/topics/flap_frequency.h>
#include <uORB/topics/rpm.h>
#include <uORB/topics/wing_phase.h>

/* Configuration Constants */
#define AS5600_I2C_ADDRESS_DEFAULT 0x36

class AS5600 : public device::I2C, public I2CSPIDriver<AS5600>
{
public:
	AS5600(const I2CSPIDriverConfig &config);
	~AS5600() override = default;

	static void print_usage();

	void RunImpl();

	int init() override;
	void print_status() override;

private:
	int probe() override;

	bool read_raw_angle(uint16_t &angle_raw);
	bool read_angle(float &angle_rad);
	void update_flap_ratio_param();

	hrt_abstime _last_read{0};
	float _last_angle_rad{0.f};
	float _rpm_estimate{0.f};
	float _flap_ratio{7.5f};
	hrt_abstime _last_param_update{0};
	param_t _param_flap_ratio_handle{PARAM_INVALID};
	uint16_t _last_pos{0};
	int64_t _total_count{0};
	bool _pos_initialized{false};

	uORB::Publication<debug_vect_s> _debug_pub{ORB_ID(debug_vect)};
	uORB::Publication<encoder_count_s> _encoder_pub{ORB_ID(encoder_count)};
	uORB::Publication<flap_frequency_s> _flap_frequency_pub{ORB_ID(flap_frequency)};
	uORB::PublicationMulti<rpm_s> _rpm_pub{ORB_ID(rpm)};
	uORB::Publication<wing_phase_s> _wing_phase_pub{ORB_ID(wing_phase)};
};
