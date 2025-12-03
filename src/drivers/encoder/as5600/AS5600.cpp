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

#include "AS5600.hpp"

#include <px4_platform_common/module.h>

using namespace time_literals;

AS5600::AS5600(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config)
{
}

int AS5600::probe()
{
	uint16_t angle_raw{};

	if (read_raw_angle(angle_raw)) {
		return PX4_OK;
	}

	return PX4_ERROR;
}

bool AS5600::read_raw_angle(uint16_t &angle_raw)
{
	// RAW_ANGLE register: high byte at 0x0C, low byte at 0x0D
	uint8_t reg = 0x0C;
	uint8_t buf[2] {};

	const int ret = transfer(&reg, 1, buf, 2);

	if (ret != PX4_OK) {
		return false;
	}

	angle_raw = (static_cast<uint16_t>(buf[0]) << 8 | buf[1]) & 0x0FFF;
	return true;
}

bool AS5600::read_angle(float &angle_rad)
{
	uint16_t raw{};

	if (!read_raw_angle(raw)) {
		return false;
	}

	static constexpr float kScale = 2.f * M_PI_F / 4096.f;
	angle_rad = static_cast<float>(raw) * kScale;
	return true;
}

int AS5600::init()
{
	int ret = I2C::init();

	if (ret != PX4_OK) {
		return ret;
	}

	if (probe() != PX4_OK) {
		PX4_ERR("AS5600 not responding");
		return PX4_ERROR;
	}

	ScheduleOnInterval(10_ms); // 100 Hz
	return PX4_OK;
}

void AS5600::RunImpl()
{
	float angle_rad{};

	if (!read_angle(angle_rad)) {
		PX4_DEBUG("AS5600 read failed");
		return;
	}

	_last_angle_rad = angle_rad;
	_last_read = hrt_absolute_time();

	debug_vect_s msg{};
	msg.timestamp = _last_read;
	strncpy(msg.name, "AS5600ANG", sizeof(msg.name));
	msg.name[sizeof(msg.name) - 1] = '\0';
	msg.x = angle_rad;
	msg.y = 0.f;
	msg.z = 0.f;

	_debug_pub.publish(msg);
}

void AS5600::print_status()
{
	I2CSPIDriverBase::print_status();
	PX4_INFO("last angle: %.3f rad", static_cast<double>(_last_angle_rad));
}

