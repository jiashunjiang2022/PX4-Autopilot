/****************************************************************************
 *
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be used to
 *    endorse or promote products derived from this software without specific
 *    prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include <float.h>

#include <cmath>
#include <cstdint>

#include <uORB/topics/fixed_wing_longitudinal_setpoint.h>

namespace fw_control
{

inline bool isIdleLongitudinalSetpoint(const fixed_wing_longitudinal_setpoint_s &setpoint)
{
	const bool no_tecs_setpoint = !std::isfinite(setpoint.altitude)
				      && !std::isfinite(setpoint.height_rate)
				      && !std::isfinite(setpoint.equivalent_airspeed);
	const bool zero_direct_setpoint = std::isfinite(setpoint.pitch_direct)
					  && std::isfinite(setpoint.throttle_direct)
					  && std::fabs(setpoint.pitch_direct) <= FLT_EPSILON
					  && std::fabs(setpoint.throttle_direct) <= FLT_EPSILON;
	return no_tecs_setpoint && zero_direct_setpoint;
}

class AutoControlHandover
{
public:
	void updateControlMode(bool auto_enabled, uint64_t control_mode_timestamp)
	{
		if (auto_enabled && !_auto_enabled) {
			_pending = true;
			_activation_timestamp = control_mode_timestamp;

		} else if (!auto_enabled) {
			_pending = false;
			_activation_timestamp = 0;
		}

		_auto_enabled = auto_enabled;
	}

	bool acceptSetpoint(bool updated, const fixed_wing_longitudinal_setpoint_s &setpoint)
	{
		if (!updated) {
			return false;
		}

		if (_pending) {
			if (setpoint.timestamp < _activation_timestamp || isIdleLongitudinalSetpoint(setpoint)) {
				return false;
			}

			_pending = false;
		}

		return true;
	}

	bool pending() const { return _pending; }

private:
	uint64_t _activation_timestamp{0};
	bool _auto_enabled{false};
	bool _pending{false};
};

} // namespace fw_control
