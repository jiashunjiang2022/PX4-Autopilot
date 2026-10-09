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

#include "FwControlHandover.hpp"

#include <gtest/gtest.h>

namespace
{

fixed_wing_longitudinal_setpoint_s idleSetpoint(uint64_t timestamp)
{
	fixed_wing_longitudinal_setpoint_s setpoint{};
	setpoint.timestamp = timestamp;
	setpoint.altitude = NAN;
	setpoint.height_rate = NAN;
	setpoint.equivalent_airspeed = NAN;
	setpoint.pitch_direct = 0.f;
	setpoint.throttle_direct = 0.f;
	return setpoint;
}

fixed_wing_longitudinal_setpoint_s missionSetpoint(uint64_t timestamp)
{
	fixed_wing_longitudinal_setpoint_s setpoint{};
	setpoint.timestamp = timestamp;
	setpoint.altitude = 39.36f;
	setpoint.height_rate = NAN;
	setpoint.equivalent_airspeed = 8.f;
	setpoint.pitch_direct = NAN;
	setpoint.throttle_direct = NAN;
	return setpoint;
}

} // namespace

TEST(FwControlHandover, WaitsForFreshNonIdleSetpointOnAutoEntry)
{
	fw_control::AutoControlHandover handover;
	handover.updateControlMode(false, 19'000'000);
	EXPECT_FALSE(handover.pending());

	handover.updateControlMode(true, 20'202'000);
	EXPECT_TRUE(handover.pending());
	EXPECT_FALSE(handover.acceptSetpoint(false, {}));
	EXPECT_FALSE(handover.acceptSetpoint(true, missionSetpoint(20'000'000)));
	EXPECT_FALSE(handover.acceptSetpoint(true, idleSetpoint(20'203'000)));
	EXPECT_TRUE(handover.pending());

	EXPECT_TRUE(handover.acceptSetpoint(true, missionSetpoint(20'214'000)));
	EXPECT_FALSE(handover.pending());
}

TEST(FwControlHandover, PreservesIdleSemanticsAfterHandover)
{
	fw_control::AutoControlHandover handover;
	handover.updateControlMode(true, 1'000'000);
	EXPECT_TRUE(handover.acceptSetpoint(true, missionSetpoint(1'010'000)));
	EXPECT_TRUE(handover.acceptSetpoint(true, idleSetpoint(2'000'000)));
	EXPECT_FALSE(handover.pending());
}

TEST(FwControlHandover, RearmsOnEachAutoEntry)
{
	fw_control::AutoControlHandover handover;
	handover.updateControlMode(true, 1'000'000);
	EXPECT_TRUE(handover.acceptSetpoint(true, missionSetpoint(1'010'000)));

	handover.updateControlMode(false, 2'000'000);
	EXPECT_FALSE(handover.pending());
	handover.updateControlMode(true, 3'000'000);
	EXPECT_TRUE(handover.pending());
	EXPECT_FALSE(handover.acceptSetpoint(true, missionSetpoint(2'500'000)));
	EXPECT_TRUE(handover.acceptSetpoint(true, missionSetpoint(3'010'000)));
}

TEST(FwControlHandover, AcceptsDirectTakeoffSetpoint)
{
	fw_control::AutoControlHandover handover;
	handover.updateControlMode(true, 1'000'000);

	auto takeoff_setpoint = idleSetpoint(1'010'000);
	takeoff_setpoint.pitch_direct = 0.2f;
	EXPECT_TRUE(handover.acceptSetpoint(true, takeoff_setpoint));
	EXPECT_FALSE(handover.pending());
}
