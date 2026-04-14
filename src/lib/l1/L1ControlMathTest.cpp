/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
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

#include <gtest/gtest.h>
#include <lib/l1/L1ControlMath.hpp>

using matrix::Vector2f;

TEST(L1ControlMathTest, StraightPathMatchesPlainL1)
{
	const float ground_speed = 8.0f;
	const float l1_distance = 12.0f;
	const float k_l1 = 2.25f;
	const float eta = 0.2f;

	const float accel = l1_control::calculateLateralAcceleration(
				    ground_speed,
				    l1_distance,
				    k_l1,
				    eta,
				    Vector2f(8.0f, 0.0f),
				    Vector2f(1.0f, 0.0f),
				    3.0f,
				    0.0f);

	const float expected = k_l1 * ground_speed * ground_speed / l1_distance * sinf(eta);
	EXPECT_NEAR(accel, expected, 1e-5f);
}

TEST(L1ControlMathTest, CurvedPathAddsCentripetalAccelerationOnTrack)
{
	const float accel = l1_control::calculateLateralAcceleration(
				    8.0f,
				    12.0f,
				    2.25f,
				    0.0f,
				    Vector2f(8.0f, 0.0f),
				    Vector2f(1.0f, 0.0f),
				    0.0f,
				    1.0f / 45.0f);

	EXPECT_NEAR(accel, 64.0f / 45.0f, 1e-5f);
}

TEST(L1ControlMathTest, CurvedPathFeedForwardVanishesForReverseMotion)
{
	const float accel = l1_control::calculateCurvatureLateralAcceleration(
				    Vector2f(-8.0f, 0.0f),
				    Vector2f(1.0f, 0.0f),
				    0.0f,
				    1.0f / 45.0f);

	EXPECT_FLOAT_EQ(accel, 0.0f);
}
