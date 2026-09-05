/****************************************************************************
 *
 *   Copyright (C) 2019 PX4 Development Team. All rights reserved.
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

#include <gtest/gtest.h>
#include <lib/rate_control/rate_control.hpp>

using namespace matrix;

TEST(RateControlTest, AllZeroCase)
{
	RateControl rate_control;
	Vector3f torque = rate_control.update(Vector3f(), Vector3f(), Vector3f(), 0.f, false);
	EXPECT_EQ(torque, Vector3f());
}

TEST(RateControlTest, TermsCaptureIntegralUsedInOutput)
{
	RateControl rate_control;
	rate_control.setPidGains(Vector3f(2.f, 3.f, 4.f), Vector3f(1.f, 1.f, 1.f), Vector3f(0.5f, 1.f, 2.f));
	rate_control.setFeedForwardGain(Vector3f(0.1f, 0.2f, 0.3f));
	rate_control.setIntegratorLimit(Vector3f(1.f, 1.f, 1.f));
	const Vector3f rate(0.5f, -0.5f, 1.f);
	const Vector3f setpoint(1.f, -1.f, 2.f);
	const Vector3f accel(2.f, -3.f, 4.f);
	rate_ctrl_status_s status{};
	rate_ctrl_terms_s terms{};
	Vector3f previous_integral;

	for (int cycle = 0; cycle < 2; ++cycle) {
		const Vector3f output = rate_control.update(rate, setpoint, accel, 0.01f, false, &terms);
		const Vector3f expected_p(1.f, -1.5f, 4.f);
		const Vector3f expected_d(-1.f, 3.f, -8.f);
		const Vector3f expected_ff(0.1f, -0.2f, 0.6f);

		for (int axis = 0; axis < 3; ++axis) {
			EXPECT_FLOAT_EQ(terms.p_term[axis], expected_p(axis));
			EXPECT_FLOAT_EQ(terms.i_term[axis], previous_integral(axis));
			EXPECT_FLOAT_EQ(terms.d_term[axis], expected_d(axis));
			EXPECT_FLOAT_EQ(terms.ff_term[axis], expected_ff(axis));
			EXPECT_FLOAT_EQ(terms.output[axis], output(axis));
			EXPECT_FLOAT_EQ(terms.p_term[axis] + terms.i_term[axis] + terms.d_term[axis] + terms.ff_term[axis], output(axis));
		}

		rate_control.getRateControlStatus(status);
		EXPECT_GT(status.rollspeed_integ, terms.i_term[0]);
		EXPECT_LT(status.pitchspeed_integ, terms.i_term[1]);
		EXPECT_GT(status.yawspeed_integ, terms.i_term[2]);
		previous_integral = Vector3f(status.rollspeed_integ, status.pitchspeed_integ, status.yawspeed_integ);
	}
}
