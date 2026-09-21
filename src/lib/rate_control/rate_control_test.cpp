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

#include <cmath>

using namespace matrix;

namespace
{
constexpr float TransferTolerance = 2e-7f;

void configure_transfer_test(RateControl &rate_control)
{
	rate_control.setPidGains(Vector3f(0.f, 0.f, 0.f), Vector3f(1.f, 1.f, 1.f), Vector3f(0.f, 0.f, 0.f));
	rate_control.setIntegratorLimit(Vector3f(0.2f, 0.3f, 0.4f));
}

rate_ctrl_status_s status_of(RateControl &rate_control)
{
	rate_ctrl_status_s status{};
	rate_control.getRateControlStatus(status);
	return status;
}

template<typename T>
auto set_headroom_context(T &rate_control, float transferred_raw, float ratio, int)
-> decltype(rate_control.setRollITransferContext(transferred_raw, ratio, true), void())
{
	rate_control.setRollITransferContext(transferred_raw, ratio, true);
}

template<typename T>
void set_headroom_context(T &rate_control, float transferred_raw, float ratio, long)
{
	(void)ratio;
	rate_control.setRollITransferContext(transferred_raw, true);
}

void drive_roll_integral_to_limit(RateControl &rate_control, float sign)
{
	for (int cycle = 0; cycle < 40; ++cycle) {
		rate_control.update(Vector3f(), Vector3f(sign, 0.f, 0.f), Vector3f(), 0.02f, false);
	}
}
}

TEST(RateControlTest, CumulativeDiagnosticsSurviveDecimatedObservation)
{
	RateControl rc;
	configure_transfer_test(rc);
	rc.setFeedForwardGain(Vector3f());
	for (int j = 0; j < 10; ++j) {
		rc.update(Vector3f(), Vector3f(.1f, 0.f, 0.f), Vector3f(), .01f, false);
	}
	const auto a = status_of(rc);
	const float a_pre = rc.rollIntegralPreImaxAccum();
	const uint32_t a_count = rc.rollIntegralUpdateCount();
	for (int j = 0; j < 10; ++j) {
		rc.update(Vector3f(), Vector3f(.1f, 0.f, 0.f), Vector3f(), .01f, false);
	}
	const auto b = status_of(rc);
	EXPECT_EQ(rc.rollIntegralUpdateCount() - a_count, 10u);
	EXPECT_FLOAT_EQ(b.rollspeed_integ_shadow_no_imax - a.rollspeed_integ_shadow_no_imax,
			rc.rollIntegralPreImaxAccum() - a_pre);
	EXPECT_NEAR(rc.rollIntegralPreImaxAccum(), rc.rollIntegralAcceptedAccum(), 1e-7f);
	EXPECT_NEAR(rc.rollIntegralBoundRejectAccum(), 0.f, 1e-7f);
	const auto epoch = rc.rollIntegralResetEpoch();
	rc.resetIntegral(0);
	const auto reset = status_of(rc);
	EXPECT_EQ(rc.rollIntegralResetEpoch(), epoch + 1u);
	EXPECT_EQ(rc.rollIntegralUpdateCount(), 0u);
	EXPECT_FLOAT_EQ(rc.rollIntegralPreImaxAccum(), 0.f);
	EXPECT_FLOAT_EQ(rc.rollIntegralAcceptedAccum(), 0.f);
	EXPECT_FLOAT_EQ(rc.rollIntegralBoundRejectAccum(), 0.f);
	EXPECT_FLOAT_EQ(reset.rollspeed_integ_shadow_no_imax, 0.f);
	rc.update(Vector3f(), Vector3f(.1f, 0.f, 0.f), Vector3f(), .01f, true);
	EXPECT_EQ(rc.rollIntegralUpdateCount(), 0u);
}

TEST(RateControlTest, CumulativeDiagnosticsAtTransferAwareBound)
{
	for (float sign : {-1.f, 1.f}) {
		RateControl rc;
		configure_transfer_test(rc);
		rc.setRollITransferContext(sign * .03f, 0.f, true);
		drive_roll_integral_to_limit(rc, sign);
		const auto s = status_of(rc);
		const float pre = rc.rollIntegralPreImaxAccum();
		const float accepted = rc.rollIntegralAcceptedAccum();
		const float reject = rc.rollIntegralBoundRejectAccum();
		EXPECT_GT(std::fabs(s.rollspeed_integ_delta_pre_imax), std::fabs(s.rollspeed_integ_delta_accepted));
		EXPECT_NEAR(reject, pre - accepted, 1e-6f);
		EXPECT_FLOAT_EQ(s.rollspeed_integ_shadow_no_imax, pre);
		rc.setPositiveSaturationFlag(0, true);
		rc.setNegativeSaturationFlag(0, true);
		rc.update(Vector3f(), Vector3f(sign, 0.f, 0.f), Vector3f(), .02f, false);
		EXPECT_FLOAT_EQ(rc.rollIntegralPreImaxAccum(), pre);
	}
}

TEST(RateControlTest, HeadroomRatioZeroMatchesLegacyBounds)
{
	RateControl rate_control;
	configure_transfer_test(rate_control);
	set_headroom_context(rate_control, 0.03f, 0.f, 0);
	drive_roll_integral_to_limit(rate_control, 1.f);
	EXPECT_NEAR(status_of(rate_control).rollspeed_integ, 0.17f, TransferTolerance);
}

TEST(RateControlTest, HalfHeadroomPositiveTransferredState)
{
	RateControl rate_control;
	configure_transfer_test(rate_control);
	set_headroom_context(rate_control, 0.03f, 0.5f, 0);
	drive_roll_integral_to_limit(rate_control, 1.f);
	EXPECT_NEAR(status_of(rate_control).rollspeed_integ, 0.185f, TransferTolerance);
}

TEST(RateControlTest, HalfHeadroomNegativeTransferredState)
{
	RateControl rate_control;
	configure_transfer_test(rate_control);
	set_headroom_context(rate_control, -0.03f, 0.5f, 0);
	drive_roll_integral_to_limit(rate_control, -1.f);
	EXPECT_NEAR(status_of(rate_control).rollspeed_integ, -0.185f, TransferTolerance);
}

TEST(RateControlTest, FullHeadroomRestoresResidualImax)
{
	for (const float sign : {1.f, -1.f}) {
		RateControl rate_control;
		configure_transfer_test(rate_control);
		set_headroom_context(rate_control, sign * 0.03f, 1.f, 0);
		drive_roll_integral_to_limit(rate_control, sign);
		EXPECT_NEAR(status_of(rate_control).rollspeed_integ, sign * 0.2f, TransferTolerance);
	}
}

TEST(RateControlTest, HeadroomDoesNotBypassPositiveAntiWindup)
{
	RateControl rate_control;
	configure_transfer_test(rate_control);
	set_headroom_context(rate_control, 0.03f, 1.f, 0);
	rate_control.setPositiveSaturationFlag(0, true);
	drive_roll_integral_to_limit(rate_control, 1.f);
	EXPECT_FLOAT_EQ(status_of(rate_control).rollspeed_integ, 0.f);
}

TEST(RateControlTest, HeadroomDoesNotBypassNegativeAntiWindup)
{
	RateControl rate_control;
	configure_transfer_test(rate_control);
	set_headroom_context(rate_control, -0.03f, 1.f, 0);
	rate_control.setNegativeSaturationFlag(0, true);
	drive_roll_integral_to_limit(rate_control, -1.f);
	EXPECT_FLOAT_EQ(status_of(rate_control).rollspeed_integ, 0.f);
}

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

TEST(RateControlTest, RollUnboundedIntegratorDiagnostics)
{
	RateControl rate_control;
	const Vector3f gain_p(0.2f, 0.3f, 0.4f);
	const Vector3f gain_i(1.f, 0.5f, 0.25f);
	const Vector3f gain_d(0.1f, 0.2f, 0.3f);
	const Vector3f gain_ff(0.05f, 0.06f, 0.07f);
	const Vector3f integral_limit(0.2f, 0.3f, 0.4f);
	rate_control.setPidGains(gain_p, gain_i, gain_d);
	rate_control.setFeedForwardGain(gain_ff);
	rate_control.setIntegratorLimit(integral_limit);
	const Vector3f rate{};
	const Vector3f rate_sp(1.f, 0.5f, -0.25f);
	const Vector3f accel(0.1f, -0.2f, 0.3f);
	rate_ctrl_status_s status{};
	rate_ctrl_terms_s terms{};
	Vector3f reference_integral{};
	float pre_limit_max_error = 0.f;
	bool real_clipped = false;
	bool shadow_continued = false;
	float pre_accum_start = 0.f;

	for (int cycle = 0; cycle < 40; ++cycle) {
		const Vector3f rate_error = rate_sp - rate;
		const Vector3f expected_output = gain_p.emult(rate_error) + reference_integral
						 - gain_d.emult(accel) + gain_ff.emult(rate_sp);
		const Vector3f output = rate_control.update(rate, rate_sp, accel, 0.02f, false, &terms);

		for (int axis = 0; axis < 3; ++axis) {
			EXPECT_FLOAT_EQ(output(axis), expected_output(axis));
			EXPECT_FLOAT_EQ(terms.output[axis], expected_output(axis));
		}

		rate_control.getRateControlStatus(status);
		if (cycle == 0) { pre_accum_start = rate_control.rollIntegralPreImaxAccum(); }

		for (int axis = 0; axis < 3; ++axis) {
			float i_factor = rate_error(axis) / math::radians(400.f);
			i_factor = math::max(0.0f, 1.f - i_factor * i_factor);
			const float candidate = reference_integral(axis) + i_factor * gain_i(axis) * rate_error(axis) * 0.02f;
			reference_integral(axis) = math::constrain(candidate, -integral_limit(axis), integral_limit(axis));
		}

		EXPECT_FLOAT_EQ(status.rollspeed_integ, reference_integral(0));
		EXPECT_FLOAT_EQ(status.pitchspeed_integ, reference_integral(1));
		EXPECT_FLOAT_EQ(status.yawspeed_integ, reference_integral(2));

		if (std::fabs(status.rollspeed_integ) < 0.8f * 0.2f) {
			pre_limit_max_error = math::max(pre_limit_max_error,
					      std::fabs(status.rollspeed_integ_shadow_no_imax - status.rollspeed_integ));
		}

		if (status.rollspeed_integ >= 0.2f) {
			real_clipped = true;
			shadow_continued = shadow_continued || status.rollspeed_integ_shadow_no_imax > 0.2f;
		}
	}

	RecordProperty("pre_limit_max_abs_error", pre_limit_max_error);
	EXPECT_LE(pre_limit_max_error, 1e-7f);
	EXPECT_TRUE(real_clipped);
	EXPECT_TRUE(shadow_continued);
	EXPECT_NEAR(status.rollspeed_integ_shadow_no_imax - pre_accum_start,
			rate_control.rollIntegralPreImaxAccum() - pre_accum_start, 1e-6f);
	EXPECT_NEAR(rate_control.rollIntegralBoundRejectAccum(),
			rate_control.rollIntegralPreImaxAccum() - rate_control.rollIntegralAcceptedAccum(), 1e-6f);
	EXPECT_EQ(rate_control.rollIntegralUpdateCount(), 40u);
	EXPECT_GT(status.rollspeed_integ_delta_raw, 0.f);
	EXPECT_GT(status.rollspeed_integ_delta_pre_imax, 0.f);
	EXPECT_TRUE(status.rollspeed_integ_update_enabled);

	const float shadow_before_reverse = status.rollspeed_integ_shadow_no_imax;
	rate_control.update(Vector3f(), Vector3f(-1.f, 0.f, 0.f), Vector3f(), 0.02f, false);
	rate_control.getRateControlStatus(status);
	EXPECT_LT(status.rollspeed_integ_delta_pre_imax, 0.f);
	EXPECT_LT(status.rollspeed_integ_shadow_no_imax, shadow_before_reverse);

	rate_control.setPositiveSaturationFlag(0, true);
	const float shadow_before_saturation = status.rollspeed_integ_shadow_no_imax;
	rate_control.update(Vector3f(), Vector3f(1.f, 0.f, 0.f), Vector3f(), 0.02f, false);
	rate_control.getRateControlStatus(status);
	EXPECT_GT(status.rollspeed_integ_delta_raw, 0.f);
	EXPECT_FLOAT_EQ(status.rollspeed_integ_delta_pre_imax, 0.f);
	EXPECT_FLOAT_EQ(status.rollspeed_integ_shadow_no_imax, shadow_before_saturation);
	EXPECT_TRUE(status.rollspeed_integ_update_enabled);

	rate_control.resetIntegral();
	rate_control.getRateControlStatus(status);
	EXPECT_FLOAT_EQ(status.rollspeed_integ, 0.f);
	EXPECT_FLOAT_EQ(status.rollspeed_integ_shadow_no_imax, 0.f);
	EXPECT_FLOAT_EQ(status.rollspeed_integ_raw_drive_accum, 0.f);
	EXPECT_FLOAT_EQ(rate_control.rollIntegralPreImaxAccum(), 0.f);
	EXPECT_FLOAT_EQ(rate_control.rollIntegralAcceptedAccum(), 0.f);
	EXPECT_FLOAT_EQ(rate_control.rollIntegralBoundRejectAccum(), 0.f);
	EXPECT_EQ(rate_control.rollIntegralUpdateCount(), 0u);
	EXPECT_FALSE(status.rollspeed_integ_update_enabled);

	rate_control.update(Vector3f(), rate_sp, Vector3f(), 0.02f, true);
	rate_control.getRateControlStatus(status);
	EXPECT_FLOAT_EQ(status.rollspeed_integ, 0.f);
	EXPECT_FLOAT_EQ(status.rollspeed_integ_shadow_no_imax, 0.f);
	EXPECT_FALSE(status.rollspeed_integ_update_enabled);

	rate_control.setPositiveSaturationFlag(0, false);
	rate_control.update(Vector3f(NAN, 0.f, 0.f), rate_sp, Vector3f(), 0.02f, false);
	rate_control.getRateControlStatus(status);
	EXPECT_TRUE(std::isfinite(status.rollspeed_error));
	EXPECT_TRUE(std::isfinite(status.rollspeed_integ_delta_raw));
	EXPECT_TRUE(std::isfinite(status.rollspeed_integ_delta_pre_imax));
	EXPECT_TRUE(std::isfinite(status.rollspeed_integ_shadow_no_imax));
	EXPECT_FALSE(status.rollspeed_integ_update_enabled);
}

TEST(RateControlTest, RollTransferAcceptsPositiveDelta)
{
	RateControl rate_control;
	configure_transfer_test(rate_control);
	rate_control.update(Vector3f(), Vector3f(-1.f, 0.f, 0.f), Vector3f(), 0.1f, false);
	const float before = status_of(rate_control).rollspeed_integ;
	RateControl::RollITransferRequest request{0.02f, 0.f, RateControl::RollITransferMode::TowardZero};
	const auto result = rate_control.applyRollITransfer(request);
	EXPECT_TRUE(result.valid);
	EXPECT_NEAR(result.accepted_delta_raw, 0.02f, TransferTolerance);
	EXPECT_NEAR(result.residual_after_raw, before + 0.02f, TransferTolerance);
}

TEST(RateControlTest, RollTransferAcceptsNegativeDeltaSymmetrically)
{
	RateControl rate_control;
	configure_transfer_test(rate_control);
	rate_control.update(Vector3f(), Vector3f(1.f, 0.f, 0.f), Vector3f(), 0.1f, false);
	const float before = status_of(rate_control).rollspeed_integ;
	RateControl::RollITransferRequest request{-0.02f, 0.f, RateControl::RollITransferMode::TowardZero};
	const auto result = rate_control.applyRollITransfer(request);
	EXPECT_TRUE(result.valid);
	EXPECT_NEAR(result.accepted_delta_raw, -0.02f, TransferTolerance);
	EXPECT_NEAR(result.residual_after_raw, before - 0.02f, TransferTolerance);
}

TEST(RateControlTest, RollTransferDoesNotCrossPositiveZero)
{
	RateControl rate_control;
	configure_transfer_test(rate_control);
	rate_control.update(Vector3f(), Vector3f(1.f, 0.f, 0.f), Vector3f(), 0.1f, false);
	const auto result = rate_control.applyRollITransfer(
		RateControl::RollITransferRequest{-1.f, 0.f, RateControl::RollITransferMode::TowardZero});
	EXPECT_TRUE(result.limited_at_zero);
	EXPECT_NEAR(result.residual_after_raw, 0.f, TransferTolerance);
}

TEST(RateControlTest, RollTransferDoesNotCrossNegativeZero)
{
	RateControl rate_control;
	configure_transfer_test(rate_control);
	rate_control.update(Vector3f(), Vector3f(-1.f, 0.f, 0.f), Vector3f(), 0.1f, false);
	const auto result = rate_control.applyRollITransfer(
		RateControl::RollITransferRequest{1.f, 0.f, RateControl::RollITransferMode::TowardZero});
	EXPECT_TRUE(result.limited_at_zero);
	EXPECT_NEAR(result.residual_after_raw, 0.f, TransferTolerance);
}

TEST(RateControlTest, RollTransferCannotCreateIntegralFromZero)
{
	RateControl rate_control;
	configure_transfer_test(rate_control);
	const auto result = rate_control.applyRollITransfer(
		RateControl::RollITransferRequest{0.02f, 0.f, RateControl::RollITransferMode::TowardZero});
	EXPECT_TRUE(result.valid);
	EXPECT_TRUE(result.limited_at_zero);
	EXPECT_NEAR(result.accepted_delta_raw, 0.f, TransferTolerance);
	EXPECT_NEAR(result.residual_after_raw, 0.f, TransferTolerance);
}

TEST(RateControlTest, TransferredContextShiftsOnlyRollNaturalBounds)
{
	RateControl rate_control;
	configure_transfer_test(rate_control);
	rate_control.setRollITransferContext(0.08f, true);

	for (int i = 0; i < 30; ++i) {
		rate_control.update(Vector3f(), Vector3f(1.f, 1.f, 1.f), Vector3f(), 0.02f, false);
	}

	const auto status = status_of(rate_control);
	EXPECT_LE(status.rollspeed_integ + 0.08f, 0.2f + TransferTolerance);
	EXPECT_NEAR(status.rollspeed_integ, 0.12f, TransferTolerance);
	EXPECT_NEAR(status.pitchspeed_integ, 0.3f, TransferTolerance);
	EXPECT_NEAR(status.yawspeed_integ, 0.4f, TransferTolerance);
}

TEST(RateControlTest, RollTransferLeavesPitchAndYawIntegratorsUntouched)
{
	RateControl rate_control;
	configure_transfer_test(rate_control);
	rate_control.update(Vector3f(), Vector3f(1.f, 1.f, -1.f), Vector3f(), 0.1f, false);
	const auto before = status_of(rate_control);
	rate_control.applyRollITransfer(
		RateControl::RollITransferRequest{-0.02f, 0.f, RateControl::RollITransferMode::TowardZero});
	const auto after = status_of(rate_control);
	EXPECT_FLOAT_EQ(after.pitchspeed_integ, before.pitchspeed_integ);
	EXPECT_FLOAT_EQ(after.yawspeed_integ, before.yawspeed_integ);
}

TEST(RateControlTest, RollResetAdvancesEpochAndClearsState)
{
	RateControl rate_control;
	configure_transfer_test(rate_control);
	rate_control.update(Vector3f(), Vector3f(1.f, 0.f, 0.f), Vector3f(), 0.1f, false);
	const uint32_t before_epoch = rate_control.rollIntegralResetEpoch();
	rate_control.resetIntegral();
	EXPECT_EQ(rate_control.rollIntegralResetEpoch(), before_epoch + 1);
	EXPECT_FLOAT_EQ(status_of(rate_control).rollspeed_integ, 0.f);
}
