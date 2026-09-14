#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <random>

#include <lib/rate_control/rate_control.hpp>

#include "../BumplessRollITransfer.hpp"

using matrix::Vector3f;

struct BumplessRollITransferTestAccess
{
	static void setState(BumplessRollITransfer &transfer, float transferred_raw, uint32_t reset_epoch)
	{
		transfer._transferred_i_raw = transferred_raw;
		transfer._expected_reset_epoch = reset_epoch;
		transfer._state = BumplessRollITransfer::State::TransferHold;
	}
};

namespace
{
constexpr float StateTolerance = 2e-7f;
constexpr float AccumulatedTolerance = 2e-6f;

void configure(RateControl &rate_control, float imax = 0.2f)
{
	rate_control.setPidGains(Vector3f(0.f, 0.f, 0.f), Vector3f(1.f, 0.5f, 0.25f), Vector3f(0.f, 0.f, 0.f));
	rate_control.setIntegratorLimit(Vector3f(imax, 0.3f, 0.4f));
}

void seed_roll(RateControl &rate_control, float sign)
{
	rate_control.update(Vector3f(), Vector3f(sign, 0.f, 0.f), Vector3f(), 0.1f, false);
}

float roll_i(RateControl &rate_control)
{
	rate_ctrl_status_s status{};
	rate_control.getRateControlStatus(status);
	return status.rollspeed_integ;
}

BumplessRollITransfer::Inputs enabled_inputs(float dt = 0.02f)
{
	BumplessRollITransfer::Inputs in{};
	in.enabled = true;
	in.eligible = true;
	in.rates_enabled = true;
	in.config_valid = true;
	in.dt = dt;
	in.imax_raw = 0.2f;
	in.cap_raw = 0.08f;
	in.slew_raw_per_s = 0.5f;
	in.safety_slew_raw_per_s = 1.f;
	in.g_current = 0.7f;
	return in;
}

void enter_transfer(BumplessRollITransfer &transfer, RateControl &rate_control,
		    const BumplessRollITransfer::Inputs &in)
{
	transfer.synchronizeReset(rate_control.rollIntegralResetEpoch());
	EXPECT_EQ(transfer.update(in, rate_control).state, BumplessRollITransfer::State::Eligible);
	EXPECT_EQ(transfer.update(in, rate_control).state, BumplessRollITransfer::State::TransferIn);
}
}

TEST(BumplessRollITransfer, PositiveRampInPreservesTotalAndStopsAtZero)
{
	RateControl rate_control;
	configure(rate_control);
	seed_roll(rate_control, 1.f);
	BumplessRollITransfer transfer;
	const auto in = enabled_inputs();
	const float total_before = roll_i(rate_control);
	enter_transfer(transfer, rate_control, in);

	for (int i = 0; i < 20; ++i) {
		const auto result = transfer.update(in, rate_control);
		EXPECT_NEAR(result.residual_i_raw + result.transferred_i_raw, total_before, StateTolerance);
		EXPECT_NEAR(result.accepted_delta_i_raw + result.accepted_delta_s_raw, 0.f, StateTolerance);
		EXPECT_GE(result.residual_i_raw, -StateTolerance);
	}

	EXPECT_NEAR(roll_i(rate_control), total_before - 0.08f, AccumulatedTolerance);
	EXPECT_NEAR(transfer.transferredRaw(), 0.08f, AccumulatedTolerance);
}

TEST(BumplessRollITransfer, NegativeRampInIsSymmetricAndNoZeroCrossing)
{
	RateControl rate_control;
	configure(rate_control);
	seed_roll(rate_control, -1.f);
	BumplessRollITransfer transfer;
	const auto in = enabled_inputs();
	const float total_before = roll_i(rate_control);
	enter_transfer(transfer, rate_control, in);

	for (int i = 0; i < 20; ++i) {
		const auto result = transfer.update(in, rate_control);
		EXPECT_NEAR(result.residual_i_raw + result.transferred_i_raw, total_before, StateTolerance);
		EXPECT_LE(result.residual_i_raw, StateTolerance);
	}

	EXPECT_NEAR(transfer.transferredRaw(), -0.08f, AccumulatedTolerance);
}

TEST(BumplessRollITransfer, TargetIsLatchedAndDoesNotHarvestNaturalIntegral)
{
	RateControl rate_control;
	configure(rate_control);
	seed_roll(rate_control, 1.f);
	BumplessRollITransfer transfer;
	auto in = enabled_inputs();
	enter_transfer(transfer, rate_control, in);
	const float latched = transfer.latchedTargetRaw();
	rate_control.setRollITransferContext(transfer.transferredRaw(), true);
	rate_control.update(Vector3f(), Vector3f(1.f, 0.f, 0.f), Vector3f(), 0.02f, false);
	transfer.update(in, rate_control);
	EXPECT_FLOAT_EQ(transfer.latchedTargetRaw(), latched);
}

TEST(BumplessRollITransfer, RawCompositionPrecedesGainAndConstrain)
{
	const float baseline_raw = 0.15f;
	const float transferred_raw = 0.08f;
	const float gain = 4.f;
	const float trim = 0.2f;
	const float raw_total = BumplessRollITransfer::composeRawRoll(baseline_raw, transferred_raw);
	const float b2b = math::constrain(gain * raw_total + trim, -1.f, 1.f);
	const float legacy_b2a = math::constrain(gain * baseline_raw + trim, -1.f, 1.f) + gain * transferred_raw;
	EXPECT_FLOAT_EQ(raw_total, 0.23f);
	EXPECT_FLOAT_EQ(b2b, 1.f);
	EXPECT_GT(legacy_b2a, 1.f);
}

TEST(BumplessRollITransfer, DynamicGainUsesCurrentGainWithoutStateConversion)
{
	const float total = 0.17f;
	const float transferred = 0.06f;
	const float residual = total - transferred;
	EXPECT_NEAR(0.5f * BumplessRollITransfer::composeRawRoll(residual, transferred), 0.5f * total, 1e-6f);
	EXPECT_NEAR(1.3f * BumplessRollITransfer::composeRawRoll(residual, transferred), 1.3f * total, 1e-6f);
}

TEST(BumplessRollITransfer, NormalBackTransferPositiveAndNegative)
{
	for (const float sign : {1.f, -1.f}) {
		RateControl rate_control;
		configure(rate_control);
		seed_roll(rate_control, sign);
		BumplessRollITransfer transfer;
		auto in = enabled_inputs();
		const float total = roll_i(rate_control);
		enter_transfer(transfer, rate_control, in);
		for (int i = 0; i < 20; ++i) { transfer.update(in, rate_control); }
		in.enabled = false;
		for (int i = 0; i < 20; ++i) {
			const auto result = transfer.update(in, rate_control);
			EXPECT_NEAR(result.residual_i_raw + result.transferred_i_raw, total, StateTolerance);
		}
		EXPECT_EQ(transfer.state(), BumplessRollITransfer::State::Disabled);
		EXPECT_NEAR(roll_i(rate_control), total, AccumulatedTolerance);
		EXPECT_NEAR(transfer.transferredRaw(), 0.f, StateTolerance);
	}
}

TEST(BumplessRollITransfer, ExactPositiveAndNegativeImaxTotalsFullyExit)
{
	for (const float sign : {1.f, -1.f}) {
		RateControl rate_control;
		configure(rate_control);
		BumplessRollITransfer transfer;
		ASSERT_TRUE(rate_control.applyRollITransfer({0.12f * sign, 0.f,
			RateControl::RollITransferMode::PairPreserving}).valid);
		BumplessRollITransferTestAccess::setState(transfer, 0.08f * sign, rate_control.rollIntegralResetEpoch());
		auto in = enabled_inputs();
		in.enabled = false;
		for (int i = 0; i < 20; ++i) { transfer.update(in, rate_control); }
		EXPECT_NEAR(roll_i(rate_control), 0.2f * sign, StateTolerance);
		EXPECT_NEAR(transfer.transferredRaw(), 0.f, StateTolerance);
		EXPECT_FALSE(transfer.lastResult().normal_limited);
	}
}

TEST(BumplessRollITransfer, LegalConvexBackTransferProperty)
{
	std::mt19937 generator(20260913);
	std::uniform_real_distribution<double> distribution(-0.2, 0.2);
	std::uniform_real_distribution<double> alpha_distribution(0.0, 1.0);

	for (int sample = 0; sample < 100000; ++sample) {
		const double i = distribution(generator);
		const double total = distribution(generator);
		const double alpha = alpha_distribution(generator);
		const double s = total - i;
		const double s_after = alpha * s;
		const double i_after = alpha * i + (1.0 - alpha) * total;
		EXPECT_LE(std::fabs(i_after), 0.2 + 1e-12);
		EXPECT_NEAR(i_after + s_after, total, 1e-12);
	}
}

TEST(BumplessRollITransfer, InvalidTotalAndRuntimeImaxShrinkEnterRecovery)
{
	RateControl rate_control;
	configure(rate_control);
	BumplessRollITransfer transfer;
	ASSERT_TRUE(rate_control.applyRollITransfer({0.19f, 0.f,
		RateControl::RollITransferMode::PairPreserving}).valid);
	BumplessRollITransferTestAccess::setState(transfer, 0.08f, rate_control.rollIntegralResetEpoch());
	auto in = enabled_inputs();
	EXPECT_EQ(transfer.update(in, rate_control).state, BumplessRollITransfer::State::RecoveryReconciliation);
	EXPECT_TRUE(transfer.lastResult().reset_required);

	RateControl second_rate_control;
	configure(second_rate_control);
	BumplessRollITransfer second_transfer;
	ASSERT_TRUE(second_rate_control.applyRollITransfer({0.12f, 0.f,
		RateControl::RollITransferMode::PairPreserving}).valid);
	BumplessRollITransferTestAccess::setState(second_transfer, 0.08f, second_rate_control.rollIntegralResetEpoch());
	in.imax_raw = 0.15f;
	EXPECT_EQ(second_transfer.update(in, second_rate_control).state,
		  BumplessRollITransfer::State::RecoveryReconciliation);
}

TEST(BumplessRollITransfer, MissionToStabilizedUsesImmediateSafetyExit)
{
	RateControl rate_control;
	configure(rate_control);
	seed_roll(rate_control, 1.f);
	BumplessRollITransfer transfer;
	auto in = enabled_inputs();
	enter_transfer(transfer, rate_control, in);
	transfer.update(in, rate_control);
	in.eligible = false;
	in.pilot_abort = true;
	const auto result = transfer.update(in, rate_control);
	EXPECT_EQ(result.state, BumplessRollITransfer::State::SafetyExit);
	EXPECT_TRUE(result.safety_exit);
	EXPECT_LT(std::fabs(result.transferred_i_raw), 0.01f + StateTolerance);
}

TEST(BumplessRollITransfer, NormalExitReenableDoesNotFreezeTransferredI)
{
	RateControl rate_control;
	configure(rate_control);
	seed_roll(rate_control, 1.f);
	BumplessRollITransfer transfer;
	auto in = enabled_inputs();
	const float total = roll_i(rate_control);
	enter_transfer(transfer, rate_control, in);

	while (transfer.state() != BumplessRollITransfer::State::TransferHold) {
		transfer.update(in, rate_control);
	}

	const float hold_magnitude = std::fabs(transfer.transferredRaw());
	ASSERT_GT(hold_magnitude, StateTolerance);
	in.enabled = false;
	const auto exit_started = transfer.update(in, rate_control);
	ASSERT_EQ(exit_started.state, BumplessRollITransfer::State::NormalTransferOut);
	ASSERT_GT(std::fabs(exit_started.transferred_i_raw), StateTolerance);
	ASSERT_LT(std::fabs(exit_started.transferred_i_raw), hold_magnitude);

	in.enabled = true;
	float previous_magnitude = std::fabs(exit_started.transferred_i_raw);
	bool reached_disabled = false;

	for (int cycle = 0; cycle < 20; ++cycle) {
		const auto result = transfer.update(in, rate_control);
		const float magnitude = std::fabs(result.transferred_i_raw);
		EXPECT_NEAR(result.total_equivalent_i_raw, total, StateTolerance);

		if (result.state == BumplessRollITransfer::State::Disabled) {
			reached_disabled = true;
			EXPECT_NEAR(magnitude, 0.f, StateTolerance);
			break;
		}

		EXPECT_EQ(result.state, BumplessRollITransfer::State::NormalTransferOut);
		EXPECT_LT(magnitude, previous_magnitude);
		previous_magnitude = magnitude;
	}

	EXPECT_TRUE(reached_disabled);
}

TEST(BumplessRollITransfer, SafetyExitGateRecoveryDoesNotCancelExit)
{
	RateControl rate_control;
	configure(rate_control);
	seed_roll(rate_control, 1.f);
	BumplessRollITransfer transfer;
	auto in = enabled_inputs();
	enter_transfer(transfer, rate_control, in);

	while (transfer.state() != BumplessRollITransfer::State::TransferHold) {
		transfer.update(in, rate_control);
	}

	const float hold_magnitude = std::fabs(transfer.transferredRaw());
	ASSERT_GT(hold_magnitude, StateTolerance);
	in.eligible = false;
	in.pilot_abort = true;
	const auto exit_started = transfer.update(in, rate_control);
	ASSERT_EQ(exit_started.state, BumplessRollITransfer::State::SafetyExit);
	ASSERT_GT(std::fabs(exit_started.transferred_i_raw), StateTolerance);
	ASSERT_LT(std::fabs(exit_started.transferred_i_raw), hold_magnitude);

	in.eligible = true;
	in.pilot_abort = false;
	float previous_magnitude = std::fabs(exit_started.transferred_i_raw);
	bool reached_disabled = false;

	for (int cycle = 0; cycle < 20; ++cycle) {
		const auto result = transfer.update(in, rate_control);
		const float magnitude = std::fabs(result.transferred_i_raw);

		if (result.state == BumplessRollITransfer::State::Disabled) {
			reached_disabled = true;
			EXPECT_NEAR(magnitude, 0.f, StateTolerance);
			break;
		}

		EXPECT_EQ(result.state, BumplessRollITransfer::State::SafetyExit);
		EXPECT_TRUE(result.safety_exit);
		EXPECT_LT(magnitude, previous_magnitude);
		previous_magnitude = magnitude;
	}

	EXPECT_TRUE(reached_disabled);
}

TEST(BumplessRollITransfer, ExitCompletesBeforeReentry)
{
	RateControl rate_control;
	configure(rate_control);
	seed_roll(rate_control, 1.f);
	BumplessRollITransfer transfer;
	auto in = enabled_inputs();
	enter_transfer(transfer, rate_control, in);

	while (transfer.state() != BumplessRollITransfer::State::TransferHold) {
		transfer.update(in, rate_control);
	}

	in.enabled = false;
	ASSERT_EQ(transfer.update(in, rate_control).state, BumplessRollITransfer::State::NormalTransferOut);
	in.enabled = true;
	bool observed_disabled = false;

	for (int cycle = 0; cycle < 20; ++cycle) {
		const auto result = transfer.update(in, rate_control);

		if (result.state == BumplessRollITransfer::State::Disabled) {
			observed_disabled = true;
			break;
		}

		EXPECT_EQ(result.state, BumplessRollITransfer::State::NormalTransferOut);
	}

	ASSERT_TRUE(observed_disabled);
	EXPECT_NEAR(transfer.transferredRaw(), 0.f, StateTolerance);
	EXPECT_EQ(transfer.update(in, rate_control).state, BumplessRollITransfer::State::Eligible);
	EXPECT_EQ(transfer.update(in, rate_control).state, BumplessRollITransfer::State::TransferIn);
}

TEST(BumplessRollITransfer, ResetMismatchCannotRemainHold)
{
	RateControl rate_control;
	configure(rate_control);
	seed_roll(rate_control, 1.f);
	BumplessRollITransfer transfer;
	const auto in = enabled_inputs();
	enter_transfer(transfer, rate_control, in);
	transfer.update(in, rate_control);
	rate_control.resetIntegral();
	const auto result = transfer.update(in, rate_control);
	EXPECT_EQ(result.state, BumplessRollITransfer::State::RecoveryReconciliation);
	EXPECT_TRUE(result.reset_mismatch);
}

TEST(BumplessRollITransfer, DisabledZeroIsBitwiseBaselineAndLegacyInactive)
{
	const float baseline = 0.1375f;
	EXPECT_FLOAT_EQ(BumplessRollITransfer::composeRawRoll(baseline, 0.f), baseline);
	BumplessRollITransfer transfer;
	RateControl rate_control;
	configure(rate_control);
	auto in = enabled_inputs();
	in.enabled = false;
	const auto result = transfer.update(in, rate_control);
	EXPECT_EQ(result.state, BumplessRollITransfer::State::Disabled);
	EXPECT_FALSE(result.legacy_b2a_active);
	EXPECT_FLOAT_EQ(result.fast_actual_roll, 0.f);
	EXPECT_FLOAT_EQ(result.fast_actual_pitch, 0.f);
	EXPECT_FLOAT_EQ(result.delta_b, 0.f);
}

TEST(BumplessRollITransfer, NaturalAndTransferDeltasAreSeparatedAndAxesIsolated)
{
	RateControl rate_control;
	configure(rate_control);
	seed_roll(rate_control, 1.f);
	BumplessRollITransfer transfer;
	const auto in = enabled_inputs();
	enter_transfer(transfer, rate_control, in);
	const float pitch_before = [] (RateControl &control) {
		rate_ctrl_status_s status{}; control.getRateControlStatus(status); return status.pitchspeed_integ;
	}(rate_control);
	const auto transferred = transfer.update(in, rate_control);
	EXPECT_NE(std::fabs(transferred.accepted_delta_i_raw), 0.f);
	rate_control.setRollITransferContext(transfer.transferredRaw(), true);
	rate_control.update(Vector3f(), Vector3f(0.2f, 0.f, 0.f), Vector3f(), 0.02f, false);
	rate_ctrl_status_s status{};
	rate_control.getRateControlStatus(status);
	EXPECT_NE(std::fabs(status.rollspeed_integ_delta_pre_imax), 0.f);
	EXPECT_FLOAT_EQ(status.pitchspeed_integ, pitch_before);
}

TEST(BumplessRollITransfer, NonfiniteStateUsesRecovery)
{
	RateControl rate_control;
	configure(rate_control);
	BumplessRollITransfer transfer;
	BumplessRollITransferTestAccess::setState(transfer, NAN, rate_control.rollIntegralResetEpoch());
	EXPECT_EQ(transfer.update(enabled_inputs(), rate_control).state,
		  BumplessRollITransfer::State::RecoveryReconciliation);
}

TEST(BumplessRollITransfer, SlewIsConsistentAtFiftyAndOneHundredHertz)
{
	auto run = [](float dt, int cycles) {
		RateControl rate_control;
		configure(rate_control);
		seed_roll(rate_control, 1.f);
		BumplessRollITransfer transfer;
		auto in = enabled_inputs(dt);
		in.slew_raw_per_s = 0.04f;
		enter_transfer(transfer, rate_control, in);
		for (int i = 0; i < cycles; ++i) { transfer.update(in, rate_control); }
		return transfer.transferredRaw();
	};
	EXPECT_NEAR(run(0.02f, 50), run(0.01f, 100), AccumulatedTolerance);
}
