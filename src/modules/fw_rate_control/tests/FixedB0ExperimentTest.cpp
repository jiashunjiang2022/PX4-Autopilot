#include <gtest/gtest.h>

#include "../FixedB0Experiment.hpp"

namespace
{
FixedB0Experiment::Inputs valid_inputs()
{
	FixedB0Experiment::Inputs in{};
	in.enabled = true; in.armed = true; in.airborne = true; in.fixed_wing = true;
	in.rates_enabled = true; in.supported_mode = true; in.config_valid = true;
	in.b0_tail = 0.195f; in.slew_tail_per_s = 0.02f;
	return in;
}
}

TEST(FixedB0Experiment, DisabledAndZeroAreBaseline)
{
	FixedB0Experiment e;
	auto in = valid_inputs(); in.enabled = false;
	const auto disabled = e.update(in, 0.02f, 0.1f);
	EXPECT_FLOAT_EQ(disabled.applied_torque, 0.f);
	EXPECT_EQ(disabled.exit_reason, FixedB0Experiment::ExitReason::NormalDisable);
	in.enabled = true; in.b0_tail = 0.f;
	EXPECT_FLOAT_EQ(e.update(in, 0.02f, 0.1f).applied_torque, 0.f);
}

TEST(FixedB0Experiment, MissionOnlyModePolicy)
{
	FixedB0Experiment e;
	auto in = valid_inputs();
	const auto mission = e.update(in, 20.f, 0.f);
	EXPECT_TRUE(mission.gate_valid);
	EXPECT_NEAR(mission.applied_torque, 0.2145f, 1e-6f);
	in.supported_mode = false;
	const auto stabilized = e.update(in, 0.1f, 0.f);
	EXPECT_FALSE(stabilized.gate_valid);
	EXPECT_EQ(stabilized.exit_reason, FixedB0Experiment::ExitReason::ModeExit);
	EXPECT_FLOAT_EQ(stabilized.target_tail, 0.f);
}

TEST(FixedB0Experiment, ScaleSignAndBound)
{
	FixedB0Experiment e;
	auto in = valid_inputs();
	const auto r = e.update(in, 10.f, 0.f);
	EXPECT_NEAR(r.applied_torque, 0.2145f, 1e-6f);
	in.b0_tail = -0.195f;
	const auto n = e.update(in, 20.f, 0.f);
	EXPECT_NEAR(n.applied_torque, -0.2145f, 1e-6f);
	in.b0_tail = 0.3f;
	EXPECT_FLOAT_EQ(e.update(in, 0.02f, 0.f).target_tail, 0.f);
}

TEST(FixedB0Experiment, SlewAndGates)
{
	FixedB0Experiment e;
	auto in = valid_inputs();
	const auto a = e.update(in, 0.1f, 0.f);
	EXPECT_LE(a.applied_tail, 0.002001f);
	const auto b = e.update(in, 0.1f, 0.f);
	EXPECT_LE(b.applied_tail - a.applied_tail, 0.002001f);
	in.failsafe = true;
	const auto c = e.update(in, 0.1f, 0.f);
	EXPECT_FALSE(c.gate_valid);
	EXPECT_EQ(c.exit_reason, FixedB0Experiment::ExitReason::Failsafe);
	EXPECT_NEAR(b.applied_tail - c.applied_tail, b.applied_tail, 1e-6f);
	in.failsafe = false; in.fixed_wing = false;
	const auto invalid_vehicle = e.update(in, 0.1f, 0.f);
	EXPECT_EQ(invalid_vehicle.exit_reason, FixedB0Experiment::ExitReason::ConfigInvalid);
}

TEST(FixedB0Experiment, SafetyExitReachesZeroWithinOneSecond)
{
	FixedB0Experiment e;
	auto in = valid_inputs();
	e.update(in, 20.f, 0.f);
	ASSERT_NEAR(e.update(in, 0.0f, 0.f).applied_tail, 0.195f, 1e-6f);
	in.supported_mode = false;
	for (int i = 0; i < 5; ++i) {
		const auto result = e.update(in, 0.2f, 0.f);
		EXPECT_EQ(result.exit_reason, FixedB0Experiment::ExitReason::ModeExit);
		EXPECT_FLOAT_EQ(result.target_tail, 0.f);
	}
	EXPECT_NEAR(e.update(in, 0.0f, 0.f).applied_tail, 0.f, 1e-6f);
}

TEST(FixedB0Experiment, InvalidInputsUseSafetyExitSlew)
{
	for (int fault = 0; fault < 3; ++fault) {
		FixedB0Experiment e;
		auto in = valid_inputs();
		e.update(in, 20.f, 0.f);
		if (fault == 0) { in.failsafe = true; }
		if (fault == 1) { in.config_valid = false; }
		if (fault == 2) { in.rates_enabled = false; }
		const auto result = e.update(in, 0.1f, 0.f);
		EXPECT_NEAR(result.applied_tail, 0.175f, 1e-6f);
		EXPECT_NE(result.exit_reason, FixedB0Experiment::ExitReason::NormalDisable);
	}
}

TEST(FixedB0Experiment, NormalDisableRetainsConfiguredSlew)
{
	FixedB0Experiment e;
	auto in = valid_inputs();
	e.update(in, 20.f, 0.f);
	in.enabled = false;
	const auto result = e.update(in, 0.1f, 0.f);
	EXPECT_EQ(result.exit_reason, FixedB0Experiment::ExitReason::NormalDisable);
	EXPECT_NEAR(result.applied_tail, 0.193f, 1e-6f);
}

TEST(FixedB0Experiment, DisarmedAndLandedHardReset)
{
	for (bool disarmed : {true, false}) {
		FixedB0Experiment e;
		auto in = valid_inputs();
		e.update(in, 20.f, 0.f);
		if (disarmed) { in.armed = false; } else { in.airborne = false; }
		const auto result = e.update(in, 0.1f, 0.f);
		EXPECT_EQ(result.exit_reason, FixedB0Experiment::ExitReason::HardReset);
		EXPECT_FLOAT_EQ(result.applied_tail, 0.f);
	}
}

TEST(FixedB0Experiment, FinalConstrainAndFastZeroContract)
{
	FixedB0Experiment e;
	auto in = valid_inputs();
	const auto r = e.update(in, 20.f, 0.95f);
	EXPECT_TRUE(r.total_clipped);
	EXPECT_FLOAT_EQ(FixedB0Experiment::InternalTorquePerTail, 1.1f);
	EXPECT_NEAR(in.b0_tail * FixedB0Experiment::InternalTorquePerTail, 0.2145f, 1e-6f);
}
