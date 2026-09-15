#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cfloat>
#include <fstream>
#include <iomanip>
#include <random>
#include <string>
#include <vector>

#include <lib/rate_control/rate_control.hpp>

#include "../BumplessRollITransfer.hpp"

using matrix::Vector3f;

struct BumplessRollITransferTestAccess
{
	template<typename T>
	static auto setHeadroom(T &transfer, float ratio, int)
	-> decltype(transfer._latched_headroom_release_ratio = ratio, void())
	{
		transfer._latched_headroom_release_ratio = ratio;
	}

	template<typename T>
	static void setHeadroom(T &, float, long) {}

	static void setState(BumplessRollITransfer &transfer, float transferred_raw, uint32_t reset_epoch,
			     float headroom_ratio = 0.f)
	{
		transfer._transferred_i_raw = transferred_raw;
		transfer._expected_reset_epoch = reset_epoch;
		transfer._state = BumplessRollITransfer::State::TransferHold;
		setHeadroom(transfer, headroom_ratio, 0);
	}
};

namespace
{
constexpr float StateTolerance = 2e-7f;
constexpr float AccumulatedTolerance = 2e-6f;

struct ReplayRow {
	int cycle;
	float time_s;
	BumplessRollITransfer::State state;
	BumplessRollITransfer::Reason reason;
	float rate_error;
	float residual_i;
	float transferred_s;
	float total_i;
	float gamma_param;
	float gamma_latched;
	float gamma_effective;
	float total_limit;
	float residual_lower;
	float residual_upper;
	float natural_i_delta_pre_limit;
	float natural_i_delta_accepted;
	float requested_transfer_i;
	float accepted_transfer_i;
	float requested_transfer_s;
	float accepted_transfer_s;
	float exit_authority_decay;
	bool recovery;
	bool reset_required;
};

const char *state_name(BumplessRollITransfer::State state)
{
	switch (state) {
	case BumplessRollITransfer::State::Disabled: return "Disabled";
	case BumplessRollITransfer::State::Eligible: return "Eligible";
	case BumplessRollITransfer::State::TransferIn: return "TransferIn";
	case BumplessRollITransfer::State::TransferHold: return "TransferHold";
	case BumplessRollITransfer::State::NormalTransferOut: return "NormalTransferOut";
	case BumplessRollITransfer::State::SafetyExit: return "SafetyExit";
	case BumplessRollITransfer::State::RecoveryReconciliation: return "RecoveryReconciliation";
	}

	return "Unknown";
}

void write_replay_csv(const char *filename, const std::vector<ReplayRow> &rows)
{
	const char *directory = std::getenv("B2B_HR_REPLAY_DIR");

	if (directory == nullptr) {
		return;
	}

	std::ofstream output(std::string(directory) + "/" + filename);
	ASSERT_TRUE(output.is_open());
	output << "cycle,time_s,state,reason,rate_error,residual_i,transferred_s,total_i,"
	       << "gamma_param,gamma_latched,gamma_effective,total_limit,residual_lower,residual_upper,"
	       << "natural_i_delta_pre_limit,natural_i_delta_accepted,requested_transfer_i,accepted_transfer_i,"
	       << "requested_transfer_s,accepted_transfer_s,exit_authority_decay,recovery,reset_required\n";
	output << std::fixed << std::setprecision(9);

	for (const ReplayRow &row : rows) {
		output << row.cycle << ',' << row.time_s << ',' << state_name(row.state) << ','
		       << static_cast<unsigned>(row.reason) << ',' << row.rate_error << ',' << row.residual_i << ','
		       << row.transferred_s << ',' << row.total_i << ',' << row.gamma_param << ',' << row.gamma_latched << ','
		       << row.gamma_effective << ',' << row.total_limit << ',' << row.residual_lower << ','
		       << row.residual_upper << ',' << row.natural_i_delta_pre_limit << ',' << row.natural_i_delta_accepted << ','
		       << row.requested_transfer_i << ',' << row.accepted_transfer_i << ',' << row.requested_transfer_s << ','
		       << row.accepted_transfer_s << ',' << row.exit_authority_decay << ',' << row.recovery << ','
		       << row.reset_required << '\n';
	}
}

template<typename T>
auto set_headroom_ratio(T &input, float ratio, int)
-> decltype(input.headroom_release_ratio = ratio, void())
{
	input.headroom_release_ratio = ratio;
}

template<typename T>
void set_headroom_ratio(T &, float, long) {}

template<typename T>
auto set_adaptive_defaults(T &input, int)
-> decltype(input.adapt_enabled = true,
		input.hold_cap_raw = 0.10f,
		input.residual_reserve_raw = 0.05f,
		input.adapt_tau_s = 5.f,
		input.adapt_slew_raw_per_s = 0.01f,
		input.entry_window_s = 5.f,
		input.gate_window_s = 3.f,
		input.gate_std_raw = 0.025f,
		input.gate_same_sign_fraction = 0.90f, void())
{
	input.adapt_enabled = true;
	input.hold_cap_raw = 0.10f;
	input.residual_reserve_raw = 0.05f;
	input.adapt_tau_s = 5.f;
	input.adapt_slew_raw_per_s = 0.01f;
	input.entry_window_s = 5.f;
	input.gate_window_s = 3.f;
	input.gate_std_raw = 0.025f;
	input.gate_same_sign_fraction = 0.90f;
}

template<typename T>
void set_adaptive_defaults(T &, long) {}

template<typename T>
auto adaptive_enabled(const T &result, int) -> decltype(result.adapt_enabled)
{
	return result.adapt_enabled;
}

template<typename T>
bool adaptive_enabled(const T &, long) { return false; }

template<typename T>
auto adaptive_entry_valid(const T &result, int) -> decltype(result.entry_est_valid)
{
	return result.entry_est_valid;
}

template<typename T>
bool adaptive_entry_valid(const T &, long) { return false; }

template<typename T>
auto adaptive_target(const T &result, int) -> decltype(result.adapt_target_raw)
{
	return result.adapt_target_raw;
}

template<typename T>
float adaptive_target(const T &, long) { return 0.f; }

template<typename T>
auto adaptive_t_hat(const T &result, int) -> decltype(result.adapt_t_hat_raw)
{
	return result.adapt_t_hat_raw;
}

template<typename T>
float adaptive_t_hat(const T &, long) { return 0.f; }

template<typename T>
auto adaptive_gate(const T &result, int) -> decltype(result.adapt_gate)
{
	return result.adapt_gate;
}

template<typename T>
bool adaptive_gate(const T &, long) { return false; }

template<typename T>
auto adaptive_limited(const T &result, int) -> decltype(result.adapt_limited)
{
	return result.adapt_limited;
}

template<typename T>
bool adaptive_limited(const T &, long) { return false; }

template<typename T>
auto adaptive_releasing(const T &result, int) -> decltype(result.adapt_releasing)
{
	return result.adapt_releasing;
}

template<typename T>
bool adaptive_releasing(const T &, long) { return false; }

template<typename T>
auto adaptive_reversal(const T &result, int) -> decltype(result.adapt_reversal)
{
	return result.adapt_reversal;
}

template<typename T>
bool adaptive_reversal(const T &, long) { return false; }

template<typename T>
auto effective_headroom_ratio(const T &result, int) -> decltype(result.headroom_release_ratio_effective)
{
	return result.headroom_release_ratio_effective;
}

template<typename T>
float effective_headroom_ratio(const T &, long) { return 0.f; }

template<typename T>
auto latched_headroom_ratio(const T &result, int) -> decltype(result.headroom_release_ratio_latched)
{
	return result.headroom_release_ratio_latched;
}

template<typename T>
float latched_headroom_ratio(const T &, long) { return 0.f; }

template<typename T>
auto exit_authority_decay(const T &result, int) -> decltype(result.exit_authority_decay_raw)
{
	return result.exit_authority_decay_raw;
}

template<typename T>
float exit_authority_decay(const T &, long) { return 0.f; }

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

ReplayRow run_replay_cycle(BumplessRollITransfer &transfer, RateControl &rate_control,
			   const BumplessRollITransfer::Inputs &in, int cycle, float rate_error,
			   std::vector<ReplayRow> &history)
{
	const auto result = transfer.update(in, rate_control);
	const bool context_enabled = std::fabs(result.transferred_i_raw) > FLT_EPSILON;
	rate_control.setRollITransferContext(result.transferred_i_raw,
			result.headroom_release_ratio_effective, context_enabled);
	rate_control.update(Vector3f(), Vector3f(rate_error, 0.f, 0.f), Vector3f(), in.dt, false);
	rate_ctrl_status_s status{};
	rate_control.getRateControlStatus(status);
	const auto limits = RateControl::computeRollILimits(in.imax_raw, result.transferred_i_raw,
			    result.headroom_release_ratio_effective);

	ReplayRow row{
		cycle,
		cycle * in.dt,
		result.state,
		result.reason,
		rate_error,
		status.rollspeed_integ,
		result.transferred_i_raw,
		status.rollspeed_integ + result.transferred_i_raw,
		in.headroom_release_ratio,
		result.headroom_release_ratio_latched,
		result.headroom_release_ratio_effective,
		limits.total_limit,
		limits.lower,
		limits.upper,
		status.rollspeed_integ_delta_pre_imax,
		status.rollspeed_integ_delta_accepted,
		result.requested_delta_i_raw,
		result.accepted_delta_i_raw,
		result.requested_delta_s_raw,
		result.accepted_delta_s_raw,
		result.exit_authority_decay_raw,
		result.recovery,
		result.reset_required
	};
	history.push_back(row);

	EXPECT_TRUE(std::isfinite(row.residual_i));
	EXPECT_TRUE(std::isfinite(row.transferred_s));
	EXPECT_TRUE(std::isfinite(row.total_i));
	EXPECT_TRUE(limits.valid);
	EXPECT_GE(row.residual_i, limits.lower - AccumulatedTolerance);
	EXPECT_LE(row.residual_i, limits.upper + AccumulatedTolerance);
	EXPECT_LE(std::fabs(row.residual_i), in.imax_raw + AccumulatedTolerance);
	EXPECT_LE(std::fabs(row.total_i), limits.total_limit + AccumulatedTolerance);
	EXPECT_FALSE(row.recovery);
	EXPECT_FALSE(row.reset_required);
	return row;
}

ReplayRow advance_replay_to_hold(BumplessRollITransfer &transfer, RateControl &rate_control,
				 const BumplessRollITransfer::Inputs &in, int &cycle,
				 std::vector<ReplayRow> &history)
{
	ReplayRow row{};
	bool reached_hold = false;
	float entry_total = 0.f;

	for (int iteration = 0; iteration < 40; ++iteration) {
		row = run_replay_cycle(transfer, rate_control, in, cycle++, 0.f, history);

		if (iteration == 0) {
			entry_total = row.total_i;
		}

		EXPECT_NEAR(row.total_i, entry_total, AccumulatedTolerance);

		if (row.state == BumplessRollITransfer::State::TransferIn) {
			EXPECT_FLOAT_EQ(row.gamma_effective, 0.f);
		}

		if (row.state == BumplessRollITransfer::State::TransferHold) {
			reached_hold = true;
			break;
		}
	}

	EXPECT_TRUE(reached_hold);
	return row;
}

void finish_normal_replay_exit(BumplessRollITransfer &transfer, RateControl &rate_control,
			       BumplessRollITransfer::Inputs &in, int &cycle,
			       std::vector<ReplayRow> &history, bool &authority_decay_observed)
{
	in.enabled = false;
	bool disabled = false;
	float previous_total_magnitude = std::fabs(rate_control.rollIntegralRaw() + transfer.transferredRaw());

	for (int iteration = 0; iteration < 40; ++iteration) {
		const ReplayRow row = run_replay_cycle(transfer, rate_control, in, cycle++, 0.f, history);
		authority_decay_observed = authority_decay_observed
					 || std::fabs(row.exit_authority_decay) > StateTolerance;
		EXPECT_LE(std::fabs(row.total_i), previous_total_magnitude + AccumulatedTolerance);
		previous_total_magnitude = std::fabs(row.total_i);

		if (row.state == BumplessRollITransfer::State::Disabled) {
			disabled = true;
			break;
		}
	}

	EXPECT_TRUE(disabled);
	EXPECT_NEAR(history.back().transferred_s, 0.f, StateTolerance);
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

void naturally_move_roll_i_to(RateControl &rate_control, float transferred_raw, float target)
{
	rate_control.setRollITransferContext(transferred_raw, true);

	for (int iteration = 0; iteration < 8; ++iteration) {
		const float error = target - roll_i(rate_control);
		rate_control.update(Vector3f(), Vector3f(error, 0.f, 0.f), Vector3f(), 1.f, false);
	}

	ASSERT_NEAR(roll_i(rate_control), target, StateTolerance);
}

void expect_normal_exit_to_disabled(BumplessRollITransfer &transfer, RateControl &rate_control,
				    const BumplessRollITransfer::Inputs &in)
{
	float previous_magnitude = std::fabs(transfer.transferredRaw());
	ASSERT_GT(previous_magnitude, StateTolerance);
	bool observed_disabled = false;

	for (int cycle = 0; cycle < 20; ++cycle) {
		const float total_before = roll_i(rate_control) + transfer.transferredRaw();
		const auto result = transfer.update(in, rate_control);
		EXPECT_FALSE(result.recovery);
		EXPECT_FALSE(result.reset_required);
		EXPECT_NEAR(result.total_equivalent_i_raw, total_before, StateTolerance);
		EXPECT_NEAR(result.accepted_delta_i_raw + result.accepted_delta_s_raw, 0.f, StateTolerance);
		const float magnitude = std::fabs(result.transferred_i_raw);

		if (result.state == BumplessRollITransfer::State::Disabled) {
			observed_disabled = true;
			EXPECT_NEAR(magnitude, 0.f, StateTolerance);
			break;
		}

		EXPECT_EQ(result.state, BumplessRollITransfer::State::NormalTransferOut);
		EXPECT_LT(magnitude, previous_magnitude);
		previous_magnitude = magnitude;
	}

	EXPECT_TRUE(observed_disabled);
}

void seed_residual(RateControl &rate_control, float value)
{
	const auto seeded = rate_control.applyRollITransfer({value, 0.f, RateControl::RollITransferMode::PairPreserving});
	ASSERT_TRUE(seeded.valid);
	ASSERT_NEAR(roll_i(rate_control), value, StateTolerance);
}

void drive_natural_roll_i(RateControl &rate_control, float sign)
{
	for (int cycle = 0; cycle < 40; ++cycle) {
		rate_control.update(Vector3f(), Vector3f(sign, 0.f, 0.f), Vector3f(), 0.02f, false);
	}
}
}

TEST(BumplessRollITransfer, TransferInConservesTotalWithHeadroomConfigured)
{
	RateControl rate_control;
	configure(rate_control);
	seed_residual(rate_control, 0.1f);
	BumplessRollITransfer transfer;
	auto in = enabled_inputs();
	in.cap_raw = 0.03f;
	set_headroom_ratio(in, 1.f, 0);
	enter_transfer(transfer, rate_control, in);
	const float initial_total = roll_i(rate_control);

	while (transfer.state() != BumplessRollITransfer::State::TransferHold) {
		const auto result = transfer.update(in, rate_control);
		EXPECT_NEAR(result.total_equivalent_i_raw, initial_total, StateTolerance);
		EXPECT_NEAR(result.accepted_delta_i_raw + result.accepted_delta_s_raw, 0.f, StateTolerance);

		if (result.state == BumplessRollITransfer::State::TransferIn) {
			EXPECT_FLOAT_EQ(effective_headroom_ratio(result, 0), 0.f);
		}
	}

	EXPECT_FLOAT_EQ(effective_headroom_ratio(transfer.lastResult(), 0), 1.f);
}

TEST(BumplessRollITransfer, HeadroomActivatesOnlyInTransferHold)
{
	RateControl rate_control;
	configure(rate_control);
	seed_residual(rate_control, 0.1f);
	BumplessRollITransfer transfer;
	auto in = enabled_inputs();
	set_headroom_ratio(in, 0.5f, 0);
	transfer.synchronizeReset(rate_control.rollIntegralResetEpoch());
	EXPECT_FLOAT_EQ(effective_headroom_ratio(transfer.update(in, rate_control), 0), 0.f);
	EXPECT_FLOAT_EQ(effective_headroom_ratio(transfer.update(in, rate_control), 0), 0.f);

	while (transfer.state() != BumplessRollITransfer::State::TransferHold) {
		transfer.update(in, rate_control);
	}

	EXPECT_FLOAT_EQ(effective_headroom_ratio(transfer.lastResult(), 0), 0.5f);
}

TEST(BumplessRollITransfer, HeadroomRatioLatchedForEpisode)
{
	RateControl rate_control;
	configure(rate_control);
	seed_residual(rate_control, 0.1f);
	BumplessRollITransfer transfer;
	auto in = enabled_inputs();
	set_headroom_ratio(in, 0.5f, 0);
	enter_transfer(transfer, rate_control, in);
	set_headroom_ratio(in, 1.f, 0);

	while (transfer.state() != BumplessRollITransfer::State::TransferHold) {
		transfer.update(in, rate_control);
	}

	EXPECT_FLOAT_EQ(latched_headroom_ratio(transfer.lastResult(), 0), 0.5f);
	EXPECT_FLOAT_EQ(effective_headroom_ratio(transfer.lastResult(), 0), 0.5f);
	in.enabled = false;

	while (transfer.state() != BumplessRollITransfer::State::Disabled) {
		transfer.update(in, rate_control);
	}

	in.enabled = true;
	EXPECT_EQ(transfer.update(in, rate_control).state, BumplessRollITransfer::State::Eligible);
	EXPECT_FLOAT_EQ(latched_headroom_ratio(transfer.lastResult(), 0), 1.f);
}

TEST(BumplessRollITransfer, HalfHeadroomAllowsNaturalResidualGrowth)
{
	RateControl rate_control;
	configure(rate_control);
	seed_residual(rate_control, 0.2f);
	BumplessRollITransfer transfer;
	auto in = enabled_inputs();
	in.cap_raw = 0.03f;
	set_headroom_ratio(in, 0.5f, 0);
	enter_transfer(transfer, rate_control, in);

	while (transfer.state() != BumplessRollITransfer::State::TransferHold) {
		transfer.update(in, rate_control);
	}

	set_headroom_context(rate_control, transfer.transferredRaw(), effective_headroom_ratio(transfer.lastResult(), 0), 0);
	drive_natural_roll_i(rate_control, 1.f);
	EXPECT_NEAR(roll_i(rate_control), 0.185f, AccumulatedTolerance);
}

TEST(BumplessRollITransfer, FullHeadroomAllowsNaturalResidualGrowthToImax)
{
	RateControl rate_control;
	configure(rate_control);
	seed_residual(rate_control, 0.2f);
	BumplessRollITransfer transfer;
	auto in = enabled_inputs();
	in.cap_raw = 0.03f;
	set_headroom_ratio(in, 1.f, 0);
	enter_transfer(transfer, rate_control, in);

	while (transfer.state() != BumplessRollITransfer::State::TransferHold) {
		transfer.update(in, rate_control);
	}

	set_headroom_context(rate_control, transfer.transferredRaw(), effective_headroom_ratio(transfer.lastResult(), 0), 0);
	drive_natural_roll_i(rate_control, 1.f);
	EXPECT_NEAR(roll_i(rate_control), 0.2f, AccumulatedTolerance);
}

TEST(BumplessRollITransfer, PositiveAndNegativeHeadroomAreSymmetric)
{
	for (const float sign : {1.f, -1.f}) {
		RateControl rate_control;
		configure(rate_control);
		seed_residual(rate_control, sign * 0.2f);
		BumplessRollITransfer transfer;
		auto in = enabled_inputs();
		in.cap_raw = 0.03f;
		set_headroom_ratio(in, 0.5f, 0);
		enter_transfer(transfer, rate_control, in);

		while (transfer.state() != BumplessRollITransfer::State::TransferHold) {
			transfer.update(in, rate_control);
		}

		set_headroom_context(rate_control, transfer.transferredRaw(), effective_headroom_ratio(transfer.lastResult(), 0), 0);
		drive_natural_roll_i(rate_control, sign);
		EXPECT_NEAR(roll_i(rate_control), sign * 0.185f, AccumulatedTolerance);
	}
}

TEST(BumplessRollITransfer, NormalExitExactWhenFeasibleWithHeadroom)
{
	RateControl rate_control;
	configure(rate_control);
	seed_residual(rate_control, 0.18f);
	BumplessRollITransfer transfer;
	auto in = enabled_inputs();
	set_headroom_ratio(in, 1.f, 0);
	BumplessRollITransferTestAccess::setState(transfer, 0.03f, rate_control.rollIntegralResetEpoch(), 1.f);
	in.enabled = false;
	const auto result = transfer.update(in, rate_control);
	EXPECT_EQ(result.state, BumplessRollITransfer::State::NormalTransferOut);
	EXPECT_FALSE(result.recovery);
	EXPECT_NEAR(result.accepted_delta_i_raw, 0.01f, StateTolerance);
	EXPECT_NEAR(result.accepted_delta_s_raw, -0.01f, StateTolerance);
	EXPECT_NEAR(result.total_equivalent_i_raw, 0.21f, StateTolerance);
}

TEST(BumplessRollITransfer, NormalExitDecaysAuthorityWhenExactTransferBecomesImpossible)
{
	RateControl rate_control;
	configure(rate_control);
	seed_residual(rate_control, 0.2f);
	BumplessRollITransfer transfer;
	auto in = enabled_inputs();
	set_headroom_ratio(in, 1.f, 0);
	BumplessRollITransferTestAccess::setState(transfer, 0.01f, rate_control.rollIntegralResetEpoch(), 1.f);
	in.enabled = false;
	const auto result = transfer.update(in, rate_control);
	EXPECT_EQ(result.state, BumplessRollITransfer::State::Disabled);
	EXPECT_FALSE(result.recovery);
	EXPECT_FALSE(result.reset_required);
	EXPECT_NEAR(result.residual_i_raw, 0.2f, StateTolerance);
	EXPECT_NEAR(result.transferred_i_raw, 0.f, StateTolerance);
	EXPECT_NEAR(result.total_equivalent_i_raw, 0.2f, StateTolerance);
	EXPECT_NEAR(exit_authority_decay(result, 0), -0.01f, StateTolerance);
}

TEST(BumplessRollITransfer, HalfHeadroomBoundaryExitDoesNotRecover)
{
	RateControl rate_control;
	configure(rate_control);
	seed_residual(rate_control, 0.185f);
	BumplessRollITransfer transfer;
	auto in = enabled_inputs();
	set_headroom_ratio(in, 0.5f, 0);
	BumplessRollITransferTestAccess::setState(transfer, 0.03f, rate_control.rollIntegralResetEpoch(), 0.5f);
	in.enabled = false;
	const auto result = transfer.update(in, rate_control);
	EXPECT_EQ(result.state, BumplessRollITransfer::State::NormalTransferOut);
	EXPECT_FALSE(result.recovery);
	EXPECT_FALSE(result.reset_required);
}

TEST(BumplessRollITransfer, GammaZeroNormalExitRemainsExact)
{
	RateControl rate_control;
	configure(rate_control);
	seed_residual(rate_control, 0.17f);
	BumplessRollITransfer transfer;
	auto in = enabled_inputs();
	BumplessRollITransferTestAccess::setState(transfer, 0.03f, rate_control.rollIntegralResetEpoch());
	in.enabled = false;

	while (transfer.state() != BumplessRollITransfer::State::Disabled) {
		const auto result = transfer.update(in, rate_control);
		EXPECT_NEAR(result.accepted_delta_i_raw + result.accepted_delta_s_raw, 0.f, StateTolerance);
		EXPECT_NEAR(result.total_equivalent_i_raw, 0.2f, StateTolerance);
	}
}

TEST(BumplessRollITransfer, HeadroomLegalTotalAboveImaxIsNotRecovery)
{
	RateControl rate_control;
	configure(rate_control);
	seed_residual(rate_control, 0.19f);
	BumplessRollITransfer transfer;
	auto in = enabled_inputs();
	set_headroom_ratio(in, 1.f, 0);
	BumplessRollITransferTestAccess::setState(transfer, 0.03f, rate_control.rollIntegralResetEpoch(), 1.f);
	const auto result = transfer.update(in, rate_control);
	EXPECT_EQ(result.state, BumplessRollITransfer::State::TransferHold);
	EXPECT_FALSE(result.recovery);
	EXPECT_FALSE(result.reset_required);
}

TEST(BumplessRollITransfer, HeadroomLegalTotalSafetyExitDoesNotRecover)
{
	RateControl rate_control;
	configure(rate_control);
	seed_residual(rate_control, 0.2f);
	BumplessRollITransfer transfer;
	auto in = enabled_inputs();
	set_headroom_ratio(in, 1.f, 0);
	BumplessRollITransferTestAccess::setState(transfer, 0.03f, rate_control.rollIntegralResetEpoch(), 1.f);
	in.pilot_abort = true;
	const auto first = transfer.update(in, rate_control);
	EXPECT_EQ(first.state, BumplessRollITransfer::State::SafetyExit);
	EXPECT_FALSE(first.recovery);
	EXPECT_FALSE(first.reset_required);
	EXPECT_LE(std::fabs(first.total_equivalent_i_raw), first.total_limit_raw + StateTolerance);

	BumplessRollITransfer::Result completed{};

	for (int cycle = 0; cycle < 3 && transfer.state() != BumplessRollITransfer::State::Disabled; ++cycle) {
		completed = transfer.update(in, rate_control);
		EXPECT_FALSE(completed.recovery);
		EXPECT_FALSE(completed.reset_required);
	}

	EXPECT_EQ(completed.state, BumplessRollITransfer::State::Disabled);
	EXPECT_NEAR(completed.residual_i_raw, 0.2f, StateTolerance);
	EXPECT_NEAR(completed.transferred_i_raw, 0.f, StateTolerance);
}

TEST(BumplessRollITransfer, TotalBeyondHeadroomEnvelopeIsRecovery)
{
	RateControl rate_control;
	configure(rate_control);
	seed_residual(rate_control, 0.19f);
	BumplessRollITransfer transfer;
	auto in = enabled_inputs();
	set_headroom_ratio(in, 0.5f, 0);
	BumplessRollITransferTestAccess::setState(transfer, 0.03f, rate_control.rollIntegralResetEpoch(), 0.5f);
	const auto result = transfer.update(in, rate_control);
	EXPECT_EQ(result.state, BumplessRollITransfer::State::RecoveryReconciliation);
	EXPECT_TRUE(result.recovery);
	EXPECT_TRUE(result.reset_required);
}

TEST(BumplessRollITransfer, ExitCompletesBeforeReentryWithHeadroom)
{
	RateControl rate_control;
	configure(rate_control);
	seed_residual(rate_control, 0.2f);
	BumplessRollITransfer transfer;
	auto in = enabled_inputs();
	in.cap_raw = 0.03f;
	set_headroom_ratio(in, 1.f, 0);
	enter_transfer(transfer, rate_control, in);

	while (transfer.state() != BumplessRollITransfer::State::TransferHold) {
		transfer.update(in, rate_control);
	}

	set_headroom_context(rate_control, transfer.transferredRaw(), 1.f, 0);
	drive_natural_roll_i(rate_control, 1.f);
	in.enabled = false;
	ASSERT_EQ(transfer.update(in, rate_control).state, BumplessRollITransfer::State::NormalTransferOut);
	in.enabled = true;

	while (transfer.state() != BumplessRollITransfer::State::Disabled) {
		EXPECT_EQ(transfer.update(in, rate_control).state == BumplessRollITransfer::State::Disabled
			  || transfer.state() == BumplessRollITransfer::State::NormalTransferOut, true);
	}

	EXPECT_EQ(transfer.update(in, rate_control).state, BumplessRollITransfer::State::Eligible);
}

TEST(BumplessRollITransfer, TransferInPositiveResidualReachesZero)
{
	RateControl rate_control;
	configure(rate_control);
	seed_roll(rate_control, 1.f);
	BumplessRollITransfer transfer;
	auto in = enabled_inputs();
	in.cap_raw = 0.05f;
	enter_transfer(transfer, rate_control, in);
	transfer.update(in, rate_control);
	ASSERT_GT(transfer.transferredRaw(), StateTolerance);
	ASSERT_LT(transfer.transferredRaw(), transfer.latchedTargetRaw());
	naturally_move_roll_i_to(rate_control, transfer.transferredRaw(), 0.f);
	const float total_before_cancel = roll_i(rate_control) + transfer.transferredRaw();
	const auto cancelled = transfer.update(in, rate_control);
	EXPECT_EQ(cancelled.state, BumplessRollITransfer::State::NormalTransferOut);
	EXPECT_FALSE(cancelled.recovery);
	EXPECT_FALSE(cancelled.reset_required);
	EXPECT_NEAR(cancelled.total_equivalent_i_raw, total_before_cancel, StateTolerance);
	expect_normal_exit_to_disabled(transfer, rate_control, in);
}

TEST(BumplessRollITransfer, TransferInNegativeResidualReachesZero)
{
	RateControl rate_control;
	configure(rate_control);
	seed_roll(rate_control, -1.f);
	BumplessRollITransfer transfer;
	auto in = enabled_inputs();
	in.cap_raw = 0.05f;
	enter_transfer(transfer, rate_control, in);
	transfer.update(in, rate_control);
	ASSERT_LT(transfer.transferredRaw(), -StateTolerance);
	ASSERT_LT(std::fabs(transfer.transferredRaw()), std::fabs(transfer.latchedTargetRaw()));
	naturally_move_roll_i_to(rate_control, transfer.transferredRaw(), 0.f);
	const float total_before_cancel = roll_i(rate_control) + transfer.transferredRaw();
	const auto cancelled = transfer.update(in, rate_control);
	EXPECT_EQ(cancelled.state, BumplessRollITransfer::State::NormalTransferOut);
	EXPECT_FALSE(cancelled.recovery);
	EXPECT_FALSE(cancelled.reset_required);
	EXPECT_NEAR(cancelled.total_equivalent_i_raw, total_before_cancel, StateTolerance);
	expect_normal_exit_to_disabled(transfer, rate_control, in);
}

TEST(BumplessRollITransfer, TransferInPositiveEntryResidualReverses)
{
	RateControl rate_control;
	configure(rate_control);
	seed_roll(rate_control, 1.f);
	BumplessRollITransfer transfer;
	auto in = enabled_inputs();
	in.cap_raw = 0.05f;
	enter_transfer(transfer, rate_control, in);
	transfer.update(in, rate_control);
	naturally_move_roll_i_to(rate_control, transfer.transferredRaw(), -0.01f);
	const auto cancelled = transfer.update(in, rate_control);
	EXPECT_EQ(cancelled.state, BumplessRollITransfer::State::NormalTransferOut);
	EXPECT_NE(cancelled.reason, BumplessRollITransfer::Reason::ControlInvalid);
	EXPECT_FALSE(cancelled.recovery);
	EXPECT_FALSE(cancelled.reset_required);
	expect_normal_exit_to_disabled(transfer, rate_control, in);
}

TEST(BumplessRollITransfer, TransferInNegativeEntryResidualReverses)
{
	RateControl rate_control;
	configure(rate_control);
	seed_roll(rate_control, -1.f);
	BumplessRollITransfer transfer;
	auto in = enabled_inputs();
	in.cap_raw = 0.05f;
	enter_transfer(transfer, rate_control, in);
	transfer.update(in, rate_control);
	naturally_move_roll_i_to(rate_control, transfer.transferredRaw(), 0.01f);
	const auto cancelled = transfer.update(in, rate_control);
	EXPECT_EQ(cancelled.state, BumplessRollITransfer::State::NormalTransferOut);
	EXPECT_NE(cancelled.reason, BumplessRollITransfer::Reason::ControlInvalid);
	EXPECT_FALSE(cancelled.recovery);
	EXPECT_FALSE(cancelled.reset_required);
	expect_normal_exit_to_disabled(transfer, rate_control, in);
}

TEST(BumplessRollITransfer, TransferHoldOppositionExits)
{
	RateControl rate_control;
	configure(rate_control);
	seed_roll(rate_control, 1.f);
	BumplessRollITransfer transfer;
	auto in = enabled_inputs();
	in.cap_raw = 0.05f;
	enter_transfer(transfer, rate_control, in);

	while (transfer.state() != BumplessRollITransfer::State::TransferHold) {
		transfer.update(in, rate_control);
	}

	naturally_move_roll_i_to(rate_control, transfer.transferredRaw(), -0.01f);
	const auto exit_started = transfer.update(in, rate_control);
	EXPECT_EQ(exit_started.state, BumplessRollITransfer::State::NormalTransferOut);
	EXPECT_FALSE(exit_started.recovery);
	EXPECT_FALSE(exit_started.reset_required);
	expect_normal_exit_to_disabled(transfer, rate_control, in);
	EXPECT_EQ(transfer.update(in, rate_control).state, BumplessRollITransfer::State::Eligible);
}

TEST(BumplessRollITransfer, OldLatchedTargetCannotHarvestLaterI)
{
	RateControl rate_control;
	configure(rate_control);
	seed_roll(rate_control, 1.f);
	BumplessRollITransfer transfer;
	auto in = enabled_inputs();
	in.cap_raw = 0.05f;
	enter_transfer(transfer, rate_control, in);
	transfer.update(in, rate_control);
	const float old_target = transfer.latchedTargetRaw();
	naturally_move_roll_i_to(rate_control, transfer.transferredRaw(), 0.f);
	ASSERT_EQ(transfer.update(in, rate_control).state, BumplessRollITransfer::State::NormalTransferOut);

	naturally_move_roll_i_to(rate_control, transfer.transferredRaw(), 0.005f);
	expect_normal_exit_to_disabled(transfer, rate_control, in);
	naturally_move_roll_i_to(rate_control, transfer.transferredRaw(), 0.02f);
	EXPECT_EQ(transfer.update(in, rate_control).state, BumplessRollITransfer::State::Eligible);
	EXPECT_NEAR(transfer.latchedTargetRaw(), roll_i(rate_control), StateTolerance);
	EXPECT_LT(transfer.latchedTargetRaw(), old_target);
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

TEST(BumplessRollITransfer, NegativeHeadroomNormalExitExactWhenFeasible)
{
	RateControl rate_control;
	configure(rate_control);
	seed_residual(rate_control, -0.18f);
	BumplessRollITransfer transfer;
	auto in = enabled_inputs();
	in.headroom_release_ratio = 1.f;
	BumplessRollITransferTestAccess::setState(transfer, -0.03f,
			rate_control.rollIntegralResetEpoch(), 1.f);
	in.enabled = false;
	const auto result = transfer.update(in, rate_control);
	EXPECT_EQ(result.state, BumplessRollITransfer::State::NormalTransferOut);
	EXPECT_TRUE(result.normal_exit);
	EXPECT_FALSE(result.recovery);
	EXPECT_FALSE(result.reset_required);
	EXPECT_NEAR(result.accepted_delta_i_raw, -0.01f, StateTolerance);
	EXPECT_NEAR(result.accepted_delta_s_raw, 0.01f, StateTolerance);
	EXPECT_NEAR(result.total_equivalent_i_raw, -0.21f, StateTolerance);
	EXPECT_NEAR(result.exit_authority_decay_raw, 0.f, StateTolerance);
}

TEST(BumplessRollITransfer, NegativeHeadroomNormalExitDecaysAuthorityAtNegativeImax)
{
	RateControl rate_control;
	configure(rate_control);
	seed_residual(rate_control, -0.2f);
	BumplessRollITransfer transfer;
	auto in = enabled_inputs();
	in.headroom_release_ratio = 1.f;
	BumplessRollITransferTestAccess::setState(transfer, -0.01f,
			rate_control.rollIntegralResetEpoch(), 1.f);
	in.enabled = false;
	const auto result = transfer.update(in, rate_control);
	EXPECT_EQ(result.state, BumplessRollITransfer::State::Disabled);
	EXPECT_FALSE(result.recovery);
	EXPECT_FALSE(result.reset_required);
	EXPECT_NEAR(result.residual_i_raw, -0.2f, StateTolerance);
	EXPECT_NEAR(result.transferred_i_raw, 0.f, StateTolerance);
	EXPECT_NEAR(result.total_equivalent_i_raw, -0.2f, StateTolerance);
	EXPECT_NEAR(result.exit_authority_decay_raw, 0.01f, StateTolerance);
}

TEST(BumplessRollITransfer, NegativeHalfHeadroomBoundaryExitDoesNotRecover)
{
	RateControl rate_control;
	configure(rate_control);
	seed_residual(rate_control, -0.185f);
	BumplessRollITransfer transfer;
	auto in = enabled_inputs();
	in.headroom_release_ratio = 0.5f;
	BumplessRollITransferTestAccess::setState(transfer, -0.03f,
			rate_control.rollIntegralResetEpoch(), 0.5f);
	in.enabled = false;

	while (transfer.state() != BumplessRollITransfer::State::Disabled) {
		const auto result = transfer.update(in, rate_control);
		EXPECT_FALSE(result.recovery);
		EXPECT_FALSE(result.reset_required);
		EXPECT_GE(result.residual_i_raw, -0.2f - StateTolerance);
	}

	EXPECT_NEAR(transfer.transferredRaw(), 0.f, StateTolerance);
	EXPECT_GE(roll_i(rate_control), -0.2f - StateTolerance);
}

TEST(BumplessRollITransfer, NegativeHeadroomSafetyExitDoesNotRecover)
{
	RateControl rate_control;
	configure(rate_control);
	seed_residual(rate_control, -0.19f);
	BumplessRollITransfer transfer;
	auto in = enabled_inputs();
	in.headroom_release_ratio = 1.f;
	BumplessRollITransferTestAccess::setState(transfer, -0.03f,
			rate_control.rollIntegralResetEpoch(), 1.f);
	in.pilot_abort = true;
	const auto first = transfer.update(in, rate_control);
	EXPECT_EQ(first.state, BumplessRollITransfer::State::SafetyExit);
	EXPECT_FALSE(first.recovery);
	EXPECT_FALSE(first.reset_required);

	while (transfer.state() != BumplessRollITransfer::State::Disabled) {
		const auto result = transfer.update(in, rate_control);
		EXPECT_FALSE(result.recovery);
		EXPECT_FALSE(result.reset_required);
	}

	EXPECT_NEAR(transfer.transferredRaw(), 0.f, StateTolerance);
	EXPECT_LE(std::fabs(roll_i(rate_control)), 0.2f + StateTolerance);
}

TEST(BumplessRollITransfer, NegativeTotalBeyondHeadroomEnvelopeStillRecovers)
{
	RateControl rate_control;
	configure(rate_control);
	seed_residual(rate_control, -0.19f);
	BumplessRollITransfer transfer;
	auto in = enabled_inputs();
	in.headroom_release_ratio = 0.5f;
	BumplessRollITransferTestAccess::setState(transfer, -0.03f,
			rate_control.rollIntegralResetEpoch(), 0.5f);
	const auto result = transfer.update(in, rate_control);
	EXPECT_EQ(result.state, BumplessRollITransfer::State::RecoveryReconciliation);
	EXPECT_TRUE(result.recovery);
	EXPECT_TRUE(result.reset_required);
	EXPECT_EQ(result.reason, BumplessRollITransfer::Reason::TotalEquivalentLimit);
}

TEST(BumplessRollITransfer, MultiCyclePositiveHalfHeadroomEpisode)
{
	RateControl rate_control;
	configure(rate_control);
	seed_residual(rate_control, 0.2f);
	BumplessRollITransfer transfer;
	transfer.synchronizeReset(rate_control.rollIntegralResetEpoch());
	auto in = enabled_inputs();
	in.cap_raw = 0.02f;
	in.slew_raw_per_s = 0.1f;
	in.headroom_release_ratio = 0.5f;
	std::vector<ReplayRow> history;
	int cycle = 0;
	const ReplayRow hold_entry = advance_replay_to_hold(transfer, rate_control, in, cycle, history);
	EXPECT_NEAR(hold_entry.transferred_s, 0.02f, AccumulatedTolerance);
	EXPECT_NEAR(hold_entry.residual_i, 0.18f, AccumulatedTolerance);
	EXPECT_NEAR(hold_entry.total_i, 0.2f, AccumulatedTolerance);
	const float residual_at_hold = hold_entry.residual_i;

	for (int iteration = 0; iteration < 6; ++iteration) {
		run_replay_cycle(transfer, rate_control, in, cycle++, 1.f, history);
	}

	EXPECT_GT(roll_i(rate_control), residual_at_hold + StateTolerance);
	EXPECT_NEAR(roll_i(rate_control), 0.19f, AccumulatedTolerance);
	bool authority_decay_observed = false;
	finish_normal_replay_exit(transfer, rate_control, in, cycle, history, authority_decay_observed);
	EXPECT_TRUE(authority_decay_observed);
	write_replay_csv("R1_positive_half.csv", history);
}

TEST(BumplessRollITransfer, MultiCycleNegativeHalfHeadroomEpisode)
{
	RateControl rate_control;
	configure(rate_control);
	seed_residual(rate_control, -0.2f);
	BumplessRollITransfer transfer;
	transfer.synchronizeReset(rate_control.rollIntegralResetEpoch());
	auto in = enabled_inputs();
	in.cap_raw = 0.02f;
	in.slew_raw_per_s = 0.1f;
	in.headroom_release_ratio = 0.5f;
	std::vector<ReplayRow> history;
	int cycle = 0;
	const ReplayRow hold_entry = advance_replay_to_hold(transfer, rate_control, in, cycle, history);
	EXPECT_NEAR(hold_entry.transferred_s, -0.02f, AccumulatedTolerance);
	EXPECT_NEAR(hold_entry.residual_i, -0.18f, AccumulatedTolerance);
	EXPECT_NEAR(hold_entry.total_i, -0.2f, AccumulatedTolerance);
	const float residual_at_hold = hold_entry.residual_i;

	for (int iteration = 0; iteration < 6; ++iteration) {
		run_replay_cycle(transfer, rate_control, in, cycle++, -1.f, history);
	}

	EXPECT_LT(roll_i(rate_control), residual_at_hold - StateTolerance);
	EXPECT_NEAR(roll_i(rate_control), -0.19f, AccumulatedTolerance);
	bool authority_decay_observed = false;
	finish_normal_replay_exit(transfer, rate_control, in, cycle, history, authority_decay_observed);
	EXPECT_TRUE(authority_decay_observed);
	write_replay_csv("R2_negative_half.csv", history);
}

TEST(BumplessRollITransfer, MultiCycleFullHeadroomRestoresResidualImax)
{
	RateControl rate_control;
	configure(rate_control);
	seed_residual(rate_control, 0.2f);
	BumplessRollITransfer transfer;
	transfer.synchronizeReset(rate_control.rollIntegralResetEpoch());
	auto in = enabled_inputs();
	in.cap_raw = 0.02f;
	in.slew_raw_per_s = 0.1f;
	in.headroom_release_ratio = 1.f;
	std::vector<ReplayRow> history;
	int cycle = 0;
	advance_replay_to_hold(transfer, rate_control, in, cycle, history);

	for (int iteration = 0; iteration < 6; ++iteration) {
		run_replay_cycle(transfer, rate_control, in, cycle++, 1.f, history);
	}

	EXPECT_NEAR(roll_i(rate_control), 0.2f, AccumulatedTolerance);
	EXPECT_NEAR(history.back().total_i, 0.22f, AccumulatedTolerance);
	bool authority_decay_observed = false;
	finish_normal_replay_exit(transfer, rate_control, in, cycle, history, authority_decay_observed);
	EXPECT_TRUE(authority_decay_observed);
	write_replay_csv("R3_full_release.csv", history);
}

TEST(BumplessRollITransfer, MultiCycleGammaZeroMatchesLegacyBehavior)
{
	RateControl rate_control;
	configure(rate_control);
	seed_residual(rate_control, 0.2f);
	BumplessRollITransfer transfer;
	transfer.synchronizeReset(rate_control.rollIntegralResetEpoch());
	auto in = enabled_inputs();
	in.cap_raw = 0.02f;
	in.slew_raw_per_s = 0.1f;
	in.headroom_release_ratio = 0.f;
	std::vector<ReplayRow> history;
	int cycle = 0;
	const ReplayRow hold_entry = advance_replay_to_hold(transfer, rate_control, in, cycle, history);

	for (int iteration = 0; iteration < 6; ++iteration) {
		run_replay_cycle(transfer, rate_control, in, cycle++, 1.f, history);
	}

	EXPECT_NEAR(roll_i(rate_control), hold_entry.residual_i, AccumulatedTolerance);
	bool authority_decay_observed = false;
	finish_normal_replay_exit(transfer, rate_control, in, cycle, history, authority_decay_observed);
	EXPECT_FALSE(authority_decay_observed);
	EXPECT_NEAR(history.back().total_i, 0.2f, AccumulatedTolerance);
	write_replay_csv("R4_gamma_zero.csv", history);
}

TEST(BumplessRollITransfer, MultiCyclePilotAbortFromReleasedHeadroom)
{
	RateControl rate_control;
	configure(rate_control);
	seed_residual(rate_control, 0.2f);
	BumplessRollITransfer transfer;
	transfer.synchronizeReset(rate_control.rollIntegralResetEpoch());
	auto in = enabled_inputs();
	in.cap_raw = 0.02f;
	in.slew_raw_per_s = 0.1f;
	in.safety_slew_raw_per_s = 0.2f;
	in.headroom_release_ratio = 1.f;
	std::vector<ReplayRow> history;
	int cycle = 0;
	advance_replay_to_hold(transfer, rate_control, in, cycle, history);

	for (int iteration = 0; iteration < 6; ++iteration) {
		run_replay_cycle(transfer, rate_control, in, cycle++, 1.f, history);
	}

	ASSERT_GT(std::fabs(history.back().total_i), in.imax_raw + StateTolerance);
	in.pilot_abort = true;
	bool safety_exit_observed = false;
	bool disabled = false;
	float previous_total_magnitude = std::fabs(history.back().total_i);
	float previous_transferred_magnitude = std::fabs(history.back().transferred_s);

	for (int iteration = 0; iteration < 20; ++iteration) {
		const ReplayRow row = run_replay_cycle(transfer, rate_control, in, cycle++, 0.f, history);
		safety_exit_observed = safety_exit_observed || row.state == BumplessRollITransfer::State::SafetyExit;
		EXPECT_LE(std::fabs(row.total_i), previous_total_magnitude + AccumulatedTolerance);
		EXPECT_LE(std::fabs(row.transferred_s), previous_transferred_magnitude + AccumulatedTolerance);
		previous_total_magnitude = std::fabs(row.total_i);
		previous_transferred_magnitude = std::fabs(row.transferred_s);

		if (row.state == BumplessRollITransfer::State::Disabled) {
			disabled = true;
			break;
		}
	}

	EXPECT_TRUE(safety_exit_observed);
	EXPECT_TRUE(disabled);
	EXPECT_NEAR(history.back().transferred_s, 0.f, StateTolerance);
	write_replay_csv("R5_pilot_abort.csv", history);
}

TEST(BumplessRollITransfer, MultiCycleHeadroomDoesNotBypassAllocatorAntiWindup)
{
	std::vector<ReplayRow> history;
	int cycle = 0;

	for (const float sign : {1.f, -1.f}) {
		RateControl rate_control;
		configure(rate_control);
		seed_residual(rate_control, sign * 0.2f);
		BumplessRollITransfer transfer;
		transfer.synchronizeReset(rate_control.rollIntegralResetEpoch());
		auto in = enabled_inputs();
		in.cap_raw = 0.02f;
		in.slew_raw_per_s = 0.1f;
		in.headroom_release_ratio = 1.f;
		advance_replay_to_hold(transfer, rate_control, in, cycle, history);
		const float residual_before_saturation = roll_i(rate_control);

		if (sign > 0.f) {
			rate_control.setPositiveSaturationFlag(0, true);

		} else {
			rate_control.setNegativeSaturationFlag(0, true);
		}

		for (int iteration = 0; iteration < 5; ++iteration) {
			run_replay_cycle(transfer, rate_control, in, cycle++, sign, history);
		}

		EXPECT_NEAR(roll_i(rate_control), residual_before_saturation, StateTolerance);

		if (sign > 0.f) {
			rate_control.setPositiveSaturationFlag(0, false);

		} else {
			rate_control.setNegativeSaturationFlag(0, false);
		}

		run_replay_cycle(transfer, rate_control, in, cycle++, sign, history);
		EXPECT_GT(sign * (roll_i(rate_control) - residual_before_saturation), StateTolerance);
	}

	write_replay_csv("R6_antiwindup.csv", history);
}

TEST(BumplessRollITransfer, AdaptiveV3StableLargePositiveBurden)
{
	RateControl rate_control;
	configure(rate_control);
	seed_residual(rate_control, 0.18f);
	BumplessRollITransfer transfer;
	auto in = enabled_inputs();
	in.cap_raw = 0.03f;
	set_adaptive_defaults(in, 0);
	transfer.synchronizeReset(rate_control.rollIntegralResetEpoch());

	for (int cycle = 0; cycle < 5; ++cycle) {
		transfer.update(in, rate_control);
	}

	for (int cycle = 0; cycle < 800; ++cycle) {
		const auto result = transfer.update(in, rate_control);
		EXPECT_NEAR(result.accepted_delta_i_raw + result.accepted_delta_s_raw, 0.f, AccumulatedTolerance);
	}

	EXPECT_NEAR(transfer.transferredRaw(), 0.10f, 2e-3f);
	EXPECT_NEAR(roll_i(rate_control), 0.08f, 2e-3f);
}

TEST(BumplessRollITransfer, AdaptiveV3StableLargeNegativeBurden)
{
	RateControl rate_control;
	configure(rate_control);
	seed_residual(rate_control, -0.18f);
	BumplessRollITransfer transfer;
	auto in = enabled_inputs();
	in.cap_raw = 0.03f;
	set_adaptive_defaults(in, 0);
	transfer.synchronizeReset(rate_control.rollIntegralResetEpoch());

	for (int cycle = 0; cycle < 805; ++cycle) {
		const auto result = transfer.update(in, rate_control);
		EXPECT_NEAR(result.accepted_delta_i_raw + result.accepted_delta_s_raw, 0.f, AccumulatedTolerance);
	}

	EXPECT_NEAR(transfer.transferredRaw(), -0.10f, 2e-3f);
	EXPECT_NEAR(roll_i(rate_control), -0.08f, 2e-3f);
}

TEST(BumplessRollITransfer, AdaptiveV3DiagnosticsAreAvailable)
{
	RateControl rate_control;
	configure(rate_control);
	seed_residual(rate_control, 0.18f);
	BumplessRollITransfer transfer;
	auto in = enabled_inputs();
	set_adaptive_defaults(in, 0);
	transfer.synchronizeReset(rate_control.rollIntegralResetEpoch());
	const auto result = transfer.update(in, rate_control);
	EXPECT_TRUE(adaptive_enabled(result, 0));
	EXPECT_TRUE(std::isfinite(adaptive_t_hat(result, 0)));
	EXPECT_TRUE(std::isfinite(adaptive_target(result, 0)));
}

TEST(BumplessRollITransfer, AdaptiveV3InsufficientEntryHistoryFallsBackConservatively)
{
	RateControl rate_control;
	configure(rate_control);
	seed_residual(rate_control, 0.12f);
	BumplessRollITransfer transfer;
	auto in = enabled_inputs();
	in.cap_raw = 0.03f;
	set_adaptive_defaults(in, 0);
	transfer.synchronizeReset(rate_control.rollIntegralResetEpoch());
	const auto result = transfer.update(in, rate_control);
	EXPECT_TRUE(adaptive_enabled(result, 0));
	EXPECT_FALSE(adaptive_entry_valid(result, 0));
	EXPECT_NEAR(adaptive_target(result, 0), 0.f, 1e-6f);
	EXPECT_FLOAT_EQ(transfer.latchedTargetRaw(), 0.03f);
}

TEST(BumplessRollITransfer, AdaptiveV3GrowthWaitsForPersistenceGate)
{
	RateControl rate_control;
	configure(rate_control);
	seed_residual(rate_control, 0.18f);
	BumplessRollITransfer transfer;
	auto in = enabled_inputs();
	in.cap_raw = 0.03f;
	set_adaptive_defaults(in, 0);
	transfer.synchronizeReset(rate_control.rollIntegralResetEpoch());
	for (int cycle = 0; cycle < 5; ++cycle) {
		transfer.update(in, rate_control);
	}
	const float hold_start = transfer.transferredRaw();
	for (int cycle = 0; cycle < 40; ++cycle) {
		const auto result = transfer.update(in, rate_control);
		EXPECT_FALSE(adaptive_gate(result, 0));
	}
	EXPECT_FLOAT_EQ(transfer.transferredRaw(), hold_start);
}

TEST(BumplessRollITransfer, AdaptiveV3InvalidParametersDoNotMoveState)
{
	RateControl rate_control;
	configure(rate_control);
	seed_residual(rate_control, 0.12f);
	BumplessRollITransfer transfer;
	auto in = enabled_inputs();
	in.cap_raw = 0.03f;
	set_adaptive_defaults(in, 0);
	in.adapt_tau_s = NAN;
	transfer.synchronizeReset(rate_control.rollIntegralResetEpoch());
	const auto result = transfer.update(in, rate_control);
	EXPECT_FALSE(adaptive_enabled(result, 0));
	EXPECT_FLOAT_EQ(transfer.transferredRaw(), 0.f);
}

TEST(BumplessRollITransfer, AdaptiveV3GoldenVectorExport)
{
	const char *directory = std::getenv("B2B_ADAPTIVE_GOLDEN_DIR");

	if (directory == nullptr) {
		GTEST_SKIP() << "B2B_ADAPTIVE_GOLDEN_DIR is not set";
	}

	std::ofstream output(std::string(directory) + "/adaptive_v3_cpp.csv");
	ASSERT_TRUE(output.is_open());
	output << "case,step,input_total,state,residual,transferred,total,t_hat,entry,entry_valid,target,std,sign_fraction,gate,limited,releasing,reversal,delta_i,delta_s\n";
	output << std::fixed << std::setprecision(9);

	struct Trace {
		const char *name;
		float entry_value;
		int entry_cycles;
		std::vector<float> hold;
	};

	std::vector<Trace> traces;
	const auto stable = [](float value, int count) { return std::vector<float>(count, value); };
	traces.push_back({"stable_pos_018", 0.18f, 300, stable(0.18f, 80)});
	traces.push_back({"stable_neg_018", -0.18f, 300, stable(-0.18f, 80)});
	traces.push_back({"small_pos_007", 0.07f, 300, stable(0.07f, 20)});
	traces.push_back({"below_reserve", 0.04f, 300, stable(0.04f, 20)});
	{
		std::vector<float> values = stable(0.10f, 65);
		for (int i = 0; i < 25; ++i) { values[40 + i] = 0.20f; }
		traces.push_back({"gust_05s", 0.10f, 300, values});
	}
	{
		std::vector<float> values = stable(0.10f, 130);
		for (int i = 0; i < 100; ++i) { values[30 + i] = 0.20f; }
		traces.push_back({"gust_2s", 0.10f, 300, values});
	}
	{
		std::vector<float> values = stable(0.10f, 22);
		for (size_t i = 20; i < values.size(); ++i) { values[i] = (i % 2 == 0) ? 0.18f : 0.02f; }
		traces.push_back({"high_variance", 0.10f, 300, values});
	}
	{
		std::vector<float> values = stable(0.10f, 45);
		for (size_t i = 40; i < values.size(); ++i) { values[i] = (i % 2 == 0) ? 0.15f : -0.15f; }
		traces.push_back({"sign_changing", 0.10f, 300, values});
	}
	{
		std::vector<float> values = stable(0.18f, 75);
		for (size_t i = 70; i < values.size(); ++i) { values[i] = 0.f; }
		traces.push_back({"bias_to_zero", 0.18f, 300, values});
	}
	{
		std::vector<float> values = stable(0.18f, 75);
		for (size_t i = 70; i < values.size(); ++i) { values[i] = -0.18f; }
		traces.push_back({"bias_to_negative", 0.18f, 300, values});
	}
	traces.push_back({"insufficient_history", 0.12f, 40, stable(0.12f, 100)});
	traces.push_back({"safety_exit", 0.18f, 300, stable(0.18f, 8)});

	for (const Trace &trace : traces) {
		RateControl rate_control;
		configure(rate_control);
		seed_residual(rate_control, trace.entry_value);
		BumplessRollITransfer transfer;
		auto in = enabled_inputs();
		in.cap_raw = 0.03f;
		set_adaptive_defaults(in, 0);
		transfer.synchronizeReset(rate_control.rollIntegralResetEpoch());

		auto set_total = [&](float total) {
			const float delta = total - (rate_control.rollIntegralRaw() + transfer.transferredRaw());
			if (std::fabs(delta) > StateTolerance) {
				const auto applied = rate_control.applyRollITransfer({delta, transfer.transferredRaw(),
						RateControl::RollITransferMode::PairPreserving, 0.f,
						transfer.transferredRaw(), true});
				ASSERT_TRUE(applied.valid);
			}
		};

		auto write_row = [&](int step, float input_total, const BumplessRollITransfer::Result &result) {
			output << trace.name << ',' << step << ',' << input_total << ',' << static_cast<unsigned>(result.state) << ','
			       << result.residual_i_raw << ',' << result.transferred_i_raw << ','
			       << result.total_equivalent_i_raw << ',' << result.adapt_t_hat_raw << ','
			       << result.entry_est_raw << ',' << result.entry_est_valid << ','
			       << result.adapt_target_raw << ',' << result.adapt_std_raw << ','
			       << result.adapt_sign_fraction << ',' << result.adapt_gate << ','
			       << result.adapt_limited << ',' << result.adapt_releasing << ','
			       << result.adapt_reversal << ',' << result.accepted_delta_i_raw << ','
			       << result.accepted_delta_s_raw << '\n';
		};

		int step = 0;
		in.eligible = false;
		for (int i = 0; i < trace.entry_cycles; ++i) {
			set_total(trace.entry_value);
			const float input_total = rate_control.rollIntegralRaw() + transfer.transferredRaw();
			write_row(step++, input_total, transfer.update(in, rate_control));
		}

		in.eligible = true;
		write_row(step++, rate_control.rollIntegralRaw() + transfer.transferredRaw(), transfer.update(in, rate_control));
		write_row(step++, rate_control.rollIntegralRaw() + transfer.transferredRaw(), transfer.update(in, rate_control));

		for (const float total : trace.hold) {
			set_total(total);
			const float input_total = rate_control.rollIntegralRaw() + transfer.transferredRaw();
			write_row(step++, input_total, transfer.update(in, rate_control));
		}

		if (std::strcmp(trace.name, "safety_exit") == 0) {
			in.failsafe = true;
			in.eligible = false;
			set_total(0.18f);
			write_row(step++, rate_control.rollIntegralRaw() + transfer.transferredRaw(), transfer.update(in, rate_control));
		}
	}
}
