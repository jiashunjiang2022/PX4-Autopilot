#include <gtest/gtest.h>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <limits>

#include "../BumplessRollITransfer.hpp"
// Compare against the immutable git base, never a reimplementation of its logic.
#define BumplessRollITransfer SlowReference
#include "SlowReference.hpp"
#undef BumplessRollITransfer

using matrix::Vector3f;
using Slow = BumplessRollITransfer;

namespace
{
constexpr float Tolerance = 2e-7f;

template<typename Input> Input inputs()
{
	Input in{};
	in.enabled = true;
	in.eligible = true;
	in.rates_enabled = true;
	in.config_valid = true;
	in.dt = 0.02f;
	in.imax_raw = 0.2f;
	in.cap_raw = 0.03f;
	in.slew_raw_per_s = 0.02f;
	in.safety_slew_raw_per_s = 0.2f;
	in.g_current = 0.73f;
	in.adapt_enabled = true;
	in.hold_cap_raw = 0.1f;
	in.residual_reserve_raw = 0.05f;
	in.adapt_tau_s = 5.f;
	in.adapt_slew_raw_per_s = 0.01f;
	in.entry_window_s = 5.f;
	in.gate_window_s = 3.f;
	in.gate_std_raw = 0.025f;
	in.gate_same_sign_fraction = 0.9f;
	return in;
}

void configure(RateControl &rc)
{
	rc.setPidGains(Vector3f(0.05f, 0.f, 0.f), Vector3f(0.1f, 0.f, 0.f), Vector3f());
	rc.setFeedForwardGain(Vector3f(0.8f, 0.f, 0.f));
	rc.setIntegratorLimit(Vector3f(0.2f, 0.2f, 0.2f));
}

void setTotal(RateControl &rc, float total, float s, float hr = 0.f)
{
	const auto accepted = rc.applyRollITransfer({total - s - rc.rollIntegralRaw(), s,
		RateControl::RollITransferMode::PairPreserving, hr});
	ASSERT_TRUE(accepted.valid);
	ASSERT_NEAR(rc.rollIntegralRaw() + s, total, Tolerance);
}

struct Episode {
	RateControl rc;
	Slow slow;
	Slow::Inputs in{inputs<Slow::Inputs>()};
	explicit Episode(int32_t mode)
	{
		configure(rc);
		in.comparator_mode = mode;
		setTotal(rc, 0.18f, 0.f);
		slow.synchronizeReset(rc.rollIntegralResetEpoch());
	}
	void enter()
	{
		for (int i = 0; i < 200 && slow.state() != Slow::State::TransferHold; ++i) { slow.update(in, rc); }
		ASSERT_EQ(slow.state(), Slow::State::TransferHold);
	}
	Slow::Result step(float total)
	{
		setTotal(rc, total, slow.transferredRaw(), slow.lastResult().headroom_release_ratio_effective);
		return slow.update(in, rc);
	}
};

void paired(const Slow::Result &out, float dt, float slew)
{
	EXPECT_FALSE(out.reset_required);
	EXPECT_NEAR(out.accepted_delta_i_raw + out.accepted_delta_s_raw, 0.f, Tolerance);
	EXPECT_NEAR(out.total_equivalent_i_raw,
		out.residual_i_before_raw + out.transferred_i_before_raw, Tolerance);
	EXPECT_LE(std::fabs(out.accepted_delta_s_raw), slew * dt + Tolerance);
}

void compare(const Slow::Result &a, const SlowReference::Result &b)
{
	EXPECT_EQ(static_cast<int>(a.state), static_cast<int>(b.state));
	EXPECT_EQ(static_cast<int>(a.reason), static_cast<int>(b.reason));
#define FIELD(x) EXPECT_EQ(a.x, b.x) << #x
	FIELD(residual_i_before_raw); FIELD(residual_i_raw); FIELD(transferred_i_before_raw);
	FIELD(transferred_i_raw); FIELD(total_equivalent_i_raw); FIELD(requested_delta_i_raw);
	FIELD(accepted_delta_i_raw); FIELD(requested_delta_s_raw); FIELD(accepted_delta_s_raw);
	FIELD(adapt_t_hat_raw); FIELD(adapt_target_raw); FIELD(adapt_gate); FIELD(adapt_reversal);
	FIELD(adapt_releasing); FIELD(adapt_std_raw); FIELD(adapt_sign_fraction); FIELD(adapt_limited);
	FIELD(reset_required); FIELD(recovery); FIELD(safety_exit); FIELD(normal_exit);
	FIELD(total_limit_raw); FIELD(residual_lower_raw); FIELD(residual_upper_raw);
	FIELD(entry_est_raw); FIELD(entry_est_valid); FIELD(headroom_release_ratio_effective);
	FIELD(transfer_limited); FIELD(unmatched_delta_raw); FIELD(exit_authority_decay_raw);
#undef FIELD
}
}

TEST(SlowLpfComparator, DefaultsToOriginal)
{
	EXPECT_EQ(Slow::Inputs{}.comparator_mode, 0);
	Episode e(0);
	e.enter();
	const auto out = e.step(0.18f);
	EXPECT_FALSE(out.comparator_active);
	EXPECT_TRUE(out.original_slow_gate_active);
	EXPECT_TRUE(out.reversal_logic_active);
}

TEST(SlowLpfComparator, OriginalMatchesImmutableBaseTrajectories)
{
	for (float hr : {0.f, 0.5f, 1.f}) {
		RateControl now_rc, old_rc;
		configure(now_rc); configure(old_rc);
		Slow now;
		SlowReference old;
		auto ni = inputs<Slow::Inputs>();
		auto oi = inputs<SlowReference::Inputs>();
		ni.headroom_release_ratio = oi.headroom_release_ratio = hr;
		setTotal(now_rc, 0.18f, 0.f); setTotal(old_rc, 0.18f, 0.f);
		now.synchronizeReset(now_rc.rollIntegralResetEpoch());
		old.synchronizeReset(old_rc.rollIntegralResetEpoch());
		for (int k = 0; k < 4200; ++k) {
			SCOPED_TRACE(k);
			// Entry evidence, growth, sustained reversal, short reversal, exits and reset.
			ni.eligible = oi.eligible = k >= 260 && k != 3500;
			ni.enabled = oi.enabled = !(k >= 3100 && k < 3300);
			ni.pilot_abort = oi.pilot_abort = k == 3500;
			ni.hard_reset = oi.hard_reset = k == 3900;
			const float total = k < 1000 ? 0.18f : (k < 2700 ? -0.09f : 0.08f);
			setTotal(now_rc, total, now.transferredRaw(), now.lastResult().headroom_release_ratio_effective);
			setTotal(old_rc, total, old.transferredRaw(), old.lastResult().headroom_release_ratio_effective);
			const auto a = now.update(ni, now_rc);
			const auto b = old.update(oi, old_rc);
			compare(a, b);
			if (a.reset_required) { now_rc.resetIntegral(0); now.synchronizeReset(now_rc.rollIntegralResetEpoch()); }
			if (b.reset_required) { old_rc.resetIntegral(0); old.synchronizeReset(old_rc.rollIntegralResetEpoch()); }
			now_rc.setRollITransferContext(now.transferredRaw(), a.headroom_release_ratio_effective, true);
			old_rc.setRollITransferContext(old.transferredRaw(), b.headroom_release_ratio_effective, true);
			now_rc.setPositiveSaturationFlag(0, k % 7 == 0);
			old_rc.setPositiveSaturationFlag(0, k % 7 == 0);
			now_rc.setNegativeSaturationFlag(0, k % 11 == 0);
			old_rc.setNegativeSaturationFlag(0, k % 11 == 0);
			const Vector3f sp((k % 2 ? 1.f : -1.f), 0.f, 0.f);
			const auto u = now_rc.update(Vector3f(), sp, Vector3f(), ni.dt, ni.hard_reset);
			const auto v = old_rc.update(Vector3f(), sp, Vector3f(), oi.dt, oi.hard_reset);
			EXPECT_EQ(now_rc.rollIntegralRaw(), old_rc.rollIntegralRaw());
			EXPECT_EQ(math::constrain(ni.g_current * (u(0) + now.transferredRaw()), -1.f, 1.f),
				math::constrain(oi.g_current * (v(0) + old.transferredRaw()), -1.f, 1.f));
		}
	}
}

TEST(SlowLpfComparator, EntryIdenticalAndModeLatched)
{
	Episode original(0), lpf(1);
	for (int k = 0; k < 200; ++k) {
		const auto a = original.slow.update(original.in, original.rc);
		const auto b = lpf.slow.update(lpf.in, lpf.rc);
		EXPECT_EQ(a.state, b.state);
		EXPECT_EQ(a.transferred_i_raw, b.transferred_i_raw);
		EXPECT_EQ(a.residual_i_raw, b.residual_i_raw);
		EXPECT_EQ(a.adapt_t_hat_raw, b.adapt_t_hat_raw);
		if (a.state == Slow::State::TransferHold) { break; }
	}
	ASSERT_EQ(lpf.slow.state(), Slow::State::TransferHold);
	lpf.in.comparator_mode = 0;
	const auto out = lpf.step(0.18f);
	EXPECT_EQ(out.comparator_mode, 1);
	EXPECT_TRUE(out.comparator_active);
}

TEST(SlowLpfComparator, SameFilterTargetAndImmediateGrowthWithoutEvidence)
{
	Episode e(1);
	e.enter();
	const float h = e.slow.lastResult().adapt_t_hat_raw;
	const auto out = e.step(0.17f);
	EXPECT_NEAR(out.adapt_t_hat_raw, h + e.in.dt / (5.f + e.in.dt) * (0.17f - h), Tolerance);
	EXPECT_NEAR(out.adapt_target_raw, std::fmin(0.1f, std::fmax(std::fabs(out.adapt_t_hat_raw) - 0.05f, 0.f)), Tolerance);
	EXPECT_GT(out.accepted_delta_s_raw, 0.f);
	EXPECT_TRUE(out.comparator_active);
	EXPECT_FALSE(out.adapt_gate);
	EXPECT_FALSE(out.original_slow_gate_active);
	EXPECT_FALSE(out.reversal_logic_active);
	paired(out, e.in.dt, e.in.adapt_slew_raw_per_s);
}

TEST(SlowLpfComparator, GateThresholdsDoNotQualifyLpfOutput)
{
	Episode a(1), b(1);
	a.enter(); b.enter();
	b.in.gate_window_s = 10.f;
	b.in.gate_std_raw = 0.f;
	b.in.gate_same_sign_fraction = 1.f;
	for (int k = 0; k < 400; ++k) {
		const float t = k % 2 ? 0.15f : 0.09f;
		const auto x = a.step(t), y = b.step(t);
		EXPECT_EQ(x.transferred_i_raw, y.transferred_i_raw);
		EXPECT_EQ(x.adapt_t_hat_raw, y.adapt_t_hat_raw);
	}
}

TEST(SlowLpfComparator, SustainedReversalCrossesWithoutFreshGate)
{
	Episode lpf(1), original(0);
	lpf.enter(); original.enter();
	for (int k = 0; k < 1000; ++k) { lpf.step(0.18f); original.step(0.18f); }
	bool lpf_negative = false, original_negative = false, original_wait = false;
	std::ofstream trace;
	if (const char *dir = std::getenv("SLOW_LPF_TRACE_DIR")) {
		trace.open(std::string(dir) + "/sustained_reversal.csv");
		trace << "cycle,lpf_s,slow_s,lpf_h,slow_h,lpf_gate,slow_gate\n" << std::setprecision(9);
	}
	for (int k = 0; k < 2200; ++k) {
		const auto a = lpf.step(-0.09f), b = original.step(-0.09f);
		paired(a, lpf.in.dt, lpf.in.adapt_slew_raw_per_s);
		EXPECT_FALSE(a.adapt_gate);
		EXPECT_FALSE(a.reversal_logic_active);
		lpf_negative |= a.transferred_i_raw < -Tolerance;
		original_wait |= std::fabs(b.transferred_i_raw) <= 1e-7f && !b.adapt_gate && b.adapt_t_hat_raw < -0.05f;
		if (b.transferred_i_raw < -Tolerance) { EXPECT_TRUE(b.adapt_gate); original_negative = true; }
		if (trace.is_open()) { trace << k << ',' << a.transferred_i_raw << ',' << b.transferred_i_raw << ','
			<< a.adapt_t_hat_raw << ',' << b.adapt_t_hat_raw << ',' << a.adapt_gate << ',' << b.adapt_gate << '\n'; }
	}
	EXPECT_TRUE(lpf_negative);
	EXPECT_TRUE(original_negative);
	EXPECT_TRUE(original_wait);
}

TEST(SlowLpfComparator, BriefOneSecondExcursionRecordsFilterResponseWithoutRanking)
{
	Episode lpf(1), original(0);
	lpf.enter(); original.enter();
	for (int k = 0; k < 1000; ++k) { lpf.step(0.18f); original.step(0.18f); }
	std::ofstream trace;
	if (const char *dir = std::getenv("SLOW_LPF_TRACE_DIR")) {
		trace.open(std::string(dir) + "/short_reversal.csv");
		trace << "cycle,total,lpf_s,slow_s,lpf_h,slow_h,slow_gate\n" << std::setprecision(9);
	}
	for (int k = 0; k < 550; ++k) {
		const float t = k < 50 ? -0.09f : 0.18f;
		const float previous_h = lpf.slow.lastResult().adapt_t_hat_raw;
		const auto a = lpf.step(t), b = original.step(t);
		EXPECT_NEAR(a.adapt_t_hat_raw, previous_h + lpf.in.dt / (5.f + lpf.in.dt) * (t - previous_h), Tolerance);
		paired(a, lpf.in.dt, lpf.in.adapt_slew_raw_per_s);
		EXPECT_FALSE(a.adapt_gate);
		if (trace.is_open()) { trace << k << ',' << t << ',' << a.transferred_i_raw << ',' << b.transferred_i_raw
			<< ',' << a.adapt_t_hat_raw << ',' << b.adapt_t_hat_raw << ',' << b.adapt_gate << '\n'; }
	}
}

TEST(SlowLpfComparator, CapsSlewConservationAndSharedAuthority)
{
	for (float hr : {0.f, 0.5f, 1.f}) {
		Episode e(1);
		e.in.headroom_release_ratio = hr;
		e.enter();
		for (int k = 0; k < 700; ++k) {
			const auto out = e.step(0.18f);
			paired(out, e.in.dt, e.in.adapt_slew_raw_per_s);
			EXPECT_LE(std::fabs(out.adapt_target_raw), e.in.hold_cap_raw);
			EXPECT_LE(std::fabs(out.transferred_i_raw), e.in.hold_cap_raw + Tolerance);
			EXPECT_NEAR(out.total_limit_raw, 0.2f + hr * std::fabs(out.transferred_i_raw), Tolerance);
		}
		// HCAP is a target cap, not an immediate state clamp on parameter shrink.
		e.in.hold_cap_raw = 0.02f;
		const auto out = e.step(0.18f);
		EXPECT_GT(out.transferred_i_raw, e.in.hold_cap_raw);
		EXPECT_NEAR(out.adapt_target_raw, 0.02f, Tolerance);
		paired(out, e.in.dt, e.in.adapt_slew_raw_per_s);
	}
}

TEST(SlowLpfComparator, NativeAcceptanceLimitKeepsPairMatched)
{
	Episode e(1);
	e.in.headroom_release_ratio = 1.f;
	e.enter();
	for (int k = 0; k < 500; ++k) { e.step(0.18f); }
	e.in.hold_cap_raw = 0.f;
	const auto out = e.step(0.2f + e.slow.transferredRaw());
	EXPECT_LT(out.requested_delta_s_raw, 0.f);
	EXPECT_TRUE(out.transfer_limited);
	EXPECT_NEAR(out.accepted_delta_s_raw, 0.f, Tolerance);
	paired(out, e.in.dt, e.in.adapt_slew_raw_per_s);
}

TEST(SlowLpfComparator, InvalidModeUsesSafetyExitAndCannotEnter)
{
	for (int32_t invalid : {-1, 2, 256}) {
		Episode fresh(invalid);
		EXPECT_EQ(fresh.slow.update(fresh.in, fresh.rc).state, Slow::State::Disabled);
		Episode e(1);
		e.enter();
		e.in.comparator_mode = invalid;
		const auto out = e.step(0.18f);
		EXPECT_EQ(out.state, Slow::State::SafetyExit);
		EXPECT_FALSE(out.comparator_active);
		EXPECT_LT(out.transferred_i_raw, out.transferred_i_before_raw);
	}
}

TEST(SlowLpfComparator, InvalidAdaptiveConfigurationRetainsExistingProtection)
{
	Episode e(1);
	e.enter();
	e.in.adapt_tau_s = std::numeric_limits<float>::quiet_NaN();
	const auto out = e.step(0.18f);
	EXPECT_TRUE(out.adapt_limited);
	EXPECT_FALSE(out.comparator_active);
	EXPECT_EQ(out.transferred_i_raw, out.transferred_i_before_raw);
}

TEST(SlowLpfComparator, ResetDisableAndSafetyUseExistingPaths)
{
	for (int event = 0; event < 6; ++event) {
		Episode a(0), b(1);
		a.enter(); b.enter();
		if (event == 0) { a.in.enabled = b.in.enabled = false; }
		if (event == 1) { a.in.pilot_abort = b.in.pilot_abort = true; }
		if (event == 2) { a.in.failsafe = b.in.failsafe = true; }
		if (event == 3) { a.in.hard_reset = b.in.hard_reset = true; }
		if (event == 4) { a.in.config_valid = b.in.config_valid = false; }
		if (event == 5) { a.rc.resetIntegral(0); b.rc.resetIntegral(0); }
		for (int k = 0; k < 200; ++k) {
			const auto x = a.slow.update(a.in, a.rc), y = b.slow.update(b.in, b.rc);
			EXPECT_EQ(x.state, y.state);
			EXPECT_EQ(x.transferred_i_raw, y.transferred_i_raw);
			EXPECT_EQ(x.residual_i_raw, y.residual_i_raw);
			EXPECT_EQ(x.reset_required, y.reset_required);
			EXPECT_FALSE(y.comparator_active);
			if (x.reset_required) {
				a.rc.resetIntegral(0); b.rc.resetIntegral(0);
				a.slow.synchronizeReset(a.rc.rollIntegralResetEpoch());
				b.slow.synchronizeReset(b.rc.rollIntegralResetEpoch());
			}
		}
		EXPECT_NEAR(b.slow.transferredRaw(), 0.f, Tolerance);
	}
}

TEST(SlowLpfComparator, NativeSaturationSuppressionStillApplies)
{
	Episode e(1);
	e.enter();
	for (float sign : {-1.f, 1.f}) {
		e.rc.setPositiveSaturationFlag(0, sign > 0.f);
		e.rc.setNegativeSaturationFlag(0, sign < 0.f);
		for (int k = 0; k < 30; ++k) {
			const auto out = e.slow.update(e.in, e.rc);
			paired(out, e.in.dt, e.in.adapt_slew_raw_per_s);
			e.rc.setRollITransferContext(e.slow.transferredRaw(), out.headroom_release_ratio_effective, true);
			const float before = e.rc.rollIntegralRaw();
			e.rc.update(Vector3f(), Vector3f(sign, 0.f, 0.f), Vector3f(), e.in.dt, false);
			EXPECT_EQ(e.rc.rollIntegralRaw(), before);
		}
	}
}

TEST(SlowLpfComparator, ContinuousCrossingUsesRemainingSlewWithoutZeroStop)
{
	Episode e(1);
	e.enter();
	for (int k = 0; k < 1000; ++k) { e.step(0.18f); }
	bool checked = false;
	for (int k = 0; k < 2200; ++k) {
		const auto out = e.step(-0.09f);
		// A continuous step may cross without ever storing exactly zero.
		if (out.transferred_i_before_raw > 0.f && out.transferred_i_raw < 0.f) {
			EXPECT_LT(out.adapt_target_raw, 0.f);
			EXPECT_NEAR(out.requested_delta_s_raw, -e.in.adapt_slew_raw_per_s * e.in.dt, Tolerance);
			EXPECT_LT(out.accepted_delta_s_raw, -out.transferred_i_before_raw);
			EXPECT_LT(out.accepted_delta_s_raw, 0.f);
			EXPECT_LT(out.transferred_i_raw, 0.f);
			EXPECT_FALSE(out.adapt_gate);
			EXPECT_FALSE(out.reversal_logic_active);
			checked = true;
			break;
		}
	}
	EXPECT_TRUE(checked);
}

TEST(SlowLpfComparator, AdaptiveDisableDoesNotSilentlyCreateDifferentEntry)
{
	Episode a(0), b(1);
	a.in.adapt_enabled = b.in.adapt_enabled = false;
	a.enter(); b.enter();
	for (int k = 0; k < 100; ++k) {
		const auto x = a.step(0.18f), y = b.step(0.18f);
		EXPECT_FALSE(y.comparator_active);
		EXPECT_FALSE(y.adapt_enabled);
		EXPECT_EQ(x.transferred_i_raw, y.transferred_i_raw);
	}
}

TEST(SlowLpfComparator, NonfiniteAndInvalidTotalUseSameRecovery)
{
	for (int mode : {0, 1}) {
		for (bool nan : {false, true}) {
			Episode e(mode);
			e.enter();
			e.in.imax_raw = nan ? std::numeric_limits<float>::quiet_NaN() : 0.01f;
			const auto out = e.slow.update(e.in, e.rc);
			EXPECT_TRUE(out.reset_required);
			EXPECT_EQ(out.state, Slow::State::RecoveryReconciliation);
			EXPECT_FALSE(out.comparator_active);
		}
	}
}
