// Future contracts, not a controller implementation. Deliberately reuse the
// unchanged V3 fixtures AND register its 72 regressions in this opt-in binary.
// Select the 16 probes with --gtest_filter=AdaptiveTailTrimContract.*.
#include "BumplessRollITransferTest.cpp"
#include <type_traits>
#include <utility>

namespace {
constexpr float KA = 1.1f;
constexpr float PhysicalTolerance = 2e-6f; // fixed before execution
template<typename T, typename = void> struct HasReserves : std::false_type {};
template<typename T> struct HasReserves<T, decltype(void(std::declval<T>().roll_reserve_pos),
	void(std::declval<T>().roll_reserve_neg))> : std::true_type {};
template<typename T, typename = void> struct HasManeuver : std::false_type {};
template<typename T> struct HasManeuver<T, decltype(void(std::declval<T>().learning_allowed))> : std::true_type {};
template<typename T, typename = void> struct HasGeometry : std::false_type {};
template<typename T> struct HasGeometry<T, decltype(void(std::declval<T>().tail_pitch_context),
	void(std::declval<T>().residual_roll_tail))> : std::true_type {};
struct Probe {
	RateControl rc;
	BumplessRollITransfer transfer;
	BumplessRollITransfer::Inputs in = enabled_inputs();
	Probe(float i = .12f, float s = .04f) {
		configure(rc); seed_residual(rc, i);
		BumplessRollITransferTestAccess::setState(transfer, s, rc.rollIntegralResetEpoch());
	}
};
}

TEST(AdaptiveTailTrimContract, RED01_PhysicalTrimMeaningIndependentOfG)
{
	Probe a, b; a.in.g_current = .7f; b.in.g_current = 1.4f;
	const auto x = a.transfer.update(a.in, a.rc), y = b.transfer.update(b.in, b.rc);
	ASSERT_NEAR(x.transferred_i_raw, y.transferred_i_raw, PhysicalTolerance);
	// Probe whether unchanged S can be relabelled b. This is NOT a V3 bug.
	EXPECT_NEAR(x.effective_slow_tail, y.effective_slow_tail, PhysicalTolerance);
}
TEST(AdaptiveTailTrimContract, RED02_BumplessTransferUsesPhysicalTorqueInvariant)
{
	Probe p; p.in.enabled = false;
	const auto r = p.transfer.update(p.in, p.rc);
	ASSERT_GT(fabsf(r.accepted_delta_s_raw), 0.f);
	ASSERT_NEAR(r.accepted_delta_i_raw + r.accepted_delta_s_raw, 0.f, PhysicalTolerance);
	EXPECT_NEAR(p.in.g_current*r.accepted_delta_i_raw + KA*r.accepted_delta_s_raw, 0.f, PhysicalTolerance);
}
TEST(AdaptiveTailTrimContract, RED03_PhysicalBMaxIsTailCoordinateBound)
{
	Probe p(.1f, .1f); p.in.cap_raw = .1f; p.in.g_current = 2.2f;
	const auto r = p.transfer.update(p.in, p.rc);
	ASSERT_LE(fabsf(r.transferred_i_raw), .1f);
	EXPECT_LE(fabsf(r.effective_slow_tail), .1f + PhysicalTolerance);
}
TEST(AdaptiveTailTrimContract, RED04_SharedAxisReserveBlocksTrimGrowth)
{
	Probe p(.18f, 0.f); set_adaptive_defaults(p.in, 0);
	BumplessRollITransferTestAccess::configureAdaptiveHold(p.transfer, 0.f, p.rc.rollIntegralResetEpoch(), .18f, 0.f);
	const float pitch = .97f, reserve = .05f;
	ASSERT_LT(1.f-pitch, reserve);
	// Current API has no pitch input; genuine updates ignore this external context.
	for (int i = 0; i < 300; ++i) { p.transfer.update(p.in, p.rc); }
	ASSERT_TRUE(p.transfer.lastResult().adapt_gate);
	EXPECT_NEAR(p.transfer.transferredRaw(), 0.f, PhysicalTolerance);
}
TEST(AdaptiveTailTrimContract, RED05_PositiveNegativeResidualReserveReportedSeparately)
{ EXPECT_TRUE((HasReserves<BumplessRollITransfer::Result>::value)); }
TEST(AdaptiveTailTrimContract, RED06_GrowthRequiresGate)
{
	Probe p(.18f, 0.f); set_adaptive_defaults(p.in, 0);
	BumplessRollITransferTestAccess::configureAdaptiveHold(p.transfer, 0.f, p.rc.rollIntegralResetEpoch(), .18f, 0.f);
	const auto r = p.transfer.update(p.in, p.rc);
	EXPECT_FALSE(r.adapt_gate); EXPECT_NEAR(r.transferred_i_raw, 0.f, PhysicalTolerance);
}
TEST(AdaptiveTailTrimContract, RED07_ReleaseDoesNotRequireGate)
{
	Probe p(0.f, .08f); set_adaptive_defaults(p.in, 0);
	BumplessRollITransferTestAccess::configureAdaptiveHold(p.transfer, .08f, p.rc.rollIntegralResetEpoch(), .08f, 0.f);
	const auto r = p.transfer.update(p.in, p.rc);
	EXPECT_FALSE(r.adapt_gate); EXPECT_LT(r.transferred_i_raw, .08f);
}
TEST(AdaptiveTailTrimContract, RED08_ReversalMustReachZeroFirst)
{
	Probe p(-.14f, .04f); set_adaptive_defaults(p.in, 0);
	BumplessRollITransferTestAccess::configureAdaptiveHold(p.transfer, .04f, p.rc.rollIntegralResetEpoch(), -.1f, 0.f);
	bool zero = false;
	for (int i = 0; i < 300; ++i) {
		const auto r = p.transfer.update(p.in, p.rc);
		if (!zero) { EXPECT_GE(r.transferred_i_raw, -PhysicalTolerance); }
		zero |= fabsf(r.transferred_i_raw) < 1e-7f;
	}
	EXPECT_TRUE(zero);
}
TEST(AdaptiveTailTrimContract, RED09_ReversalRequiresFreshFullGateAfterZero)
{
	Probe p(.18f, 0.f); set_adaptive_defaults(p.in, 0);
	p.transfer.synchronizeReset(p.rc.rollIntegralResetEpoch());
	for (int i = 0; i < 800; ++i) { p.transfer.update(p.in, p.rc); }
	ASSERT_GT(p.transfer.transferredRaw(), .09f);
	int zero = -1, gate = -1;
	for (int i = 0; i < 1600; ++i) {
		const float s = p.transfer.transferredRaw(); set_residual(p.rc, -.1f-s, s);
		const auto r = p.transfer.update(p.in, p.rc);
		if (zero < 0 && fabsf(r.transferred_i_raw) <= 1e-7f) { zero = i; }
		if (zero >= 0 && i-zero < 150) {
			EXPECT_FALSE(r.adapt_gate); EXPECT_NEAR(r.transferred_i_raw, 0.f, PhysicalTolerance);
		}
		if (zero >= 0 && gate < 0 && r.adapt_gate) { gate = i; }
	}
	ASSERT_GE(zero, 0); EXPECT_EQ(gate-zero, 150);
}
TEST(AdaptiveTailTrimContract, RED10_ManeuverLearningCanBeDisabledOrFrozen)
{ EXPECT_TRUE((HasManeuver<BumplessRollITransfer::Inputs>::value)); }
TEST(AdaptiveTailTrimContract, RED11_NormalExitExactHandbackWhenFeasible)
{
	Probe p; p.in.enabled = false;
	const float before = p.in.g_current*(p.rc.rollIntegralRaw()+p.transfer.transferredRaw());
	for (int i = 0; i < 20; ++i) { p.transfer.update(p.in, p.rc); }
	EXPECT_NEAR(p.transfer.transferredRaw(), 0.f, PhysicalTolerance);
	EXPECT_NEAR(p.in.g_current*p.rc.rollIntegralRaw(), before, PhysicalTolerance);
}
TEST(AdaptiveTailTrimContract, RED12_NormalExitDecaysAuthorityWhenHandbackImpossible)
{
	Probe p(.2f, .04f);
	BumplessRollITransferTestAccess::setState(p.transfer, .04f, p.rc.rollIntegralResetEpoch(), 1.f);
	p.in.enabled = false; p.in.headroom_release_ratio = 1.f;
	for (int i = 0; i < 20; ++i) {
		const float before = p.transfer.transferredRaw(); const auto r = p.transfer.update(p.in, p.rc);
		EXPECT_LE(fabsf(p.rc.rollIntegralRaw()), .2f);
		EXPECT_LE(fabsf(r.transferred_i_raw-before), p.in.slew_raw_per_s*p.in.dt+PhysicalTolerance);
	}
	EXPECT_NEAR(p.transfer.transferredRaw(), 0.f, PhysicalTolerance);
}
TEST(AdaptiveTailTrimContract, RED13_SafetyExitFinite)
{
	Probe p; p.in.pilot_abort = true;
	for (int i = 0; i < 4; ++i) { p.transfer.update(p.in, p.rc); }
	EXPECT_NEAR(p.transfer.transferredRaw(), 0.f, PhysicalTolerance);
}
TEST(AdaptiveTailTrimContract, RED14_ResetMismatchCannotLeaveHiddenTrim)
{
	Probe p; p.rc.resetIntegral(); const auto r = p.transfer.update(p.in, p.rc);
	EXPECT_TRUE(r.reset_mismatch); ASSERT_TRUE(r.reset_required);
	// Match the real caller's recovery handshake (FixedwingRateControl.cpp:405).
	p.rc.resetIntegral(0);
	p.transfer.synchronizeReset(p.rc.rollIntegralResetEpoch());
	EXPECT_NEAR(p.transfer.transferredRaw(), 0.f, PhysicalTolerance);
}
TEST(AdaptiveTailTrimContract, RED15_NonfiniteGCannotDivide)
{
	for (float g : {0.f, -1.f, NAN, INFINITY}) {
		Probe p; p.in.g_current = g; const auto r = p.transfer.update(p.in, p.rc);
		EXPECT_TRUE(std::isfinite(r.effective_slow_torque));
		EXPECT_EQ(r.state, BumplessRollITransfer::State::RecoveryReconciliation);
	}
}
TEST(AdaptiveTailTrimContract, RED16_NoPhysicalTrimBeyondServoGeometry)
{ EXPECT_TRUE((HasGeometry<BumplessRollITransfer::Inputs>::value)); }
