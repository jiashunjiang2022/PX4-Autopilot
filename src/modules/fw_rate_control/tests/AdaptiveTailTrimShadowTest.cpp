#include <gtest/gtest.h>
#include "../AdaptiveTailTrimShadow.hpp"
#include "../BumplessRollITransfer.hpp"

namespace {
using Shadow = AdaptiveTailTrimShadow;
constexpr float Tol = 2e-6f;
Shadow::Config config() { Shadow::Config c{}; c.window = .5f; c.tau = .1f; c.slew = .05f; return c; }
Shadow::Inputs input() {
	Shadow::Inputs in{};
	in.now = in.actuator_timestamp = 1000000; in.dt = .02f; in.g = .7f;
	in.i_actual = .15f; in.s_actual = .04f; in.imax = .3f;
	in.enabled = in.armed = in.control_valid = in.setpoint_valid = in.mapping_valid = true;
	in.landed = false; return in;
}
Shadow::Result tick(Shadow &s, Shadow::Inputs &in, int count = 1) {
	Shadow::Result r{};
	for (int n = 0; n < count; ++n) { in.now += 20000; in.actuator_timestamp = in.now; r = s.update(in); }
	return r;
}
}

TEST(TailTrimEstimator, CausalLpf) {
	AdaptiveTailTrimEstimator e; AdaptiveTailTrimEstimator::Config c{}; c.tau = 1.f;
	EXPECT_FLOAT_EQ(e.update(.1f, .02f, true, true, c).b_hat, .1f);
	EXPECT_NEAR(e.update(.2f, .02f, true, true, c).b_hat, .1f+.1f*.02f/1.02f, Tol);
}
TEST(TailTrimEstimator, FullWindowAndInvalidFreshRestart) {
	AdaptiveTailTrimEstimator e; AdaptiveTailTrimEstimator::Config c{}; c.window = .5f;
	for (int n = 0; n < 24; ++n) { EXPECT_FALSE(e.update(.1f, .02f, true, true, c).gate); }
	EXPECT_TRUE(e.update(.1f, .02f, true, true, c).gate);
	EXPECT_EQ(e.update(.1f, .02f, false, true, c).sample_count, 0);
	for (int n = 0; n < 24; ++n) { EXPECT_FALSE(e.update(.1f, .02f, true, true, c).gate); }
	EXPECT_TRUE(e.update(.1f, .02f, true, true, c).gate);
}
TEST(TailTrimEstimator, ManeuverClearsEvidence) {
	AdaptiveTailTrimEstimator e; AdaptiveTailTrimEstimator::Config c{}; c.window = .5f;
	for (int n = 0; n < 30; ++n) { e.update(.1f, .02f, true, true, c); }
	EXPECT_EQ(e.update(.1f, .02f, true, false, c).sample_count, 0);
	EXPECT_FALSE(e.update(.1f, .02f, true, true, c).gate);
}
TEST(TailTrimEstimator, StabilityAndSign) {
	AdaptiveTailTrimEstimator e; AdaptiveTailTrimEstimator::Config c{}; c.window = .5f; c.std_limit = .001f;
	AdaptiveTailTrimEstimator::Result r{};
	for (int n = 0; n < 100; ++n) { r = e.update(n%2 ? .1f : .3f, .05f, true, true, c); }
	EXPECT_FALSE(r.gate); EXPECT_GT(r.std, .05f);
	c.std_limit = 1.f; e.reset();
	for (int n = 0; n < 100; ++n) { r = e.update(n%2 ? -.1f : .1f, .05f, true, true, c); }
	EXPECT_FALSE(r.gate); EXPECT_LT(r.same_sign_fraction, .9f);
}
TEST(TailTrimEstimator, TenSecondCapacityAndNoCatchup) {
	AdaptiveTailTrimEstimator e; AdaptiveTailTrimEstimator::Config c{}; c.window = 10.f;
	for (int n = 0; n < 199; ++n) { EXPECT_FALSE(e.update(.1f, .05f, true, true, c).gate); }
	EXPECT_TRUE(e.update(.1f, .05f, true, true, c).gate);
	EXPECT_FALSE(e.update(.1f, .5f, true, true, c).valid);
}
TEST(TailTrimManeuver, BankAndRateEntry) {
	AdaptiveTailTrimManeuverGate g;
	EXPECT_FALSE(g.update(0.f, 0.f, .02f, true));
	EXPECT_TRUE(g.update(.2f, 0.f, .02f, true)); g.reset();
	EXPECT_TRUE(g.update(0.f, -.2f, .02f, true));
}
TEST(TailTrimManeuver, HysteresisDwellAndNoChatter) {
	AdaptiveTailTrimManeuverGate g; ASSERT_TRUE(g.update(.3f, 0.f, .02f, true));
	for (int n = 0; n < 100; ++n) { EXPECT_TRUE(g.update(n%2 ? .16f : .18f, 0.f, .02f, true)); }
	for (int n = 0; n < 49; ++n) { EXPECT_TRUE(g.update(0.f, 0.f, .02f, true)); }
	EXPECT_FALSE(g.update(0.f, 0.f, .02f, true));
}
TEST(TailTrimManeuver, InvalidBlocksAndRestartsDwell) {
	AdaptiveTailTrimManeuverGate g; EXPECT_TRUE(g.update(NAN, 0.f, .02f, false));
	EXPECT_TRUE(g.update(0.f, 0.f, .02f, true));
}
TEST(TailTrimPitch, PreReversalGeometryAttackDecay) {
	CausalTailPitchEnvelope p;
	auto r = p.update(.2f, .6f, 100, 100, .02f, true);
	ASSERT_TRUE(r.valid); EXPECT_NEAR(r.raw, .4f, Tol); EXPECT_NEAR(r.roll, .2f, Tol);
	EXPECT_NEAR(r.envelope, .4f, Tol);
	r = p.update(0.f, 0.f, 200, 200, .02f, true); EXPECT_NEAR(r.envelope, .4f*expf(-.02f), Tol);
	r = p.update(.8f, .8f, 300, 300, .02f, true); EXPECT_GE(r.envelope, fabsf(r.raw));
	p.reset(); EXPECT_FLOAT_EQ(p.update(0.f, 0.f, 400, 400, .02f, true).envelope, 0.f);
}
TEST(TailTrimPitch, DistinctInvalidReasons) {
	CausalTailPitchEnvelope p;
	EXPECT_TRUE(p.update(0.f, 0.f, 1, 300000, .02f, true).invalid & CausalTailPitchEnvelope::Stale);
	EXPECT_TRUE(p.update(NAN, 0.f, 1, 1, .02f, true).invalid & CausalTailPitchEnvelope::Nonfinite);
	EXPECT_TRUE(p.update(0.f, 0.f, 1, 1, .02f, false).invalid & CausalTailPitchEnvelope::Mapping);
	EXPECT_TRUE(p.update(0.f, 0.f, 2, 1, .02f, true).invalid & CausalTailPitchEnvelope::Future);
	EXPECT_TRUE(p.update(2.f, 0.f, 1, 1, .02f, true).invalid & CausalTailPitchEnvelope::Range);
}
TEST(TailTrimShadow, InitializationAndObservation) {
	Shadow s; s.configure(config()); auto in = input(); auto r = tick(s, in);
	EXPECT_TRUE(r.reset); EXPECT_FLOAT_EQ(r.b_shadow, 0.f);
	r = tick(s, in); EXPECT_NEAR(r.b_obs, .7f*.19f/1.1f, Tol);
	in.s_actual = 0.f; r = tick(s, in); EXPECT_NEAR(r.b_obs, .7f*.15f/1.1f, Tol);
}
TEST(TailTrimShadow, GrowthVirtualOffsetAndConservation) {
	Shadow s; s.configure(config()); auto in = input(); tick(s, in);
	for (int n = 0; n < 100; ++n) {
		const auto r = tick(s, in); ASSERT_TRUE(r.valid);
		EXPECT_NEAR(r.transfer.transfer_mismatch, 0.f, Tol);
		EXPECT_NEAR(r.i_shadow, r.t_actual+r.offset, Tol);
		EXPECT_NEAR(r.b_total, r.b_obs, Tol);
		EXPECT_FLOAT_EQ(r.actual_tail_trim_torque, 0.f);
	}
	auto r = tick(s, in); EXPECT_GT(r.b_shadow, .05f); EXPECT_LT(r.offset, 0.f);
	const float offset = r.offset; in.i_actual += .01f; in.phi_sp = .3f;
	r = tick(s, in); EXPECT_FLOAT_EQ(r.offset, offset); EXPECT_NEAR(r.i_shadow, .20f+offset, Tol);
}
TEST(TailTrimShadow, NormalManeuverFreezesAndFreshExit) {
	Shadow s; s.configure(config()); auto in = input(); auto before = tick(s, in, 100);
	ASSERT_GT(before.b_shadow, 0.f); in.phi_sp = .3f; in.i_actual = 0.f; in.s_actual = 0.f;
	auto r = tick(s, in, 100); EXPECT_FLOAT_EQ(r.b_shadow, before.b_shadow);
	EXPECT_FLOAT_EQ(r.target, before.target); EXPECT_EQ(r.estimate.sample_count, 0);
	in.phi_sp = 0.f; tick(s, in, 50); r = tick(s, in); EXPECT_FALSE(r.estimate.gate);
}
TEST(TailTrimShadow, PitchDoesNotEnterEstimator) {
	Shadow a, b; a.configure(config()); b.configure(config()); auto x = input(), y = x;
	y.left = y.right = .95f;
	for (int n = 0; n < 100; ++n) {
		auto ra = tick(a, x), rb = tick(b, y);
		EXPECT_FLOAT_EQ(ra.estimate.b_hat, rb.estimate.b_hat); EXPECT_EQ(ra.estimate.gate, rb.estimate.gate);
	}
	EXPECT_GT(tick(a, x).b_shadow, 0.f); auto r = tick(b, y);
	EXPECT_FLOAT_EQ(r.b_shadow, 0.f); EXPECT_TRUE(r.transfer.limited_by_shared_axis);
}
TEST(TailTrimShadow, InvalidPitchPreservesBiasEvidenceButReleases) {
	Shadow s; s.configure(config()); auto in = input(); auto r = tick(s, in, 100);
	const float before = r.b_shadow; in.left = NAN; r = tick(s, in);
	EXPECT_FALSE(r.pitch.valid); EXPECT_TRUE(r.estimate.gate); EXPECT_LT(r.b_shadow, before);
	EXPECT_TRUE(r.transfer.safety_release_active);
}
TEST(TailTrimShadow, SafetyOverridesManeuver) {
	Shadow s; s.configure(config()); auto in = input(); auto r = tick(s, in, 100);
	const float before = r.b_shadow; in.phi_sp = .3f; in.safety = true;
	r = tick(s, in); EXPECT_LT(r.b_shadow, before); EXPECT_TRUE(r.transfer.safety_release_active);
}
TEST(TailTrimShadow, ZeroFirstAndFullFreshGate) {
	Shadow s; s.configure(config()); auto in = input(); auto r = tick(s, in, 100);
	ASSERT_GT(r.b_shadow, 0.f); in.i_actual = -.15f; in.s_actual = -.04f; in.imax = 1.f;
	bool zero = false; int after = 0;
	for (int n = 0; n < 500; ++n) {
		r = tick(s, in);
		if (!zero) { EXPECT_GE(r.b_shadow, 0.f); }
		if (r.transfer.reversal_reached_zero) { zero = true; EXPECT_FALSE(r.estimate.gate); continue; }
		if (zero && ++after < 25) { EXPECT_FALSE(r.estimate.gate); EXPECT_FLOAT_EQ(r.b_shadow, 0.f); }
	}
	EXPECT_TRUE(zero); EXPECT_LT(r.b_shadow, 0.f);
}
TEST(TailTrimShadow, InvalidGainAndBoundsReset) {
	for (int scenario = 0; scenario < 4; ++scenario) {
		Shadow s; s.configure(config()); auto in = input(); ASSERT_GT(tick(s, in, 100).b_shadow, 0.f);
		if (scenario == 0) { in.g = NAN; }
		if (scenario == 1) { in.g = .001f; }
		if (scenario == 2) { in.i_actual = 2.f; }
		if (scenario == 3) { in.epoch++; }
		auto r = tick(s, in); EXPECT_FALSE(r.valid); EXPECT_TRUE(r.reset);
		EXPECT_FLOAT_EQ(r.b_shadow, 0.f); EXPECT_FLOAT_EQ(r.offset, 0.f);
	}
}
TEST(TailTrimShadow, DisableDisarmLandedReenable) {
	for (int scenario = 0; scenario < 3; ++scenario) {
		Shadow s; s.configure(config()); auto in = input(); ASSERT_GT(tick(s, in, 100).b_shadow, 0.f);
		if (scenario == 0) { in.enabled = false; }
		if (scenario == 1) { in.armed = false; }
		if (scenario == 2) { in.landed = true; }
		auto r = tick(s, in); EXPECT_FLOAT_EQ(r.b_shadow, 0.f); EXPECT_FLOAT_EQ(r.offset, 0.f);
		in.enabled = in.armed = true; in.landed = false;
		r = tick(s, in); EXPECT_FALSE(r.estimate.gate); EXPECT_FLOAT_EQ(r.b_shadow, 0.f);
	}
}
TEST(TailTrimShadow, FrozenGainAndCurrentNaturalBurden) {
	Shadow s; s.configure(config()); auto in = input(); tick(s, in, 100);
	in.g = .9f; auto r = tick(s, in);
	EXPECT_FLOAT_EQ(r.transfer.g_used, .9f); EXPECT_NEAR(r.transfer.transfer_mismatch, 0.f, Tol);
	EXPECT_NEAR(r.b_obs, .9f*.19f/1.1f, Tol);
}
TEST(TailTrimShadow, RealObjectsUnchanged) {
	RateControl rc; rc.setIntegratorLimit(matrix::Vector3f(.3f,.3f,.3f));
	rc.setPidGains(matrix::Vector3f(),matrix::Vector3f(1.f,0.f,0.f),matrix::Vector3f());
	rc.update(matrix::Vector3f(),matrix::Vector3f(.1f,0.f,0.f),matrix::Vector3f(),.02f,false);
	BumplessRollITransfer b2b; const float actual_i = rc.rollIntegralRaw(), actual_s = b2b.transferredRaw();
	const auto epoch = rc.rollIntegralResetEpoch(); Shadow s; s.configure(config()); auto in = input();
	in.i_actual = actual_i; in.s_actual = actual_s; tick(s, in, 100);
	EXPECT_FLOAT_EQ(rc.rollIntegralRaw(), actual_i); EXPECT_FLOAT_EQ(b2b.transferredRaw(), actual_s);
	EXPECT_EQ(rc.rollIntegralResetEpoch(), epoch);
}
