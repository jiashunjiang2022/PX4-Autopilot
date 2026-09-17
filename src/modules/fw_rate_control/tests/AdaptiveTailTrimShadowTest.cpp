#include <gtest/gtest.h>
#include <chrono>
#include <algorithm>
#include "../AdaptiveTailTrimShadow.hpp"
#include "../BumplessRollITransfer.hpp"
#include <uORB/topics/vehicle_torque_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/manual_control_setpoint.h>

namespace {
using Shadow = AdaptiveTailTrimShadow;
constexpr float Tol = 2e-6f;
Shadow::Config config() { Shadow::Config c{}; c.window = .5f; c.tau = .1f; c.slew = .05f; return c; }
Shadow::Inputs input() {
	Shadow::Inputs in{};
	in.now = in.actuator_timestamp = 1000000; in.dt = .02f; in.g = .7f;
	in.i_actual = .15f; in.s_actual = .04f; in.imax = .3f;
	in.enabled = in.armed = in.control_valid = in.setpoint_valid = in.mapping_valid = true;
	in.landed = false; in.mission_eligible = true; return in;
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
	ASSERT_TRUE(e.update(.1f, .02f, true, true, c).gate);
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

namespace {
// Closest-possible module test: genuine RateControl and B2B, production output
// block extracted at configure time. Gain source and parameter cache are mocks;
// this tests arithmetic and state isolation, not scheduler/uORB delivery.
struct CompositionHarness {
	using Vector3f = matrix::Vector3f;
	RateControl _rate_control;
	BumplessRollITransfer b2b;
	struct Gain {
		Vector3f value{.7f,.8f,.9f};
		Vector3f getGains() const { return value; }
		void update(Vector3f, float) { value(0) = .8f; }
	} _gain_compression;
	struct Param { float get() const { return 1.f; } } _param_fw_acro_yaw_en, _param_fw_man_y_sc;
	vehicle_control_mode_s _vcontrol_mode{};
	manual_control_setpoint_s _manual_control_setpoint{};
	vehicle_torque_setpoint_s _vehicle_torque_setpoint{};
	float _airspeed_scaling{1.f}, _b2b_g_current{0.f}, _b2b_roll_baseline{0.f};
	bool _b2b_total_clipped{false};
	void resetIntegralAndTransfer() { _rate_control.resetIntegral(); b2b.synchronizeReset(_rate_control.rollIntegralResetEpoch()); }
	CompositionHarness() {
		_rate_control.setIntegratorLimit(Vector3f(.3f,.3f,.3f));
		_rate_control.setPidGains(Vector3f(.1f,.1f,.1f),Vector3f(.01f,0.f,0.f),Vector3f());
		RateControl::RollITransferRequest seed{}; seed.requested_delta_raw = .18f;
		seed.mode = RateControl::RollITransferMode::PairPreserving;
		_rate_control.applyRollITransfer(seed);
	}
	void cycle(bool slow) {
		BumplessRollITransfer::Inputs in{};
		in.enabled = in.eligible = slow; in.rates_enabled = in.config_valid = true;
		in.dt = .02f; in.imax_raw = .3f; in.cap_raw = .05f; in.slew_raw_per_s = .05f;
		in.safety_slew_raw_per_s = .1f; in.g_current = _gain_compression.getGains()(0);
		b2b.update(in, _rate_control);
		const float transferred_roll_i_raw = b2b.transferredRaw();
		const float dt = .02f, airspeed_scale_squared = _airspeed_scaling*_airspeed_scaling;
		const Vector3f trim(.01f,.02f,.03f);
		Vector3f angular_acceleration_setpoint = _rate_control.update(Vector3f(),Vector3f(.05f,0.f,0.f),Vector3f(),dt,false);
#include "TailTrimActualComposition.inc"
	}
};
}

TEST(TailTrimIsolation, ActualCompositionExactOffOnAndEpoch) {
	for (bool slow : {false, true}) {
		CompositionHarness off, on; Shadow a, b; a.configure(config()); b.configure(config());
		auto ia = input(), ib = input(); ia.enabled = false;
		float max_shadow = 0.f;
		for (int n = 0; n < 200; ++n) {
			off.cycle(slow); on.cycle(slow);
			const float i_before = on._rate_control.rollIntegralRaw(), s_before = on.b2b.transferredRaw();
			ia.g = off._b2b_g_current; ib.g = on._b2b_g_current;
			ia.i_actual = off._rate_control.rollIntegralRaw(); ib.i_actual = i_before;
			ia.s_actual = off.b2b.transferredRaw(); ib.s_actual = s_before;
			tick(a, ia); const auto r = tick(b, ib);
			for (int axis = 0; axis < 3; ++axis) { EXPECT_FLOAT_EQ(on._vehicle_torque_setpoint.xyz[axis], off._vehicle_torque_setpoint.xyz[axis]); }
			EXPECT_FLOAT_EQ(on._rate_control.rollIntegralRaw(), off._rate_control.rollIntegralRaw());
			EXPECT_FLOAT_EQ(on._rate_control.rollIntegralRaw(), i_before);
			EXPECT_FLOAT_EQ(on.b2b.transferredRaw(), s_before);
			if (!r.reset) { EXPECT_FLOAT_EQ(r.t_actual, i_before+s_before); }
			max_shadow = fmaxf(max_shadow, r.b_shadow);
		}
		EXPECT_GT(max_shadow, .01f);
		if (slow) { EXPECT_GT(on.b2b.transferredRaw(), 0.f); }
	}
}

TEST(TailTrimShadow, PriorReversalCanUnwindDuringManeuver) {
	Shadow s; s.configure(config()); auto in = input(); tick(s, in, 100);
	in.i_actual = -.15f; in.s_actual = -.04f; in.imax = 1.f;
	Shadow::Result r{}; bool latched = false;
	for (int n = 0; n < 100; ++n) { r = tick(s,in); if (r.reversal) { latched = true; break; } }
	ASSERT_TRUE(latched); const float before = r.b_shadow; in.phi_sp = .3f;
	r = tick(s,in); EXPECT_LT(r.b_shadow,before);
	r = tick(s,in,200); EXPECT_FLOAT_EQ(r.b_shadow,0.f); EXPECT_FALSE(r.estimate.gate);
}
TEST(TailTrimShadow, InvalidConfigurationAndTimeGapClearAll) {
	Shadow s; s.configure(config()); auto in = input(); ASSERT_GT(tick(s,in,100).b_shadow,0.f);
	in.now += 500000; auto r = tick(s,in); EXPECT_TRUE(r.reset); EXPECT_FLOAT_EQ(r.b_shadow,0.f);
	tick(s,in,100); auto c = config(); c.bmax = .2f; s.configure(c);
	r = tick(s,in); EXPECT_TRUE(r.reset); EXPECT_FLOAT_EQ(r.offset,0.f);
}
TEST(TailTrimEstimator, CausalPrefixAndControllerRates) {
	for (float dt : {.002f,.01f,.02f,.04f}) {
		AdaptiveTailTrimEstimator a,b; AdaptiveTailTrimEstimator::Config c{}; c.window = 1.f;
		float elapsed = 0.f; AdaptiveTailTrimEstimator::Result r{};
		while (elapsed < 1.f-1e-5f) {
			r = a.update(.1f,dt,true,true,c); auto same = b.update(.1f,dt,true,true,c);
			EXPECT_FLOAT_EQ(r.b_hat,same.b_hat); elapsed += dt;
			if (elapsed < 1.f-1e-5f) { EXPECT_FALSE(r.gate); }
		}
		EXPECT_TRUE(r.gate);
		b.update(-.1f,dt,true,true,c); EXPECT_NEAR(r.b_hat,.1f,Tol);
	}
}

TEST(TailTrimShadow, EquivalentActualRedistributionAndPhysicalStdConversion) {
	Shadow a,b; auto cfg = config(); cfg.std_raw = .001f; a.configure(cfg); b.configure(cfg);
	auto x = input(), y = x; y.i_actual += y.s_actual; y.s_actual = 0.f;
	for (int n = 0; n < 100; ++n) {
		const float step = n%2 ? .01f : -.01f;
		x.i_actual = .15f+step; y.i_actual = .19f+step;
		auto ra = tick(a,x), rb = tick(b,y);
		EXPECT_NEAR(ra.b_obs, rb.b_obs, Tol); EXPECT_NEAR(ra.estimate.b_hat, rb.estimate.b_hat, Tol);
	}
	auto r = tick(a,x); EXPECT_FALSE(r.estimate.gate);
	EXPECT_GT(r.estimate.std, x.g*cfg.std_raw/1.1f);
}

TEST(TailTrimShadow, InvalidResetContextAndAlwaysZeroInjection) {
	Shadow s; s.configure(config()); auto in = input(); tick(s,in,100); in.i_actual = .5f;
	auto r = tick(s,in); EXPECT_TRUE(r.reset); EXPECT_FLOAT_EQ(r.i_actual,.5f);
	EXPECT_NEAR(r.t_actual,.54f,Tol); EXPECT_FLOAT_EQ(r.actual_tail_trim_torque,0.f);
	in.i_actual = NAN; r = tick(s,in); EXPECT_FALSE(r.valid);
	EXPECT_TRUE(std::isfinite(r.b_obs)); EXPECT_FLOAT_EQ(r.actual_tail_trim_torque,0.f);
}

TEST(TailTrimResources, HostCostAndFixedStorage) {
	Shadow s; auto cfg = config(); cfg.window = 10.f; s.configure(cfg); auto in = input(); tick(s,in,600);
	long long cost[512]{}; // test-only timing storage, never linked in firmware
	for (auto &ns : cost) {
		const auto start = std::chrono::steady_clock::now();
		const auto result = tick(s,in);
		ns = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now()-start).count();
		ASSERT_TRUE(result.valid);
	}
	std::sort(cost,cost+512);
	printf("SHADOW_HOST_BYTES=%zu ESTIMATOR_HOST_BYTES=%zu HOST_NS_MEDIAN=%lld P95=%lld MAX=%lld\n",
	       sizeof(Shadow), sizeof(AdaptiveTailTrimEstimator), cost[256], cost[486], cost[511]);
}
