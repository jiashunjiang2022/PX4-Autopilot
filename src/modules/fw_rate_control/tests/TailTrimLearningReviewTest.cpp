// Reuse real V3 fixtures, not a second implementation of its learning algorithm.
#include "BumplessRollITransferTest.cpp"
#include "../AdaptiveTailTrimShadow.hpp"
#include "FrozenAdaptiveTailTrimShadow.hpp"
#include <sstream>

namespace review {
using Shadow = AdaptiveTailTrimShadow;
Shadow::Inputs input() {
	Shadow::Inputs in{}; in.enabled = in.armed = in.control_valid = in.setpoint_valid = in.mapping_valid = true;
	in.landed = false; in.dt = .02f; in.g = .7f; in.imax = 1.f; in.i_actual = .18f; in.mission_eligible = true;
	return in;
}
Shadow::Result tick(Shadow &s, Shadow::Inputs &in, int count = 1) {
	Shadow::Result r{};
	for (int n = 0; n < count; ++n) { in.now += 20000; in.actuator_timestamp = in.now; r = s.update(in); }
	return r;
}
}

TEST(TailTrimLearningReview, ManeuverRetainsEstimateButClearsFreshGate) {
	AdaptiveTailTrimEstimator e; AdaptiveTailTrimEstimator::Config c{};
	for (int n = 0; n < 200; ++n) { e.update(.1f,.02f,true,true,c); }
	for (int n = 0; n < 200; ++n) {
		auto r = e.update(-.2f,.02f,true,false,c);
		EXPECT_FLOAT_EQ(r.b_hat,.1f); EXPECT_FALSE(r.gate); EXPECT_EQ(r.sample_count,0);
	}
	for (int n = 0; n < 149; ++n) { EXPECT_FALSE(e.update(.1f,.02f,true,true,c).gate); }
	EXPECT_TRUE(e.update(.1f,.02f,true,true,c).gate);
}
TEST(TailTrimLearningReview, InvalidSetpointCannotBeValidSample) {
	review::Shadow s; auto in = review::input(); review::tick(s,in,250);
	in.setpoint_valid = false; auto r = review::tick(s,in);
	EXPECT_FALSE(r.valid); EXPECT_FALSE(r.learning); EXPECT_FLOAT_EQ(r.actual_tail_trim_torque,0.f);
}
TEST(TailTrimLearningReview, OutsideMissionObservesButCannotTrim) {
	review::Shadow s; auto in = review::input(); in.mission_eligible = false;
	auto r = review::tick(s,in,400);
	EXPECT_GT(r.b_obs,.1f); EXPECT_FALSE(r.learning); EXPECT_FLOAT_EQ(r.b_shadow,0.f);
}
TEST(TailTrimLearningReview, TrustedPreentryMedianRejectsEntrySpike) {
	review::Shadow s; auto in = review::input(); in.mission_eligible = false;
	in.i_actual = -.18f; review::tick(s,in,2); in.i_actual = .18f;
	review::tick(s,in,300); in.mission_eligible = true; in.i_actual = -.18f;
	auto r = review::tick(s,in);
	EXPECT_GT(r.estimate.b_hat,.11f); EXPECT_FALSE(r.estimate.gate);
	EXPECT_FLOAT_EQ(r.b_shadow,0.f); // never bypass fresh-growth admission
}

TEST(TailTrimLearningReview, SyntheticSixCases) {
	const char *directory = std::getenv("TAIL_TRIM_REVIEW_DIR");
	ASSERT_NE(directory,nullptr);
	std::ofstream csv(std::string(directory)+"/synthetic.csv");
	csv << "scenario,t,burden,g,mission,maneuver,old_b,old_hat,old_gate,old_target,old_zero,candidate_b,candidate_hat,candidate_gate,candidate_target,candidate_zero,v3_b,v3_hat,v3_gate,v3_target,v3_zero,old_mission_start_b,old_mission_start_hat,old_mission_start_gate,old_mission_start_target,old_mission_start_zero\n";
	const char *cases[] = {"constant","noisy","transient","turn","decrease","reversal"};
	for (int scenario = 0; scenario < 6; ++scenario) {
		review::Shadow candidate;
		FrozenAdaptiveTailTrimShadow old;
		FrozenAdaptiveTailTrimShadow old_mission_start;
		RateControl rc; configure(rc,1.f); BumplessRollITransfer v3;
		auto vi = enabled_inputs(); set_adaptive_defaults(vi,0);
		vi.imax_raw = 1.f; vi.cap_raw = .03f; vi.slew_raw_per_s = .02f; // frozen cohort CAP
		vi.adapt_slew_raw_per_s = .01f;
		auto in = review::input(); FrozenAdaptiveTailTrimShadow::Inputs oi{};
		oi.enabled = oi.armed = oi.control_valid = oi.setpoint_valid = oi.mapping_valid = true;
		oi.landed = false; oi.dt = .02f; oi.g = .7f; oi.imax = 1.f;
		float prior_v3 = 0.f;
		float prior_candidate = 0.f, turn_hat = 0.f;
		bool candidate_zero_seen = false;
		int candidate_zero_tick = -1;
		for (int n = 0; n < 5000; ++n) {
			const float t = static_cast<float>(n)*.02f - 6.f;
			float burden = .18f;
			if (scenario == 1) { burden += .012f*sinf(static_cast<float>(n)*1.7f); }
			if (scenario == 2) { burden = t >= 10.f && t < 10.5f ? .18f : 0.f; }
			if (scenario == 3 && t >= 20.f && t < 30.f) { burden = -.1f; }
			if (scenario == 4 && t >= 20.f) { burden = .03f; }
			if (scenario == 5 && t >= 20.f) { burden = -.18f; }
			const bool turn = scenario == 3 && t >= 20.f && t < 30.f;
			in.i_actual = burden; in.phi_sp = turn ? .3f : 0.f;
			in.mission_eligible = t >= 0.f;
			in.now += 20000; in.actuator_timestamp = in.now;
			oi.i_actual = burden; oi.phi_sp = in.phi_sp; oi.now = in.now; oi.actuator_timestamp = in.now;
			const auto a = old.update(oi);
			oi.enabled = t >= 0.f;
			const auto mission_start = old_mission_start.update(oi);
			oi.enabled = true;
			const auto b = candidate.update(in);
			if (scenario == 2) { EXPECT_FLOAT_EQ(b.b_shadow,0.f); }
			if (turn) {
				EXPECT_FLOAT_EQ(b.b_shadow,prior_candidate);
				if (t > 20.02f) { EXPECT_FLOAT_EQ(b.estimate.b_hat,turn_hat); }
				turn_hat = b.estimate.b_hat;
			}
			if (scenario == 5 && t >= 20.f) {
				if (!candidate_zero_seen) { EXPECT_GE(b.b_shadow,0.f); }
				if (b.transfer.reversal_reached_zero) { candidate_zero_seen=true; candidate_zero_tick=n; }
				if (candidate_zero_tick >= 0 && n-candidate_zero_tick < 150) {
					EXPECT_FALSE(b.estimate.gate); EXPECT_FLOAT_EQ(b.b_shadow,0.f);
				}
			}
			prior_candidate = b.b_shadow;
			vi.eligible = t >= 0.f;
			set_residual(rc,burden-v3.transferredRaw(),v3.transferredRaw());
			const auto vr = v3.update(vi,rc);
			ASSERT_FALSE(vr.recovery);
			const float vb = vi.g_current*vr.transferred_i_raw/1.1f;
			const bool vz = prior_v3 > 1e-7f && fabsf(vb) < 1e-7f; prior_v3 = vb;
			csv << cases[scenario] << ',' << t << ',' << burden << ',' << in.g << ',' << vi.eligible << ',' << turn
			    << ',' << a.b_shadow << ',' << a.estimate.b_hat << ',' << a.estimate.gate << ',' << a.target << ',' << a.transfer.reversal_reached_zero
			    << ',' << b.b_shadow << ',' << b.estimate.b_hat << ',' << b.estimate.gate << ',' << b.target << ',' << b.transfer.reversal_reached_zero
			    << ',' << vb << ',' << vi.g_current*vr.adapt_t_hat_raw/1.1f << ',' << vr.adapt_gate
			    << ',' << vi.g_current*vr.adapt_target_raw/1.1f << ',' << vz
			    << ',' << mission_start.b_shadow << ',' << mission_start.estimate.b_hat << ',' << mission_start.estimate.gate
			    << ',' << mission_start.target << ',' << mission_start.transfer.reversal_reached_zero << '\n';
			EXPECT_FLOAT_EQ(a.actual_tail_trim_torque,0.f); EXPECT_FLOAT_EQ(b.actual_tail_trim_torque,0.f);
		}
	}
}

TEST(TailTrimLearningReview, HistoricalCausalRows) {
	const char *directory = std::getenv("TAIL_TRIM_REVIEW_DIR"); ASSERT_NE(directory,nullptr);
	std::ifstream input(std::string(directory)+"/replay-input.csv");
	if (!input.good()) { GTEST_SKIP() << "Historical causal rows not exported"; }
	std::ofstream output(std::string(directory)+"/replay-output.csv");
	output << "flight,t,armed,mission,trusted,obs,actual_s,old_b,old_hat,old_gate,old_valid,old_reset,candidate_b,candidate_hat,candidate_gate,candidate_valid,candidate_reset,maneuver,old_maneuver,offset,mismatch,actual_injection\n";
	std::string line; std::getline(input,line);
	review::Shadow candidate; FrozenAdaptiveTailTrimShadow old;
	int last_flight = -1; std::vector<double> previous_config;
	while (std::getline(input,line)) {
		std::stringstream row(line); std::string cell; std::vector<double> v;
		while (std::getline(row,cell,',')) { v.push_back(std::stod(cell)); }
		ASSERT_EQ(v.size(),30u);
		const int flight = static_cast<int>(v[0]);
		std::vector<double> cfg(v.begin()+25,v.end());
		if (flight != last_flight || cfg != previous_config) {
			review::Shadow::Config c{}; c.tau = static_cast<float>(v[25]); c.window = static_cast<float>(v[26]);
			c.std_raw = static_cast<float>(v[27]); c.sign_fraction = static_cast<float>(v[28]); c.entry_window = static_cast<float>(v[29]);
			candidate.configure(c);
			FrozenAdaptiveTailTrimShadow::Config oc{}; oc.tau = c.tau; oc.window = c.window; oc.std_raw = c.std_raw; oc.sign_fraction = c.sign_fraction;
			old.configure(oc); last_flight = flight; previous_config = cfg;
		}
		auto in = review::input(); FrozenAdaptiveTailTrimShadow::Inputs oi{};
		in.now = oi.now = static_cast<uint64_t>(v[2]); in.actuator_timestamp = oi.actuator_timestamp = static_cast<uint64_t>(v[3]);
		in.dt = oi.dt = static_cast<float>(v[4]); in.g = oi.g = static_cast<float>(v[5]);
		in.i_actual = oi.i_actual = static_cast<float>(v[6]); in.s_actual = oi.s_actual = static_cast<float>(v[7]);
		in.imax = oi.imax = static_cast<float>(v[8]); in.phi_sp = oi.phi_sp = static_cast<float>(v[9]);
		in.p_sp = oi.p_sp = static_cast<float>(v[10]); in.left = oi.left = static_cast<float>(v[11]); in.right = oi.right = static_cast<float>(v[12]);
		in.armed = oi.armed = v[13] > .5; in.landed = oi.landed = v[14] > .5;
		in.control_valid = oi.control_valid = v[15] > .5; in.setpoint_valid = oi.setpoint_valid = v[16] > .5;
		in.mapping_valid = oi.mapping_valid = v[17] > .5; in.safety = oi.safety = v[18] > .5;
		in.mission_eligible = v[19] > .5; in.epoch = oi.epoch = static_cast<uint32_t>(v[20]); in.enabled = oi.enabled = true;
		const auto a = old.update(oi); const auto b = candidate.update(in);
		const bool trusted = in.armed && !in.landed && in.mission_eligible && in.control_valid && in.setpoint_valid
			&& in.mapping_valid && !in.safety && !a.maneuver && !b.maneuver && b.pitch.valid;
		output << flight << ',' << v[1] << ',' << in.armed << ',' << in.mission_eligible << ',' << trusted
		       << ',' << v[21] << ',' << v[22] << ',' << a.b_shadow << ',' << a.estimate.b_hat << ',' << a.estimate.gate << ',' << a.valid << ',' << a.reset
		       << ',' << b.b_shadow << ',' << b.estimate.b_hat << ',' << b.estimate.gate << ',' << b.valid << ',' << b.reset << ',' << b.maneuver << ',' << a.maneuver
		       << ',' << b.offset << ',' << b.transfer.transfer_mismatch << ',' << b.actual_tail_trim_torque << '\n';
		ASSERT_FLOAT_EQ(b.actual_tail_trim_torque,0.f);
		ASSERT_TRUE(std::isfinite(b.b_shadow));
	}
}

TEST(TailTrimLearningReview, MedianWindowFreshnessAndNoTurnEvidence) {
	AdaptiveTailTrimEstimator e; AdaptiveTailTrimEstimator::Config c{};
	for (int n = 0; n < 249; ++n) { e.observePreEntry(.1f,.02f,true); }
	e.enterMission(5.f); EXPECT_FLOAT_EQ(e.update(-.1f,.02f,true,true,c).b_hat,-.1f);
	e.reset();
	for (int n = 0; n < 250; ++n) { e.observePreEntry(.1f,.02f,true); }
	e.enterMission(5.f); EXPECT_GT(e.update(-.1f,.02f,true,true,c).b_hat,.09f);
	e.reset();
	for (int n = 0; n < 300; ++n) { e.observePreEntry(.1f,.02f,true); }
	e.observePreEntry(-.2f,.02f,false); e.enterMission(5.f);
	EXPECT_FLOAT_EQ(e.update(-.1f,.02f,true,true,c).b_hat,-.1f);
}

TEST(TailTrimLearningReview, PreentryHistoryIndependentOfPitchAndFullTenSecondWindow) {
	AdaptiveTailTrimEstimator e; AdaptiveTailTrimEstimator::Config c{};
	for (int n = 0; n < 500; ++n) { e.observePreEntry(n%10 == 0 ? -.2f : .1f,.02f,true); }
	e.enterMission(10.f); EXPECT_GT(e.update(.1f,.02f,true,true,c).b_hat,.09f);
	review::Shadow a,b; auto x=review::input(), y=x; x.mission_eligible=y.mission_eligible=false;
	y.left=NAN;
	for (int n=0;n<300;++n) { review::tick(a,x); review::tick(b,y); }
	x.mission_eligible=y.mission_eligible=true;
	const auto ra=review::tick(a,x), rb=review::tick(b,y);
	EXPECT_FLOAT_EQ(ra.estimate.b_hat,rb.estimate.b_hat); EXPECT_GT(ra.estimate.b_hat,.1f);
}
TEST(TailTrimLearningReview, MissionExitFreezesTargetAndFreshGateOnReturn) {
	review::Shadow s; auto in=review::input(); const auto before=review::tick(s,in,1000);
	ASSERT_GT(before.b_shadow,.02f); in.mission_eligible=false; in.i_actual=-.18f;
	const auto frozen=review::tick(s,in,100);
	EXPECT_FLOAT_EQ(frozen.b_shadow,before.b_shadow); EXPECT_FLOAT_EQ(frozen.target,before.target);
	in.mission_eligible=true; auto r=review::tick(s,in);
	EXPECT_FALSE(r.estimate.gate); EXPECT_FALSE(r.reversal);
}
