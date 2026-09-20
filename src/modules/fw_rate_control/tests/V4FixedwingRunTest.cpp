// SPDX-License-Identifier: BSD-3-Clause
#include <gtest/gtest.h>
#include "../FixedwingRateControl.hpp"
#include "BaselineFixedwingRateControl.hpp"
#include <memory>

struct ResidualSlowFeedforwardMemoryTestAccess {
	static void nativeInput(RateControl &n, float i) { n._rate_int(0) = i; }
	static void seed(RateControl &n, ResidualSlowFeedforwardMemory &m, float i, float b)
	{
		n._rate_int(0) = i; m._b = b; m._epoch = n.rollIntegralResetEpoch();
	}
};
struct V4IntegrationTestAccess {
	template<typename Controller> static void setup(Controller &c)
	{
		c._vehicle_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_FIXED_WING;
		c._vehicle_status.arming_state = vehicle_status_s::ARMING_STATE_ARMED;
		c._vehicle_status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION;
		c._vcontrol_mode.flag_control_rates_enabled = true;
		c._landed = false;
		c._rates_sp.roll = .1f;
		c._param_ca_airframe.set(1); c._param_ca_method.set(0); c._param_ca_sv_cs_count.set(3);
		c._param_ca_sv_cs0_type.set(5); c._param_ca_sv_cs0_trq_r.set(-.55f);
		c._param_ca_sv_cs0_trq_p.set(1.f); c._param_ca_sv_cs0_trq_y.set(0.f); c._param_ca_sv_cs0_trim.set(0.f);
		c._param_ca_sv_cs1_type.set(6); c._param_ca_sv_cs1_trq_r.set(.55f);
		c._param_ca_sv_cs1_trq_p.set(1.f); c._param_ca_sv_cs1_trq_y.set(0.f); c._param_ca_sv_cs1_trim.set(0.f);
		c._param_ca_sv_cs2_type.set(4); c._param_ca_sv_cs2_trq_r.set(0.f);
		c._param_ca_sv_cs2_trq_p.set(0.f); c._param_ca_sv_cs2_trq_y.set(1.f); c._param_ca_sv_cs2_trim.set(0.f);
		c._param_pwm_main_func1.set(201); c._param_pwm_main_func2.set(202); c._param_pwm_main_func5.set(203);
		c._param_pwm_main_rev.set(17);
		c._param_fw_rr_imax.set(.2f); c._param_fw_rr_i.set(.1f);
		c._param_flap_slow_en.set(true); c._param_flap_b2b_cap.set(.05f);
		c._param_flap_b2b_slew.set(.02f); c._param_flap_b2b_adapt.set(false);
		c.parameters_update();
		ResidualSlowFeedforwardMemoryTestAccess::nativeInput(c._rate_control, .15f);
	}
	template<typename Controller> static void run(Controller &c)
	{
		// Fixture parameters above are explicit cache inputs, not parameter-server stimuli.
		parameter_update_s ignored{}; c._parameter_update_sub.copy(&ignored);
		c._last_run = 0; c.Run(); c.ScheduleClear();
	}
	template<typename Controller> static float i(Controller &c) { return c._rate_control.rollIntegralRaw(); }
	template<typename Controller> static float s(Controller &c) { return c._bumpless_roll_i_transfer.transferredRaw(); }
	template<typename Controller> static float torque(Controller &c) { return c._vehicle_torque_setpoint.xyz[0]; }
	static void acquire(FixedwingRateControl &c)
	{
		c._param_fw_v4_en.set(true);
		c._vehicle_status.arming_state = vehicle_status_s::ARMING_STATE_DISARMED;
		run(c);
		c._vehicle_status.arming_state = vehicle_status_s::ARMING_STATE_ARMED;
		ResidualSlowFeedforwardMemoryTestAccess::seed(c._rate_control, c._v4_memory._core, .1f, .04f);
	}
	static const flap_v4_memory_status_s &status(FixedwingRateControl &c) { return c._v4_memory.status(); }
	static float gain(FixedwingRateControl &c) { return c._gain_compression.getGains()(0); }
	static void shrink(FixedwingRateControl &c) { c._param_fw_rr_imax.set(.1f); c._param_fw_v4_bmax.set(.05f); c.parameters_update(); }
	static void reset(FixedwingRateControl &c) { c._rates_sp.reset_integral = true; }
	static void abort(FixedwingRateControl &c) { c._vehicle_status.nav_state = vehicle_status_s::NAVIGATION_STATE_STAB; }
	static void disarm(FixedwingRateControl &c) { c._vehicle_status.arming_state = vehicle_status_s::ARMING_STATE_DISARMED; }
	static void disableAndChangeLimit(FixedwingRateControl &c)
	{
		ResidualSlowFeedforwardMemoryTestAccess::seed(c._rate_control, c._v4_memory._core, 0.f, 0.f);
		c._param_fw_v4_en.set(false); c._param_fw_rr_imax.set(.12f); c.parameters_update();
	}
	static float limit(FixedwingRateControl &c) { return c._rate_control.rollIntegralLimit(); }
};

class V4FixedwingRun : public ::testing::Test
{
protected:
	static void SetUpTestSuite() { hrt_init(); }
	std::unique_ptr<FixedwingRateControl> current;
	uORB::Publication<vehicle_angular_velocity_s> gyro{ORB_ID(vehicle_angular_velocity)};
	uORB::Publication<control_allocator_status_s> allocator{ORB_ID(control_allocator_status)};
	void SetUp() override
	{
		param_reset_all();
		current.reset(new FixedwingRateControl());
		V4IntegrationTestAccess::setup(*current);
		publish();
	}
	void publish()
	{
		vehicle_angular_velocity_s a{}; a.timestamp = a.timestamp_sample = hrt_absolute_time(); gyro.publish(a);
		control_allocator_status_s s{}; s.timestamp = a.timestamp; allocator.publish(s);
	}
	void run() { publish(); V4IntegrationTestAccess::run(*current); }
};
TEST_F(V4FixedwingRun, DisabledEqualsActualE624RunAcrossCycles)
{
	std::unique_ptr<BaselineFixedwingRateControl> baseline(new BaselineFixedwingRateControl());
	V4IntegrationTestAccess::setup(*baseline);
	for (int j = 0; j < 250; ++j) {
		publish(); V4IntegrationTestAccess::run(*current); V4IntegrationTestAccess::run(*baseline);
		EXPECT_FLOAT_EQ(V4IntegrationTestAccess::torque(*current), V4IntegrationTestAccess::torque(*baseline));
		EXPECT_FLOAT_EQ(V4IntegrationTestAccess::i(*current), V4IntegrationTestAccess::i(*baseline));
		EXPECT_FLOAT_EQ(V4IntegrationTestAccess::s(*current), V4IntegrationTestAccess::s(*baseline));
	}
	EXPECT_GT(V4IntegrationTestAccess::s(*baseline), 0.f);
}
TEST_F(V4FixedwingRun, ActualRunUsesPostTransferIAndOneBPublishesDiagnostics)
{
	V4IntegrationTestAccess::acquire(*current);
	const float frozen_gain = V4IntegrationTestAccess::gain(*current);
	run();
	const auto &s = V4IntegrationTestAccess::status(*current);
	ASSERT_TRUE(s.active); ASSERT_TRUE(s.apply_valid); ASSERT_GT(s.b_raw, 0.f);
	EXPECT_FLOAT_EQ(V4IntegrationTestAccess::s(*current), 0.f);
	uORB::Subscription terms_sub{ORB_ID(rate_ctrl_terms)};
	rate_ctrl_terms_s terms{}; ASSERT_TRUE(terms_sub.copy(&terms));
	EXPECT_FLOAT_EQ(terms.i_term[0], s.i_post_transfer_raw);
	EXPECT_GT(s.i_post_natural_raw, s.i_post_transfer_raw);
	// Airspeed scale=1 and trim=0; gain compression is frozen before its update.
	EXPECT_NEAR(V4IntegrationTestAccess::torque(*current), frozen_gain * (terms.output[0] + s.b_raw), 2e-7f);
	uORB::Subscription diag{ORB_ID(flap_v4_memory_status)};
	flap_v4_memory_status_s published{}; ASSERT_TRUE(diag.copy(&published));
	EXPECT_FLOAT_EQ(published.b_raw, s.b_raw); EXPECT_EQ(published.timestamp, s.timestamp);
}
TEST_F(V4FixedwingRun, ActualParameterShrinkDeferredThenRecovers)
{
	V4IntegrationTestAccess::acquire(*current);
	V4IntegrationTestAccess::shrink(*current);
	EXPECT_FLOAT_EQ(V4IntegrationTestAccess::i(*current), .1f);
	run(); const auto &s = V4IntegrationTestAccess::status(*current);
	EXPECT_TRUE(s.recovery_active); EXPECT_FLOAT_EQ(s.b_raw, 0.f); EXPECT_FLOAT_EQ(s.i_post_transfer_raw, 0.f);
	EXPECT_FLOAT_EQ(s.imax, .1f);
}
TEST_F(V4FixedwingRun, ActualRateResetClearsB)
{
	V4IntegrationTestAccess::acquire(*current); V4IntegrationTestAccess::reset(*current); run();
	const auto &s = V4IntegrationTestAccess::status(*current);
	EXPECT_TRUE(s.recovery_active); EXPECT_FLOAT_EQ(s.b_raw, 0.f); EXPECT_EQ(s.window_sample_count, 0u);
}
TEST_F(V4FixedwingRun, ActualMissionToStabilizedHandback)
{
	V4IntegrationTestAccess::acquire(*current); run();
	const float b = V4IntegrationTestAccess::status(*current).b_raw;
	V4IntegrationTestAccess::abort(*current); run();
	const auto &s = V4IntegrationTestAccess::status(*current);
	EXPECT_LT(s.b_raw, b); EXPECT_TRUE(s.handback_active);
	EXPECT_NEAR(s.t_pre_transfer_raw, s.t_post_transfer_raw, 2e-7f);
}
TEST_F(V4FixedwingRun, DisarmedV4CannotNaturallyReintegrate)
{
	V4IntegrationTestAccess::acquire(*current); V4IntegrationTestAccess::disarm(*current); run();
	EXPECT_FLOAT_EQ(V4IntegrationTestAccess::i(*current), 0.f);
	EXPECT_FLOAT_EQ(V4IntegrationTestAccess::status(*current).b_raw, 0.f);
}
TEST_F(V4FixedwingRun, ReleaseOwnershipInstallsDeferredV3Limit)
{
	V4IntegrationTestAccess::acquire(*current);
	V4IntegrationTestAccess::disableAndChangeLimit(*current); run();
	EXPECT_FALSE(V4IntegrationTestAccess::status(*current).active);
	EXPECT_FLOAT_EQ(V4IntegrationTestAccess::limit(*current), .12f);
}
