// Investigation only: real uORB and RateControl; no controller fix.
#include <gtest/gtest.h>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/control_allocator_status.h>
#include <lib/rate_control/rate_control.hpp>
#include <cfloat>

namespace {
struct ObservedRateControl : RateControl {
	int positive_calls[3]{};
	void setPositiveSaturationFlag(size_t axis, bool value) {
		++positive_calls[axis];
		RateControl::setPositiveSaturationFlag(axis, value);
	}
};
struct ObservedSubscription {
	uORB::Subscription sub;
	int reads{0};
	explicit ObservedSubscription(uint8_t instance) : sub(ORB_ID(control_allocator_status), instance) {}
	bool update(control_allocator_status_s *status) { ++reads; return sub.update(status); }
};
struct RoutingHarness {
	ObservedSubscription _control_allocator_status_subs[2]{ObservedSubscription(0), ObservedSubscription(1)};
	ObservedRateControl _rate_control;
	struct { bool is_vtol{false}; } _vehicle_status;
	RoutingHarness() {
		_rate_control.setPidGains(matrix::Vector3f(), matrix::Vector3f(1.f, 1.f, 1.f), matrix::Vector3f());
		_rate_control.setIntegratorLimit(matrix::Vector3f(.2f, .2f, .2f));
		_rate_control.setSaturationStatus(matrix::Vector3<bool>(false, false, false), matrix::Vector3<bool>(false, false, false));
	}
	void run(const matrix::Vector3<bool> &diffthr_enabled) {
#include "AllocatorRouting.inc"
	}
	void integrate(float error) {
		_rate_control.update(matrix::Vector3f(), matrix::Vector3f(error, error, error), matrix::Vector3f(), .02f, false);
	}
};
}

class AllocatorSaturationFeedback : public ::testing::Test
{
protected:
	orb_advert_t publisher{};
	void SetUp() override {
		control_allocator_status_s message{};
		message.unallocated_torque[0] = .2f;
		publisher = orb_advertise(ORB_ID(control_allocator_status), &message);
		ASSERT_NE(publisher, nullptr);
		ASSERT_EQ(orb_get_queue_size(ORB_ID(control_allocator_status)), 1);
	}
	void TearDown() override { orb_unadvertise(publisher); }
};

TEST_F(AllocatorSaturationFeedback, SinglePublicationTwoUpdates)
{
	uORB::Subscription sub{ORB_ID(control_allocator_status)};
	control_allocator_status_s received{};
	EXPECT_TRUE(sub.updated());
	EXPECT_TRUE(sub.updated()); // predicate does not consume
	ASSERT_TRUE(sub.update(&received));
	EXPECT_FLOAT_EQ(received.unallocated_torque[0], .2f);
	EXPECT_FALSE(sub.updated());
	EXPECT_FALSE(sub.update(&received));
	EXPECT_TRUE(sub.copy(&received)); // repeat read allowed, even without update
	EXPECT_FALSE(sub.update(&received));
	control_allocator_status_s next{};
	next.unallocated_torque[0] = .3f;
	ASSERT_EQ(orb_publish(ORB_ID(control_allocator_status), publisher, &next), 0);
	EXPECT_TRUE(sub.updated());
	ASSERT_TRUE(sub.copy(&received)); // copy also consumes pending generation
	EXPECT_FLOAT_EQ(received.unallocated_torque[0], .3f);
	EXPECT_FALSE(sub.updated());
	EXPECT_FALSE(sub.update(&received));
}

TEST_F(AllocatorSaturationFeedback, DesiredNonVtolSurfaceAndNegativeSymmetry)
{
	for (float sign : {1.f, -1.f}) {
		control_allocator_status_s message{};
		message.unallocated_torque[0] = sign * .2f;
		ASSERT_EQ(orb_publish(ORB_ID(control_allocator_status), publisher, &message), 0);
		RoutingHarness h;
		h.run(matrix::Vector3<bool>(false, false, false));
		EXPECT_EQ(h._rate_control.positive_calls[0], 1);
		EXPECT_EQ(h._control_allocator_status_subs[0].reads, 1);
		h.integrate(sign * .1f);
		EXPECT_FLOAT_EQ(h._rate_control.rollIntegralRaw(), 0.f);
		// No new sample: no new setter call; previously set flag is retained.
		h.run(matrix::Vector3<bool>(false, false, false));
		EXPECT_EQ(h._rate_control.positive_calls[0], 1);
		h.integrate(sign * .1f);
		EXPECT_FLOAT_EQ(h._rate_control.rollIntegralRaw(), 0.f);
	}
}

TEST_F(AllocatorSaturationFeedback, DesiredMixedAxesShareOneSample)
{
	control_allocator_status_s message{};
	for (int i = 0; i < 3; ++i) { message.unallocated_torque[i] = .2f; }
	ASSERT_EQ(orb_publish(ORB_ID(control_allocator_status), publisher, &message), 0);
	RoutingHarness h;
	h.run(matrix::Vector3<bool>(true, false, false));
	h.integrate(.1f);
	rate_ctrl_status_s result{};
	h._rate_control.getRateControlStatus(result);
	EXPECT_FLOAT_EQ(result.rollspeed_integ, 0.f);
	EXPECT_FLOAT_EQ(result.pitchspeed_integ, 0.f);
	EXPECT_FLOAT_EQ(result.yawspeed_integ, 0.f);
	for (int i = 0; i < 3; ++i) { EXPECT_EQ(h._rate_control.positive_calls[i], 1); }
	EXPECT_EQ(h._control_allocator_status_subs[0].reads, 1);
}

TEST_F(AllocatorSaturationFeedback, DesiredVtolKeepsIndependentInstances)
{
	control_allocator_status_s surface{};
	// Opposite signs distinguish instance 1 from the Roll-positive instance 0.
	surface.unallocated_torque[0] = -.2f;
	surface.unallocated_torque[1] = -.2f;
	surface.unallocated_torque[2] = -.2f;
	int instance = -1;
	const auto second = orb_advertise_multi(ORB_ID(control_allocator_status), &surface, &instance);
	ASSERT_NE(second, nullptr);
	EXPECT_EQ(instance, 1);
	{
		RoutingHarness h; h._vehicle_status.is_vtol = true;
		h.run(matrix::Vector3<bool>(true, false, false));
		h._rate_control.update(matrix::Vector3f(), matrix::Vector3f(.1f, -.1f, -.1f), matrix::Vector3f(), .02f, false);
		rate_ctrl_status_s result{}; h._rate_control.getRateControlStatus(result);
		EXPECT_FLOAT_EQ(result.rollspeed_integ, 0.f);
		EXPECT_FLOAT_EQ(result.pitchspeed_integ, 0.f);
		EXPECT_FLOAT_EQ(result.yawspeed_integ, 0.f);
		EXPECT_EQ(h._control_allocator_status_subs[0].reads, 1);
		EXPECT_EQ(h._control_allocator_status_subs[1].reads, 1);
	}
	orb_unadvertise(second);
}

TEST_F(AllocatorSaturationFeedback, NonVtolExtractedPathLosesFlagAndAllowsIntegralGrowth)
{
	uORB::SubscriptionMultiArray<control_allocator_status_s, 2> subs{ORB_ID::control_allocator_status};
	RateControl rc, positive_control;
	for (auto *controller : {&rc, &positive_control}) {
		controller->setPidGains(matrix::Vector3f(), matrix::Vector3f(1.f, 0.f, 0.f), matrix::Vector3f());
		controller->setIntegratorLimit(matrix::Vector3f(.2f, .2f, .2f));
		controller->setSaturationStatus(matrix::Vector3<bool>(false, false, false), matrix::Vector3<bool>(false, false, false));
	}
	const bool is_vtol = false;
	const matrix::Vector3<bool> diffthr_enabled(false, false, false);
	control_allocator_status_s status{};
	int roll_positive_writes = 0;
	// Extracted verbatim branch/index logic from FixedwingRateControl.cpp:352-369.
	// Counts instrument setter reachability; production setter and integrator are real.
	const bool first = subs[0].update(&status);
	if (first) {
		for (size_t i = 0; i < 3; ++i) {
			if (diffthr_enabled(i)) {
				rc.setPositiveSaturationFlag(i, status.unallocated_torque[i] > FLT_EPSILON);
				rc.setNegativeSaturationFlag(i, status.unallocated_torque[i] < -FLT_EPSILON);
				if (i == 0) { ++roll_positive_writes; }
			}
		}
	}
	const bool second = subs[is_vtol ? 1 : 0].update(&status);
	if (second) {
		for (size_t i = 0; i < 3; ++i) {
			if (!diffthr_enabled(i)) {
				rc.setPositiveSaturationFlag(i, status.unallocated_torque[i] > FLT_EPSILON);
				rc.setNegativeSaturationFlag(i, status.unallocated_torque[i] < -FLT_EPSILON);
				if (i == 0) { ++roll_positive_writes; }
			}
		}
	}
	ASSERT_TRUE(first); ASSERT_FALSE(second);
	ASSERT_FLOAT_EQ(status.unallocated_torque[0], .2f);
	EXPECT_EQ(roll_positive_writes, 0);
	// Positive experimental control: prove actual native anti-windup can block
	// this stimulus. This is not a proposed routing change.
	positive_control.setPositiveSaturationFlag(0, true);
	const matrix::Vector3f zero{}, rate_sp(.1f, 0.f, 0.f);
	rc.update(zero, rate_sp, zero, .02f, false);
	positive_control.update(zero, rate_sp, zero, .02f, false);
	EXPECT_NEAR(rc.rollIntegralRaw(), .00199958965f, 1e-8f);
	EXPECT_FLOAT_EQ(positive_control.rollIntegralRaw(), 0.f);
	for (int i = 0; i < 1000; ++i) { rc.update(zero, rate_sp, zero, .02f, false); }
	EXPECT_FLOAT_EQ(rc.rollIntegralRaw(), .2f); // absolute clamp survives
}
