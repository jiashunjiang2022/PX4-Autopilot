// Investigation only: real uORB and RateControl; no controller fix.
#include <gtest/gtest.h>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/control_allocator_status.h>
#include <lib/rate_control/rate_control.hpp>
#include <cfloat>

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
