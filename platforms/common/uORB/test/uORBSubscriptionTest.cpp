/****************************************************************************
 *
 *   Copyright (c) 2019-2024 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * Test for Subscription
 */

#include <gtest/gtest.h>
#include <uORB/Subscription.hpp>
#include <uORB/uORB.h>
#include <uORB/topics/airspeed_quality_input.h>
#include <uORB/topics/orb_test.h>

namespace uORB
{
namespace test
{


class uORBSubscriptionTestable  : public  uORB::Subscription
{
public:
	uORBSubscriptionTestable() : Subscription(ORB_ID(orb_test), 0)
	{
	}

	void setNodeValue(void *node)
	{
		_node = node;

	}
	void *getNodeValue()
	{
		return _node;
	}

};


class uORBSubscriptionTest : public ::testing::Test
{
protected:
	uORBSubscriptionTestable  testable;

	static void SetUpTestSuite()
	{
		uORB::Manager::initialize();

		orb_test_s message{};
		orb_advertise(ORB_ID(orb_test), &message);

	}
	static void TearDownTestSuite()
	{
		uORB::Manager::terminate();
	}

	void TearDown() override
	{
		testable.setNodeValue(nullptr);
	}


};

TEST_F(uORBSubscriptionTest, updateWhenSubscribedThenNotSubscribedTwice)
{
	int anyValue = 1;
	testable.setNodeValue(&anyValue);

	testable.updated();

	ASSERT_EQ(testable.getNodeValue(), &anyValue) << "Original node value don't have to be overrwiten";
}

TEST_F(uORBSubscriptionTest, updateWhenNotSubscribedThenSubscribed)
{
	testable.setNodeValue(nullptr);

	testable.updated();

	ASSERT_NE(testable.getNodeValue(), nullptr) << "Node value after 'updated' have to be initialized";
}

TEST_F(uORBSubscriptionTest, queuedAirspeedQualitySamplesAreDeliveredInOrder)
{
	ASSERT_EQ(orb_get_queue_size(ORB_ID(airspeed_quality_input)), 4);

	airspeed_quality_input_s initial{};
	const orb_advert_t publisher = orb_advertise(ORB_ID(airspeed_quality_input), &initial);
	ASSERT_NE(publisher, nullptr);

	uORB::Subscription quality_sub{ORB_ID(airspeed_quality_input)};
	airspeed_quality_input_s received{};
	ASSERT_TRUE(quality_sub.update(&received));
	EXPECT_EQ(received.timestamp_sample, 0u);

	airspeed_quality_input_s first{};
	airspeed_quality_input_s second{};
	first.timestamp_sample = 20000;
	second.timestamp_sample = 40000;
	orb_publish(ORB_ID(airspeed_quality_input), publisher, &first);
	orb_publish(ORB_ID(airspeed_quality_input), publisher, &second);

	ASSERT_TRUE(quality_sub.update(&received));
	EXPECT_EQ(received.timestamp_sample, first.timestamp_sample);
	ASSERT_TRUE(quality_sub.update(&received));
	EXPECT_EQ(received.timestamp_sample, second.timestamp_sample);
	EXPECT_FALSE(quality_sub.update(&received));

	airspeed_quality_input_s old_identity{};
	airspeed_quality_input_s reset_marker{};
	airspeed_quality_input_s new_identity{};
	old_identity.timestamp_sample = 60000;
	old_identity.device_id = 1;
	reset_marker.timestamp_sample = 80000;
	reset_marker.device_id = 2;
	reset_marker.reset_reason = airspeed_quality_input_s::RESET_REASON_DEVICE_CHANGE;
	new_identity.timestamp_sample = 100000;
	new_identity.device_id = 2;
	new_identity.valid = true;
	orb_publish(ORB_ID(airspeed_quality_input), publisher, &old_identity);
	orb_publish(ORB_ID(airspeed_quality_input), publisher, &reset_marker);
	orb_publish(ORB_ID(airspeed_quality_input), publisher, &new_identity);

	ASSERT_TRUE(quality_sub.update(&received));
	EXPECT_EQ(received.device_id, old_identity.device_id);
	ASSERT_TRUE(quality_sub.update(&received));
	EXPECT_EQ(received.reset_reason, reset_marker.reset_reason);
	ASSERT_TRUE(quality_sub.update(&received));
	EXPECT_EQ(received.device_id, new_identity.device_id);
	EXPECT_EQ(received.timestamp_sample, new_identity.timestamp_sample);
	EXPECT_FALSE(quality_sub.update(&received));

	orb_unadvertise(publisher);
}
}
}
