/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#include <gtest/gtest.h>

#include <parameters/param.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/airspeed_validated.h>
#include <uORB/topics/airspeed_wind.h>
#include <uORB/topics/debug_vect.h>
#include <uORB/topics/encoder_count.h>
#include <uORB/topics/flap_frequency.h>
#include <uORB/topics/rpm.h>
#include <uORB/topics/sensor_gnss_relative.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/wind.h>

#include "logged_topics.h"

namespace
{

px4::logger::SDLogProfileMask combine_profiles(std::initializer_list<int32_t> values)
{
	int32_t mask = 0;

	for (int32_t value : values) {
		mask |= value;
	}

	return static_cast<px4::logger::SDLogProfileMask>(mask);
}

int find_interval_ms(const px4::logger::LoggedTopics &logged_topics, ORB_ID topic_id, uint8_t instance = 0)
{
	const auto &subscriptions = logged_topics.subscriptions();

	for (int i = 0; i < subscriptions.count; ++i) {
		const auto &subscription = subscriptions.sub[i];

		if ((subscription.id == topic_id) && (subscription.instance == instance)) {
			return subscription.interval_ms;
		}
	}

	return -1;
}

template<typename MessageType>
void publish_once(uORB::Publication<MessageType> &publication)
{
	MessageType message{};
	message.timestamp = 1;
	publication.publish(message);
}

template<typename MessageType>
void publish_once(uORB::PublicationMulti<MessageType> &publication)
{
	MessageType message{};
	message.timestamp = 1;
	publication.publish(message);
}

class LoggedTopicsTest : public ::testing::Test
{
public:
	void SetUp() override
	{
		param_control_autosave(false);

		publish_once(_airspeed_validated_pub);
		publish_once(_airspeed_wind_pub);
		publish_once(_rpm_pub);
	}

protected:
	uORB::Publication<airspeed_validated_s> _airspeed_validated_pub{ORB_ID(airspeed_validated)};
	uORB::PublicationMulti<airspeed_wind_s> _airspeed_wind_pub{ORB_ID(airspeed_wind)};
	uORB::PublicationMulti<rpm_s> _rpm_pub{ORB_ID(rpm)};
};

TEST_F(LoggedTopicsTest, FlappingDatasetProfileAddsRequiredTopicsAtFullRate)
{
	px4::logger::LoggedTopics logged_topics;

	ASSERT_TRUE(logged_topics.initialize_logged_topics(combine_profiles({
		static_cast<int32_t>(px4::logger::SDLogProfileMask::DEFAULT),
		1 << 12
	})));

	EXPECT_EQ(find_interval_ms(logged_topics, ORB_ID(wind)), 0);
	EXPECT_EQ(find_interval_ms(logged_topics, ORB_ID(sensor_gps), 0), 0);
	EXPECT_EQ(find_interval_ms(logged_topics, ORB_ID(sensor_gps), 3), 0);
	EXPECT_EQ(find_interval_ms(logged_topics, ORB_ID(sensor_gnss_relative), 0), 0);
	EXPECT_EQ(find_interval_ms(logged_topics, ORB_ID(vehicle_odometry)), 0);
	EXPECT_EQ(find_interval_ms(logged_topics, ORB_ID(flap_frequency)), 0);
	EXPECT_EQ(find_interval_ms(logged_topics, ORB_ID(rpm), 0), 0);
	EXPECT_EQ(find_interval_ms(logged_topics, ORB_ID(encoder_count)), 0);
	EXPECT_EQ(find_interval_ms(logged_topics, ORB_ID(debug_vect)), 0);
	EXPECT_EQ(find_interval_ms(logged_topics, ORB_ID(airspeed_validated)), 0);
	EXPECT_EQ(find_interval_ms(logged_topics, ORB_ID(airspeed_wind), 0), 0);
}

} // namespace
