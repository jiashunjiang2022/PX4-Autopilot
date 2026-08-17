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

#pragma once

#include <stdint.h>

#include <uORB/topics/dataman_request.h>
#include <uORB/topics/dataman_response.h>

namespace dataman_trace
{

enum class EventType : uint8_t {
	ClientRequestPublish = 1,
	ClientRequestRetry,
	ServerRequestReceived,
	ServerOperationComplete,
	ServerResponsePublish,
	ClientResponseSeen,
	ClientResponseAccepted,
	ClientResponseRejected,
	ClientTimeout,
};

enum RejectReason : uint8_t {
	RejectNone = 0,
	RejectWrongClientId = 1 << 0,
	RejectWrongRequestType = 1 << 1,
	RejectWrongItem = 1 << 2,
	RejectWrongIndex = 1 << 3,
	RejectGetIdTimestamp = 1 << 4,
};

enum class NearMissTriggerReason : uint8_t {
	None = 0,
	Retry,
	Reject,
};

struct Counters {
	uint32_t request_publish_count;
	uint32_t request_retry_count;
	uint32_t request_publish_failure_count;
	uint32_t server_request_count;
	uint32_t server_response_count;
	uint32_t server_response_publish_failure_count;
	uint32_t client_response_seen_count;
	uint32_t client_response_drain_copy_count;
	uint32_t client_response_accepted_count;
	uint32_t client_response_rejected_count;
	uint32_t timeout_count;
	uint32_t trace_drop_count;
};

uint16_t allocateClientInstance();

void recordClientRequest(EventType type, uint16_t client_instance, const dataman_request_s &request,
			 uint16_t attempt, bool publish_success, bool trigger_on_sync_retry = false);
void recordClientResponse(EventType type, uint16_t client_instance, const dataman_request_s &request,
				  const dataman_response_s &response, uint16_t attempt, uint8_t reject_reason = RejectNone,
				  uint8_t drain_copy_index = 0);
void recordClientTimeout(uint16_t client_instance, const dataman_request_s &request, uint16_t attempt);
void recordServer(EventType type, const dataman_request_s &request, const dataman_response_s *response = nullptr,
		  bool publish_success = true);

Counters getCounters();
void printCounters();
void printNearMissStatus();
void dump();

} // namespace dataman_trace
