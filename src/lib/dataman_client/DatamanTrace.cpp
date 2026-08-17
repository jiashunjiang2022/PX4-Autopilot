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

#include "DatamanTrace.hpp"

#include <drivers/drv_hrt.h>
#include <inttypes.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/log.h>

namespace dataman_trace
{

namespace
{

static constexpr uint32_t TraceDepth = 512;
static constexpr uint32_t PostTriggerEvents = 96;

enum class CaptureState : uint32_t {
	Armed = 0,
	Triggering,
	Capturing,
	Freezing,
	Frozen,
};

enum EventFlag : uint8_t {
	FlagNone = 0,
	FlagPublishSuccess = 1 << 0,
	FlagResponseOlderThanRequest = 1 << 1,
};

struct TraceEvent {
	uint64_t timestamp;
	uint64_t request_timestamp;
	uint64_t response_timestamp;
	uint32_t expected_index;
	uint32_t actual_index;
	uint16_t client_instance;
	uint16_t attempt;
	uint8_t event_type;
	uint8_t expected_client_id;
	uint8_t expected_request_type;
	uint8_t expected_item;
	uint8_t actual_client_id;
	uint8_t actual_request_type;
	uint8_t actual_item;
	uint8_t response_status;
	uint8_t reject_reason;
	uint8_t flags;
	uint8_t drain_copy_index;
};

struct TraceSlot {
	px4::atomic<bool> busy{false};
	px4::atomic<uint32_t> committed_sequence{0};
	TraceEvent event{};
};

struct AtomicCounters {
	px4::atomic<uint32_t> request_publish_count{0};
	px4::atomic<uint32_t> request_retry_count{0};
	px4::atomic<uint32_t> request_publish_failure_count{0};
	px4::atomic<uint32_t> server_request_count{0};
	px4::atomic<uint32_t> server_response_count{0};
	px4::atomic<uint32_t> server_response_publish_failure_count{0};
	px4::atomic<uint32_t> client_response_seen_count{0};
	px4::atomic<uint32_t> client_response_drain_copy_count{0};
	px4::atomic<uint32_t> client_response_accepted_count{0};
	px4::atomic<uint32_t> client_response_rejected_count{0};
	px4::atomic<uint32_t> timeout_count{0};
	px4::atomic<uint32_t> trace_drop_count{0};
};

TraceSlot g_trace[TraceDepth];
AtomicCounters g_counters;
px4::atomic<uint32_t> g_next_sequence{0};
px4::atomic<uint32_t> g_next_client_instance{1};
px4::atomic<bool> g_dumping{false};
px4::atomic<uint32_t> g_capture_state{static_cast<uint32_t>(CaptureState::Armed)};
px4::atomic<uint32_t> g_trigger_reason{static_cast<uint32_t>(NearMissTriggerReason::None)};
px4::atomic<uint32_t> g_trigger_sequence{0};
px4::atomic<uint32_t> g_trigger_client_instance{0};
px4::atomic<uint32_t> g_frozen_sequence{0};
px4::atomic<uint32_t> g_active_writers{0};

void finishWrite()
{
	if ((g_active_writers.fetch_sub(1) == 1)
	    && (static_cast<CaptureState>(g_capture_state.load()) == CaptureState::Freezing)) {
		g_frozen_sequence.store(g_next_sequence.load());
		g_capture_state.store(static_cast<uint32_t>(CaptureState::Frozen));
	}
}

uint32_t writeEvent(const TraceEvent &event)
{
	CaptureState capture_state = static_cast<CaptureState>(g_capture_state.load());

	if ((capture_state == CaptureState::Freezing) || (capture_state == CaptureState::Frozen)) {
		return 0;
	}

	g_active_writers.fetch_add(1);
	capture_state = static_cast<CaptureState>(g_capture_state.load());

	if ((capture_state == CaptureState::Freezing) || (capture_state == CaptureState::Frozen)) {
		finishWrite();
		return 0;
	}

	const uint32_t sequence = g_next_sequence.fetch_add(1) + 1;
	TraceSlot &slot = g_trace[(sequence - 1) % TraceDepth];
	bool expected_busy = false;

	if (!slot.busy.compare_exchange(&expected_busy, true)) {
		g_counters.trace_drop_count.fetch_add(1);
		finishWrite();
		return sequence;
	}

	slot.event = event;
	slot.committed_sequence.store(sequence);
	slot.busy.store(false);

	if ((static_cast<CaptureState>(g_capture_state.load()) == CaptureState::Capturing)
	    && (sequence >= g_trigger_sequence.load() + PostTriggerEvents)) {
		uint32_t expected_state = static_cast<uint32_t>(CaptureState::Capturing);

		if (g_capture_state.compare_exchange(&expected_state, static_cast<uint32_t>(CaptureState::Freezing))) {
			// Existing writers finish normally; FREEZING prevents new writers from entering.
		}
	}

	finishWrite();
	return sequence;
}

void triggerNearMiss(NearMissTriggerReason reason, uint16_t client_instance, uint32_t sequence)
{
	if (sequence == 0) {
		return;
	}

	uint32_t expected_state = static_cast<uint32_t>(CaptureState::Armed);

	if (g_capture_state.compare_exchange(&expected_state, static_cast<uint32_t>(CaptureState::Triggering))) {
		g_trigger_reason.store(static_cast<uint32_t>(reason));
		g_trigger_sequence.store(sequence);
		g_trigger_client_instance.store(client_instance);
		g_capture_state.store(static_cast<uint32_t>(CaptureState::Capturing));
	}
}

const char *triggerReasonString(NearMissTriggerReason reason)
{
	switch (reason) {
	case NearMissTriggerReason::Retry:
		return "RETRY";

	case NearMissTriggerReason::Reject:
		return "REJECT";

	case NearMissTriggerReason::None:
	default:
		return "NONE";
	}
}

const char *captureStateString(CaptureState state)
{
	switch (state) {
	case CaptureState::Armed:
		return "ARMED";

	case CaptureState::Triggering:
		return "TRIGGERING";

	case CaptureState::Capturing:
		return "CAPTURING";

	case CaptureState::Freezing:
		return "FREEZING";

	case CaptureState::Frozen:
		return "FROZEN";

	default:
		return "UNKNOWN";
	}
}

TraceEvent clientEvent(EventType type, uint16_t client_instance, const dataman_request_s &request,
		       uint16_t attempt)
{
	TraceEvent event{};
	event.timestamp = hrt_absolute_time();
	event.request_timestamp = request.timestamp;
	event.expected_index = request.index;
	event.client_instance = client_instance;
	event.attempt = attempt;
	event.event_type = static_cast<uint8_t>(type);
	event.expected_client_id = request.client_id;
	event.expected_request_type = request.request_type;
	event.expected_item = request.item;
	return event;
}

TraceEvent serverEvent(EventType type, const dataman_request_s &request, const dataman_response_s *response)
{
	TraceEvent event = clientEvent(type, 0, request, 0);

	if (response != nullptr) {
		event.response_timestamp = response->timestamp;
		event.actual_index = response->index;
		event.actual_client_id = response->client_id;
		event.actual_request_type = response->request_type;
		event.actual_item = response->item;
		event.response_status = response->status;
	}

	return event;
}

} // namespace

uint16_t allocateClientInstance()
{
	return static_cast<uint16_t>(g_next_client_instance.fetch_add(1));
}

void recordClientRequest(EventType type, uint16_t client_instance, const dataman_request_s &request,
			 uint16_t attempt, bool publish_success, bool trigger_on_sync_retry)
{
	TraceEvent event = clientEvent(type, client_instance, request, attempt);

	if (publish_success) {
		event.flags |= FlagPublishSuccess;

	} else {
		g_counters.request_publish_failure_count.fetch_add(1);
	}

	g_counters.request_publish_count.fetch_add(1);

	if (type == EventType::ClientRequestRetry) {
		g_counters.request_retry_count.fetch_add(1);
	}

	const uint32_t sequence = writeEvent(event);

	if (trigger_on_sync_retry && (type == EventType::ClientRequestRetry)) {
		triggerNearMiss(NearMissTriggerReason::Retry, client_instance, sequence);
	}
}

void recordClientResponse(EventType type, uint16_t client_instance, const dataman_request_s &request,
				  const dataman_response_s &response, uint16_t attempt, uint8_t reject_reason,
				  uint8_t drain_copy_index)
{
	TraceEvent event = clientEvent(type, client_instance, request, attempt);
	event.response_timestamp = response.timestamp;
	event.actual_index = response.index;
	event.actual_client_id = response.client_id;
	event.actual_request_type = response.request_type;
	event.actual_item = response.item;
	event.response_status = response.status;
	event.reject_reason = reject_reason;
	event.drain_copy_index = drain_copy_index;

	if (response.timestamp < request.timestamp) {
		event.flags |= FlagResponseOlderThanRequest;
	}

	switch (type) {
	case EventType::ClientResponseSeen:
		g_counters.client_response_seen_count.fetch_add(1);
		g_counters.client_response_drain_copy_count.fetch_add(1);
		break;

	case EventType::ClientResponseAccepted:
		g_counters.client_response_accepted_count.fetch_add(1);
		break;

	case EventType::ClientResponseRejected:
		g_counters.client_response_rejected_count.fetch_add(1);
		break;

	default:
		break;
	}

	writeEvent(event);
}

void recordClientTimeout(uint16_t client_instance, const dataman_request_s &request, uint16_t attempt)
{
	g_counters.timeout_count.fetch_add(1);
	writeEvent(clientEvent(EventType::ClientTimeout, client_instance, request, attempt));
}

void recordServer(EventType type, const dataman_request_s &request, const dataman_response_s *response,
		  bool publish_success)
{
	TraceEvent event = serverEvent(type, request, response);

	if (type == EventType::ServerRequestReceived) {
		g_counters.server_request_count.fetch_add(1);

	} else if (type == EventType::ServerResponsePublish) {
		g_counters.server_response_count.fetch_add(1);

		if (publish_success) {
			event.flags |= FlagPublishSuccess;

		} else {
			g_counters.server_response_publish_failure_count.fetch_add(1);
		}
	}

	writeEvent(event);
}

Counters getCounters()
{
	return {
		.request_publish_count = g_counters.request_publish_count.load(),
		.request_retry_count = g_counters.request_retry_count.load(),
		.request_publish_failure_count = g_counters.request_publish_failure_count.load(),
		.server_request_count = g_counters.server_request_count.load(),
		.server_response_count = g_counters.server_response_count.load(),
		.server_response_publish_failure_count = g_counters.server_response_publish_failure_count.load(),
		.client_response_seen_count = g_counters.client_response_seen_count.load(),
		.client_response_drain_copy_count = g_counters.client_response_drain_copy_count.load(),
		.client_response_accepted_count = g_counters.client_response_accepted_count.load(),
		.client_response_rejected_count = g_counters.client_response_rejected_count.load(),
		.timeout_count = g_counters.timeout_count.load(),
		.trace_drop_count = g_counters.trace_drop_count.load(),
	};
}

void printCounters()
{
	const Counters counters = getCounters();
	PX4_INFO("trace req pub/retry/fail: %" PRIu32 "/%" PRIu32 "/%" PRIu32,
		 counters.request_publish_count, counters.request_retry_count, counters.request_publish_failure_count);
	PX4_INFO("trace server req/resp/fail: %" PRIu32 "/%" PRIu32 "/%" PRIu32,
		 counters.server_request_count, counters.server_response_count, counters.server_response_publish_failure_count);
	PX4_INFO("trace client seen/accept/reject: %" PRIu32 "/%" PRIu32 "/%" PRIu32,
		 counters.client_response_seen_count, counters.client_response_accepted_count,
		 counters.client_response_rejected_count);
	PX4_INFO("trace active-drain copies: %" PRIu32, counters.client_response_drain_copy_count);
	PX4_INFO("trace timeout/drop: %" PRIu32 "/%" PRIu32, counters.timeout_count, counters.trace_drop_count);
}

void printNearMissStatus()
{
	const CaptureState state = static_cast<CaptureState>(g_capture_state.load());
	const NearMissTriggerReason reason = static_cast<NearMissTriggerReason>(g_trigger_reason.load());
	PX4_INFO("near-miss trigger mode: RETRY_ONLY");
	PX4_INFO("near-miss state: %s", captureStateString(state));
	PX4_INFO("near-miss trigger: %s seq=%" PRIu32 " client_instance=%" PRIu32,
		 triggerReasonString(reason), g_trigger_sequence.load(), g_trigger_client_instance.load());
	PX4_INFO("near-miss frozen: %s frozen_seq=%" PRIu32 " trace_drop_count=%" PRIu32,
		 state == CaptureState::Frozen ? "true" : "false", g_frozen_sequence.load(),
		 g_counters.trace_drop_count.load());
}

void dump()
{
	bool expected_dumping = false;

	if (!g_dumping.compare_exchange(&expected_dumping, true)) {
		return;
	}

	const uint32_t end_sequence = g_next_sequence.load();
	const uint32_t start_sequence = end_sequence > TraceDepth ? end_sequence - TraceDepth + 1 : 1;
	PX4_ERR("DMTRACE BEGIN seq=%" PRIu32 "..%" PRIu32, start_sequence, end_sequence);
	printNearMissStatus();
	printCounters();

	for (uint32_t sequence = start_sequence; sequence <= end_sequence; sequence++) {
		TraceSlot &slot = g_trace[(sequence - 1) % TraceDepth];
		bool expected_busy = false;

		if (!slot.busy.compare_exchange(&expected_busy, true)) {
			continue;
		}

		if (slot.committed_sequence.load() == sequence) {
			const TraceEvent &event = slot.event;
			PX4_INFO_RAW("DMTRACE A s=%" PRIu32 " t=%" PRIu64 " e=%u ci=%u a=%u d=%u req=%" PRIu64 "\n",
				sequence, event.timestamp, event.event_type, event.client_instance, event.attempt,
				event.drain_copy_index, event.request_timestamp);
			PX4_INFO_RAW("DMTRACE B s=%" PRIu32 " x=%u/%u/%u/%" PRIu32 " p=%" PRIu64
				     " y=%u/%u/%u/%" PRIu32 "/%u r=%02x f=%02x\n",
				sequence, event.expected_client_id, event.expected_request_type, event.expected_item,
				event.expected_index, event.response_timestamp, event.actual_client_id,
				event.actual_request_type, event.actual_item, event.actual_index, event.response_status,
				event.reject_reason, event.flags);
		}

		slot.busy.store(false);
	}

	PX4_ERR("DMTRACE END seq=%" PRIu32, end_sequence);
	g_dumping.store(false);
}

} // namespace dataman_trace
