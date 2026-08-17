# Dataman Timeout Instrumentation Plan

## Scope and baseline

- Repository: `https://github.com/jiashunjiang2022/PX4-Autopilot.git`
- Branch: `air`
- Baseline HEAD: `e44285af8dd9d066e27fddd4ec8975b619bb530f`
- Target: `px4_fmu-v6c_default` (Pixhawk 6C Mini / FMUv6C)
- Purpose: diagnose the existing 5000 ms Dataman transaction timeout. This patch does not fix or alter the transaction behavior.
- Not changed: timeout, 100 ms synchronous retry poll, asynchronous retry timing, uORB queue depth, DatamanClient state machine, Mission Manager, Navigator, storage backend, or locking.
- No upstream commit was applied or cherry-picked.

The trace path uses a statically allocated 64-slot RAM ring. It performs no heap allocation and no file I/O. Normal transactions do not print. A ring dump is emitted only after the existing synchronous path has concluded that a request timed out and only when the requested timeout equals 5000 ms.

## Source audit

### Actual request and response fields

`msg/DatamanRequest.msg` defines:

```text
timestamp, client_id, request_type, item, index, data[56], data_length
```

`msg/DatamanResponse.msg` defines:

```text
timestamp, client_id, request_type, item, index, data[56], status
```

`src/modules/dataman/dataman.h` defines the actual `dm_function_t` operations as `DM_GET_ID`, `DM_WRITE`, `DM_READ`, and `DM_CLEAR`; storage is selected by the real `dm_item_t` values such as `DM_KEY_WAYPOINTS_OFFBOARD_0`, `DM_KEY_WAYPOINTS_OFFBOARD_1`, and `DM_KEY_MISSION_STATE`.

### uORB queue depth

Neither Dataman message declares `ORB_QUEUE_LENGTH`. `Tools/msg/templates/uorb/msg.cpp.em:77` defaults it to 1. The generated FMUv6C sources confirm this directly:

```text
build/px4_fmu-v6c_default/msg/topics_sources/dataman_request.cpp:49  ORB_DEFINE(..., 1)
build/px4_fmu-v6c_default/msg/topics_sources/dataman_response.cpp:49 ORB_DEFINE(..., 1)
```

Therefore a newer publication can replace an unread request or response. Each subscription reads the newest generation; there is no multi-message transaction queue here.

### Client/server path

1. A caller constructs `dataman_request_s` and enters `DatamanClient::syncHandler()` in `src/lib/dataman_client/DatamanClient.cpp`.
2. The client publishes once, polls its response subscription for 100 ms, and republishes the same synchronous request after a poll timeout. The enclosing API's supplied timeout remains unchanged.
3. The single Dataman worker in `src/modules/dataman/dataman.cpp` polls one `dataman_request` subscription, copies the newest request, runs the selected RAM/file backend operation, copies `client_id/request_type/item/index` into the response, stamps `response.timestamp`, and publishes it.
4. For ordinary synchronous requests, the client accepts only equality of `client_id`, `request_type`, `item`, and `index`.
5. For initial `DM_GET_ID`, where the request client ID is `CLIENT_ID_NOT_SET`, the server places the request timestamp in `response.data`; the client uses that timestamp to identify its reply.
6. The asynchronous client uses the same four ordinary identity fields. Its retry timing and state transitions are unchanged.

There is no ordinary-response request timestamp echo and no stale-response rejection in this baseline. A response whose server timestamp is older than the current request timestamp is recorded as a diagnostic flag only; the patch does not reject it.

### Navigator feasibility path

`MissionBase::check_mission_valid()` in `src/modules/navigator/mission_base.cpp` creates `MissionFeasibilityChecker` and calls `checkMissionFeasible()`. `src/modules/navigator/mission_feasibility_checker.cpp:78-89` iterates all `mission.count` items and calls the shared Navigator client's `readSync()` without an explicit timeout, so the declared 5000 ms default applies. A failed read returns false and leads to the existing `mission check failed` warning.

### MAVLink Mission Manager path

Every `MavlinkMissionManager` owns a `DatamanClient` (`src/modules/mavlink/mavlink_mission.h`). Upload handling selects the inactive waypoint bank and performs `writeSync()` for received items, then `writeSync(DM_KEY_MISSION_STATE, 0, ...)` and publishes `mission`. Download handling performs synchronous reads. This activity can overlap Navigator's feasibility reads because the request and response topics are global and depth 1.

## Instrumentation design

### Ring and concurrency

- 64 statically allocated slots; no `malloc`, `new`, `free`, or file writes.
- One atomic global sequence reserves event order.
- Each slot has a nonblocking atomic busy/commit protocol. A producer never waits; a collision increments `trace_drop_count`.
- Dump takes a slot only long enough to print one committed event, so it does not allocate a large stack snapshot.
- Each client receives a trace-only `client_instance` number. It does not replace or alter Dataman's server-issued `client_id`.

### Event codes

```text
1 client initial request publish
2 client retry publish
3 server received request
4 server completed operation
5 server published response
6 client copied/saw response
7 client accepted response
8 client rejected response
9 client reached timeout
```

Each event is split across two compact lines with the same `s` sequence so all fields fit PX4's 127-byte log-message limit:

```text
DMTRACE A: s sequence, t hrt event time, e event code,
           ci trace client instance, a attempt, req request timestamp
DMTRACE B: s sequence, x expected client_id/request_type/item/index,
           p response timestamp,
           y response client_id/request_type/item/index/status,
           r rejection mask, f flags
```

Actual rejection mask bits, based only on existing filters:

```text
0x01 wrong client_id
0x02 wrong request_type
0x04 wrong item
0x08 wrong index
0x10 DM_GET_ID request-timestamp mismatch
```

Flags:

```text
0x01 publish() returned true
0x02 response.timestamp < current request.timestamp (diagnostic only)
```

### Counters

`dataman status` now reports:

```text
request_publish_count
request_retry_count
request_publish_failure_count
server_request_count
server_response_count
server_response_publish_failure_count
client_response_seen_count
client_response_accepted_count
client_response_rejected_count
timeout_count
trace_drop_count
```

`server_response_count` counts response publication attempts; the adjacent failure counter distinguishes a failed `publish()` return.

### Answering the three diagnostic questions

A. A matching event 1 or 2 with `f & 0x01` proves a client publication call returned true. A successful publish without a later matching event 3, while later requests do reach event 3 and `trace_drop_count==0`, is evidence consistent with depth-1 request replacement.

B. Event 3 proves the server copied that request. Correlate `req`, `client_id`, `request_type`, `item`, and `index`. For synchronous retries the request timestamp is intentionally unchanged, so ordering shows that at least one attempt arrived but cannot uniquely number which retry; no protocol field was changed to add such an ID.

C. Event 5 with `f & 0x01` proves the server's response publish returned true. Event 6 proves this client copied a response. Event 8 and `rr` give every real baseline rejection reason. Event 5 without a matching event 6, followed by newer response traffic and with no trace drops, is evidence consistent with depth-1 response replacement. Event 7 proves acceptance.

The ring is intentionally finite. Conclusions based on absence require that the relevant interval is still present and `trace_drop_count` is zero; positive events remain direct evidence.

## Why transaction behavior is unchanged

- Every original publish still occurs at the same control-flow location and exactly once; its existing boolean return is only observed.
- The original 100 ms poll/retry and caller-provided timeout are unchanged.
- All original matching `if` conditions and state transitions remain in place.
- The diagnostic rejection helper mirrors those conditions but does not feed the decision.
- No subscription is added and no queued response is drained by the trace.
- No queue depth, task priority, backend, mutex, Mission Manager, or Navigator code is changed.
- Runtime cost is bounded atomic/copy work into RAM. There is no normal-path printf. Printing begins only after the pre-existing 5000 ms timeout decision.

This is minimal-impact instrumentation, not zero-impact instrumentation: atomic operations and RAM copies add a small bounded scheduling cost. That limitation must be considered when comparing timeout frequency.

## Upstream comparison (analysis only)

### `b4d5eca3c0`

This commit adds `_client_id == CLIENT_ID_NOT_SET` guards to synchronous and asynchronous public operations. It fails fast if client initialization never obtained a server ID, rather than waiting for a response timeout. It does not address a valid initialized client losing a request/response during mission contention. It was not applied.

### `5fd7fed8d8`

This commit adds `clearPendingResponse()` before each new sync/async request, draining responses left from an aborted or timed-out prior operation. It also makes zero-size DatamanCache construction/resizing safe, aborts operations before freeing/resizing cache storage, repairs indices/state on resize, and adds Dataman tests. The pending-response drain is directly relevant if this trace shows old matching or mismatching responses influencing a later transaction, but it is a behavioral change and was not applied.

### MAVLink mission shared-state mutex (`a4e8430407`, `81ab99c2a9`)

`a4e8430407` protects shared static mission IDs, counts, CRCs, current sequence, and related read/update paths across MAVLink instances. `81ab99c2a9` replaces the recursive design with a normal statically initialized mutex and a `_locked()` helper.

The observed reproduction with RADIO `rx=0` lowers these commits' priority as the necessary cause of this Dataman timeout: no second inbound mission protocol stream is required, while the timeout occurs inside Navigator's independent `DatamanClient::readSync()` amid USB upload and global depth-1 Dataman topics. This points more directly at request/response delivery and matching contention.

The mutex fixes are not disproved. A second `MavlinkMissionManager` still exists; its periodic `send()` and `check_active_mission()` paths read/update shared static state and can react to the global `mission` topic without receiving mission packets on RADIO. Thus RADIO `rx=0` removes the strongest dual-receiver trigger but not all cross-instance shared-state access. The correct priority is lower, not zero. The trace experiment should first establish whether the timed-out Navigator request published, reached the server, and/or lost or rejected its response.

## Modified files

```text
src/lib/dataman_client/DatamanTrace.hpp       trace API, event/reason definitions, counter snapshot
src/lib/dataman_client/DatamanTrace.cpp       fixed RAM ring, atomic counters, timeout dump
src/lib/dataman_client/DatamanClient.hpp      trace instance and async attempt bookkeeping
src/lib/dataman_client/DatamanClient.cpp      client publish/seen/accept/reject/timeout probes
src/lib/dataman_client/CMakeLists.txt         builds the trace implementation in dataman_client
src/modules/dataman/dataman.cpp               server receive/complete/publish probes and status counters
src/modules/dataman/CMakeLists.txt             links the server to the shared trace implementation
DATAMAN_TIMEOUT_INSTRUMENTATION_PLAN.md       this audit and runbook
```

The pre-existing untracked files `listener` and `param` are not part of this patch and were not modified.

Combined diff stat (including the three new files, which plain unstaged `git diff --stat` omits):

```text
 DATAMAN_TIMEOUT_INSTRUMENTATION_PLAN.md       | new
 src/lib/dataman_client/CMakeLists.txt         |   5 +-
 src/lib/dataman_client/DatamanClient.cpp      | 120 +-
 src/lib/dataman_client/DatamanClient.hpp      |   4 +
 src/lib/dataman_client/DatamanTrace.cpp       | new
 src/lib/dataman_client/DatamanTrace.hpp       | new
 src/modules/dataman/CMakeLists.txt            |   2 +
 src/modules/dataman/dataman.cpp               |   8 +-
 8 files changed, 858 insertions(+), 8 deletions(-)
```

## Complete source diff

The exact complete source diff is the working-tree diff below. Because Git omits untracked files from plain `git diff`, use all three commands:

```sh
git diff -- \
  src/lib/dataman_client/CMakeLists.txt \
  src/lib/dataman_client/DatamanClient.cpp \
  src/lib/dataman_client/DatamanClient.hpp \
  src/modules/dataman/CMakeLists.txt \
  src/modules/dataman/dataman.cpp
git diff --no-index /dev/null src/lib/dataman_client/DatamanTrace.hpp || true
git diff --no-index /dev/null src/lib/dataman_client/DatamanTrace.cpp || true
```

To include this document in the complete patch view:

```sh
git diff --no-index /dev/null DATAMAN_TIMEOUT_INSTRUMENTATION_PLAN.md || true
```

## Build and flash

Build only (performed for this patch):

```sh
cd /Users/jiangjiashun/PX4/PX4-Autopilot
git diff --check
make px4_fmu-v6c_default
```

Result: PASS. Link memory report: FLASH `1,814,964 B / 1,920 KiB (92.31%)`; AXI SRAM `65,548 B / 512 KiB (12.50%)`.

Firmware artifact:

```text
build/px4_fmu-v6c_default/px4_fmu-v6c_default.px4
```

Flash later, only after explicitly choosing to do so and with the flight controller connected by USB:

```sh
cd /Users/jiangjiashun/PX4/PX4-Autopilot
make px4_fmu-v6c_default upload
```

No flash was performed while preparing this patch.

## Bench reproduction

Safety conditions:

- Remove propellers and keep the vehicle disarmed.
- Save the current QGroundControl Plan before testing. `mission_stress.py` repeatedly uploads and then clears Mission; the existing onboard Mission will be overwritten and cleared.
- Use `SYS_DM_BACKEND=1` and reboot if reproducing the established RAM-backend case.
- Keep Logger off for the established case.
- Do not run SIH or the full bench suite.

Extract the current upstream bench tool without changing this working tree:

```sh
cd /Users/jiangjiashun/PX4/PX4-Autopilot
BENCH_TMP=$(mktemp -d /tmp/px4-mission-stress.XXXXXX)
git archive upstream/main Tools/bench_test | tar -x -C "$BENCH_TMP"
python3 -m pip install --user pymavlink pyserial
ls -1 /dev/tty.usbmodem*
```

Run the established USB-only test, substituting the detected device exactly:

```sh
python3 "$BENCH_TMP/Tools/bench_test/bench/mission_stress.py" \
  /dev/tty.usbmodemXXXX --iterations 100 --items 220
```

Do not run QGroundControl or another MAVLink client on the same USB serial device during the script.

## Trace and status extraction

The timeout itself emits `DMTRACE BEGIN`, the most recent committed events, and `DMTRACE END` into the NuttX console/dmesg buffer. Immediately after the test (or immediately after seeing a timeout), open an NSH console and run:

```text
dataman status
mavlink status
top once
dmesg
```

Capture the complete console output. On the host, isolate trace lines from saved output with:

```sh
grep 'DMTRACE' nsh-output.txt
```

For each failure report:

```text
mission_stress PASS/FAIL totals
failed phase: upload/download/compare/clear/verify_clear
iteration
seq (when reported)
timeout text/duration
dataman status
mavlink status
top once
full dmesg including DMTRACE BEGIN..END
```

Do not restart Dataman or reboot before collecting `dataman status` and `dmesg`, because doing so can destroy the evidence.

## Expected decision after the run

- Publish event present, no server receive: focus on depth-1 request replacement/scheduling.
- Server receive and response publish present, no client seen: focus on depth-1 response replacement/subscriber scheduling.
- Client seen plus rejection: use the exact `rr` bit and expected/actual fields.
- Client accepted but caller still times out: inspect event order and client identity for a state/observation discrepancy.
- Trace drops nonzero: positive evidence is still usable, but absence is not proof; consider a second run before changing ring size.

This diagnostic firmware is for disarmed bench reproduction only. It is not a flight-ready fix.
