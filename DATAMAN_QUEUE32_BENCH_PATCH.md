# Dataman Queue-32 Candidate A Bench Patch

## Scope and baseline

- Repository: `/Users/jiangjiashun/PX4/PX4-Autopilot`
- Branch: `air`
- Baseline commit: `e44285af8dd9d066e27fddd4ec8975b619bb530f`
- Candidate: A, enlarge the Dataman request and response uORB queues only.
- Existing Dataman transaction instrumentation and the 512-slot one-shot near-miss trace are retained unchanged.
- No firmware was flashed while preparing or validating this patch.

## Functional changes

`msg/DatamanRequest.msg` now contains:

```text
uint8 ORB_QUEUE_LENGTH = 32
```

`msg/DatamanResponse.msg` now contains:

```text
uint8 ORB_QUEUE_LENGTH = 32
```

No DatamanClient sync/async state machine, retry interval, timeout, response matching, MissionManager, Navigator, storage backend, task priority, locking, or server poll/operate/respond logic was changed. Candidate B transaction serialization was not implemented, and no upstream commit was cherry-picked.

## Generated-code verification

The FMUv6C build regenerated the topic metadata as follows:

```cpp
ORB_DEFINE(dataman_request, struct dataman_request_s, 75, 1010689725u,
           static_cast<orb_id_size_t>(ORB_ID::dataman_request), 32);
ORB_DEFINE(dataman_response, struct dataman_response_s, 72, 1017987279u,
           static_cast<orb_id_size_t>(ORB_ID::dataman_response), 32);
```

The third argument is `o_size_no_padding`; the final argument is the configured queue depth. Both final arguments are exactly 32.

## Diagnostics and interpretation

The existing instrumentation remains available for:

- client request initial publish and retry;
- server request receive, operation complete, and response publish;
- client response seen, accepted, and rejected;
- timeout, publication failure, trace drop, and one-shot near-miss capture.

This PX4/uORB version exposes configured queue size and subscriber update state, but does not expose a non-invasive historical queue-backlog high-water mark to DatamanClient or the dataman server. `uorb top` can confirm the configured queue depth and rates, but it cannot report the maximum pending request or response backlog seen during the run. No uORB-core instrumentation was added.

The substitute evidence is:

- request path: initial/retry publication counters compared with server-receive counters, plus per-event timestamps and sequence numbers;
- response path: server-response counters compared with client seen/accept/reject events and matching fields;
- queue pressure: publication-to-server-receive and response-publication-to-client-seen trace latency and any missing stage in a frozen trace.

These aggregate counters are not a transaction-atomic backlog measurement. In particular, every DatamanClient has a subscription to the shared response topic, so aggregate `client_response_seen_count` must not be interpreted as one seen event per server response.

### Duplicate retry identification

No transaction protocol field was added. For a synchronous request, the retry reuses the same request and therefore the same `request.timestamp`. Two server-receive events with the same client ID, request type, item, index, and request timestamp demonstrate that both the initial request and its retry were executed. The trace `attempt` and event sequence identify which publication was the retry and whether a later response was accepted. This is sufficient to diagnose duplicate synchronous execution from the existing trace, subject to the 512-event capture window.

## Memory

Final FMUv6C linker report:

```text
FLASH:       1,815,940 B / 1,920 KiB = 92.36%
AXI SRAM:       90,636 B /   512 KiB = 17.29%
```

GNU ELF summary:

```text
text       data     bss      dec
1811520    4420     86540    1902480
```

The queue buffers are allocated on first publication by `uORB::DeviceNode::write()` using `o_size * o_queue`; they are not statically placed in the linked AXI `.bss`. Consequently, the linked FLASH and AXI SRAM figures are unchanged from the immediately preceding instrumented build.

Runtime queue-buffer request sizes are:

| Topic | `sizeof(message)` | Depth 1 | Depth 32 | Payload-buffer delta |
|---|---:|---:|---:|---:|
| `dataman_request` | 80 B | 80 B | 2,560 B | +2,480 B |
| `dataman_response` | 72 B | 72 B | 2,304 B | +2,232 B |
| Total | | 152 B | 4,864 B | +4,712 B |

The table reports bytes requested for uORB payload buffers. Allocator metadata and cache-alignment overhead are implementation-dependent and are not included. FMUv6C has ample margin for this approximately 4.6 KiB runtime-heap increase.

## Files and diff summary

Candidate A changes:

```text
msg/DatamanRequest.msg   | 2 ++
msg/DatamanResponse.msg  | 2 ++
2 files changed, 4 insertions(+)
```

This report adds `DATAMAN_QUEUE32_BENCH_PATCH.md`. The working tree also contains the previously requested Dataman instrumentation/near-miss changes in `src/lib/dataman_client` and `src/modules/dataman`; they were preserved and were not functionally altered by Candidate A.

## Validation

```text
git diff --check
PASS (no output)

make px4_fmu-v6c_default
PASS (844/844 build steps in the clean queue-depth rebuild)

final incremental confirmation
ninja: no work to do.
```

The generated request and response topic sources both contain `ORB_DEFINE(..., 32)`. The linker emitted the existing RWX LOAD-segment warning; it did not fail the build.

## Flash and bench commands

Before either test, remove propellers, keep the vehicle disarmed, and save the current QGroundControl Plan. `mission_stress.py` repeatedly replaces and clears the onboard Mission. Do not arm, and do not run SIH or the full bench suite.

Flash, only when ready:

```sh
cd /Users/jiangjiashun/PX4/PX4-Autopilot
make px4_fmu-v6c_default upload
```

Prepare the upstream test outside the working tree:

```sh
BENCH_TMP=$(mktemp -d /tmp/px4-mission-stress.XXXXXX)
git archive upstream/main Tools/bench_test | tar -x -C "$BENCH_TMP"
python3 -m pip install --user pymavlink pyserial
ls -1 /dev/tty.usbmodem* /dev/tty.usbserial* 2>/dev/null
```

### A. RAM backend, Logger off, USB only, 100 x 220

On NSH, configure and reboot before the run:

```text
param set SYS_DM_BACKEND 1
param save
reboot
```

After reconnecting, ensure Logger is off, then run on the Mac with the actual USB device:

```text
logger stop
```

```sh
python3 "$BENCH_TMP/Tools/bench_test/bench/mission_stress.py" \
  /dev/tty.usbmodemXXXX --iterations 100 --items 220
```

### B. SD backend, Logger on, USB plus RADIO, 100 x 220

On NSH, configure and reboot before the run:

```text
param set SYS_DM_BACKEND 0
param save
reboot
logger on
logger status
```

Run with the actual USB and telemetry-radio serial devices and configured baud rates:

```sh
python3 "$BENCH_TMP/Tools/bench_test/bench/mission_stress.py" \
  /dev/tty.usbmodemXXXX /dev/tty.usbserialYYYY \
  --baudrate 115200 --baudrate2 57600 --iterations 100 --items 220
```

The script alternates complete iterations between link 1 and link 2; it does not issue simultaneous mission operations on both links.

After each mission test has finished, collect diagnostics from NSH. Do not dump the trace during the stress run:

```text
dataman trace status
dataman trace dump
dataman status
mavlink status
uorb top dataman_request dataman_response
top once
dmesg
```

Record the mission script PASS/FAIL summary and, for any failure, its phase, iteration, sequence, and timeout. Compare counter deltas from before and after each phase, and inspect the frozen trace for retry/reject recovery and any duplicate synchronous execution.

## Status

Candidate A queue-32 firmware is compiled and ready for controlled bench A/B testing. It is diagnostic bench firmware, not flight-ready firmware.
