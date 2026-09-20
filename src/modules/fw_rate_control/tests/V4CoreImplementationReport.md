# V4 core implementation — e624

Date: 2026-09-20
HEAD: e624a99f2955addbf76681e636c44162a0c03055
Branch: v4-slow-memory-ut-e624
Status: uncommitted core-only implementation. No FW Run wiring, parameter/message
changes, allocator changes, Fast, flash, push, or flight test. Existing analysis/
was neither read nor modified.

## Implementation boundary

ResidualSlowFeedforwardMemory.hpp/.cpp is a fixed-storage, uORB-independent
core backed by a reference to the real RateControl. Config/Inputs/Result expose
configuration, freshness/context and diagnostic state. CMake builds a separate
library; FixedwingRateControl does not link/call it as a module dependency.

History: 128 fixed entries (q, microsecond timestamp), bounded scans, no runtime
allocation. Window uses real span, retains its bracketing boundary sample,
and requires 61 samples at 20 Hz for 3 seconds. Oversampling beyond capacity
cannot falsely shorten the required real span; integration should supply
nominal 20 Hz evidence. Config window supports .05 to 6 seconds. Prototype
defaults are not a flight-validated tuning claim.

Only fresh accepted timestamps update q_hat/history. Duplicate/backward samples
cannot learn. update also checks actual time on no-sample ticks. >150 ms gaps
invalidate the estimate/window. Allocator UNKNOWN and maneuver/unstable context
invalidate learning history and hold B; UNKNOWN-to-fresh must rebuild the full
window. Allocator age conversion remains future routing; the test adapter
converts timestamp age <=100 ms to the production freshness boolean.

B/integrator authoritative faults cause explicit reasoned recovery, reset Roll-I,
clear B/evidence/gate, synchronize epoch and install valid limits. Invalid
configuration is rejected, retaining previous valid limits and disabling
learning. Estimator-only NaN invalidates evidence without resetting native I/B.
Hard reset is an explicit input for future landed/disarm routing.

Disable, non-Mission, pilot abort and failsafe use paired handback while legal.
An initiated handback completes before learning can reenter. Exit step uses
control_dt bounded at 150 ms; learning uses fresh evidence dt. Ordinary maneuver
does not initiate exit. Partial opposite innovation can stop before zero.
Reaching zero clears history and requires the next complete fresh window.

## Atomic pair

No preview was added to the old mutating V3 API. Instead:

1. Core reads I/B/epoch and computes a feasible candidate within the intersection
   of BMAX, native IMAX, slew-selected request and reversal zero barrier.
2. Core validates finite final pair, shifted bounds, actual delta conservation
   and total conservation before any authoritative write.
3. New RateControl::commitRollMemoryPair rechecks the expected I/B/epoch/limit
   snapshot and the same HR=0 bounds, then writes native I and the passed owned
   memory reference with no callbacks, clipping or fallible operation between.
4. Postcommit debug assertions verify finite state, bounds and conservation.
   Runtime safety relies on explicit validation, not assertions.

This is a logical transaction under SINGLE CONTROLLER THREAD ownership, not
cross-thread atomic hardware writes. Future integration must serialize native
integration, resets, configuration and core calls. Do not publish/use state
between operations. Existing applyRollITransfer and updateIntegral bodies are
unchanged.

The representable pre-transfer total is used to form I_post = total - B_post;
actual deltas and conservation are checked to 2e-7. This avoids accumulated
subtraction drift. A capped/rounded-to-zero B request does not rewrite I.
Only the final actual B delta is logged as accepted. q history is not shifted.
Residual/sign statistics are reinterpreted after transfer without changing
timestamps/count or creating new learning authorization.

## Adapter audit

Subject seeds fixture state/faults through friend access, converts Inputs,
calls the production APIs, and reads Result. No learning, cap, gate, reversal,
recovery or output-B equation is implemented in the adapter. Test natural
integration calls real RateControl::update; output uses production composeOutput.
The prescribed native-I samples are fixture stimuli, not onboard setters.

Original 31-test source SHA256, unchanged from test-first:
8de0cfd95a5ffd4fe232668e5cc92f4dcf923acc7315a1de4cd8f8b4118b2811

No oracle values or tolerances changed. Historical test-first report is retained
as historical LINK_RED evidence; this report supersedes its implementation status.

## Verification and exit codes

- Baseline normal configure/build/regression chain: exit 0, 103/103 PASS.
- First new-core build: exit 1 (-Wfloat-equal on explicit snapshot/config comparisons).
  Corrected only new implementation comparisons; no warning suppression or oracle edit.
- New-core build and phases A/B/C/D: exit 0; 9/7/4/11 tests PASS.
- Regression rebuild and rerun after RateControl additions: exit 0, 103/103 PASS:
  RateControl 17, BumplessRollITransfer 72, allocator 5, FixedB0Experiment 9.
  Golden exporter enabled with existing temporary output directory; no skipped test.
- Final V4 configure/build/test: exit 0, original 31/31 PASS.
- Supplemental production safety tests: exit 0, 9/9 PASS:
  rejected joint commits preserve both states; successful commit isolates axes;
  no-publication timeout boundary; allocator requalification;
  full negative reversal and post-zero window; abort/failsafe handback;
  hard reset/control-invalid epochs; estimator-only NaN retains native authority;
  100000 paired transfers preserve total without inventing evidence.
- Restored V4_SLOW_MEMORY_CONTRACT_RED=OFF: configure and normal core/regression
  target build exit 0.
- git diff --check: exit 0.

Logs (local /tmp):
v4-core-baseline-tests.log, v4-core-regression-build.log,
v4-core-regression-tests.log, v4-core-phase-{a,b,c,d}.log,
v4-core-final-v4-build.log, v4-core-final-v4-tests.log,
v4-core-off.log, v4-core-off-build.log.
Build target is px4_sitl_test for host units, not a production firmware build.

## Source audit

| Check | Result | Evidence |
|---|---|---|
| Double B | PASS | composeRaw adds B once; composeOutput invokes it once; no FW call |
| HR zero | PASS | no HR Config member; bounds/context/joint commit all use literal 0 |
| Accepted only | PASS | proposed states validated then committed; no requested authoritative assignment |
| Rebase not evidence | PASS | transfer changes neither q history nor last timestamp; may clear qualification at zero |
| Maneuver hold | PASS | no handback for maneuver/stable-gate failure |
| Zero crossing | PASS | accepted step stops at -B; zero invalidates window |
| Limit recovery | PASS | illegal new pair causes reset before new native limit; no one-sided B clamp |
| V3 unchanged | PASS | only additive RateControl methods/friend; FW/V3/old tests byte-identical to HEAD |

All old production function bodies are unchanged. No uORB or parameters were
modified. Native-I bounds/anti-windup still run in RateControl.

## Next-stage obligations

READY_FOR_FW_INTEGRATION=YES means ready for separately authorized integration,
not a tested airborne implementation. Remaining: real timestamp/allocator routing,
mode/landed/disarm events, parameter-update ordering, single-thread ownership,
one-and-only-one raw insertion replacing S, output/native-integration log timing,
ULog diagnostics, production board builds and bench validation.

FLIGHT_READY=NO

