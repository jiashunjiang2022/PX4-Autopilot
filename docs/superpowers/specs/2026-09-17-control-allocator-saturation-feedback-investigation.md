# Control allocator saturation feedback investigation — 2026-09-17

## Scope and identity

Baseline: a644ca7868f07004e45658d4ae9a7a8c880c6bb5.
Branch: adaptive-tail-trim-v1-design-red-20260917.
Starting HEAD: 721ffe87fd7f618b5db35a8399e630d07acf63e0.
Worktree: /Users/jiangjiashun/PX4/PX4-Autopilot-v4-fixed-b0-exp-20260913.
Initially clean. All five investigated production files are byte-identical to the baseline (git diff is empty). No production fix, Tail-Trim design change, firmware build/flash, flight attribution or push.

Hypothesis recorded before experiment: two successive update calls on subscription instance 0 consume the same single-depth publication; with no intervening publication, the second branch is skipped. This report's conclusion follows runtime reproduction, not just that hypothesis.

## Experiment before root-cause conclusion

Added tests/AllocatorSaturationFeedbackTest.cpp under src/modules/fw_rate_control, registered using existing px4_add_functional_gtest with RateControl.
The functional runner initializes real uORB. No generation mock, no replacement RateControl, no private production access or production interface edits.

Test 1: SinglePublicationTwoUpdates
- Advertise actual control_allocator_status with unallocated_torque[0]=.2. Advertisement supplies the one initial sample.
- Confirm actual topic queue depth is 1.
- One real Subscription, repeated updated() both true before read.
- First update=true, received torque=.2.
- updated=false, second update=false; no publication between the two reads.
- copy=true despite no fresh publication; subsequent update remains false.
- Publish .3, updated=true, copy=true and receives .3; updated/update then false.
- This last stage separately establishes that copy also advances the subscriber cursor when reading unread data.

Test 2: NonVtolExtractedPathLosesFlagAndAllowsIntegralGrowth
- Real SubscriptionMultiArray<control_allocator_status_s,2>.
- Extract the branch/index/setter logic from FixedwingRateControl.cpp:352–369 into test scope, retaining both updates and both loops. is_vtol=false; all diffthr_enabled=false.
- Real RateControl, explicitly initialize saturation flags false, Roll I gain=1, IMAX=.2, initial I=0.
- One real advertised sample, positive unallocated Roll torque=.2; no republish.
- Observed first=true, second=false; Roll-positive setter calls=0.
- Execute real RateControl::update with Roll rate error=.1, dt=.02, landed=false.
- I becomes approximately .00199958965; tolerance 1e-8 was specified before running.
- Positive experimental control: identical RateControl with positive saturation explicitly set true. Same input leaves I exactly 0.
- Continue 1000 cycles on affected controller: I reaches exactly .2, confirming the absolute IMAX clamp survives.

This is behavioral reproduction of the extracted routing with real message transport and real integral implementation. It does NOT instantiate FixedwingRateControl::Run, run SITL, inspect a flying vehicle, or read a private flag. Setter reachability is instrumented; the distinct integral response verifies the lost directional suppression. Full scheduled-module behavior is NOT_TESTED.

## Commands and observed result

Run from the worktree:

```sh
cmake -S . -B build/px4_sitl_test -G Ninja -DCONFIG=px4_sitl_test
ninja -C build/px4_sitl_test functional-AllocatorSaturationFeedback
build/px4_sitl_test/functional-AllocatorSaturationFeedback
```

Configure PASS; compile/link PASS; process exit 0; 2/2 tests PASS.
These are investigation tests asserting the existing fault, not tests claiming a fix. They should change expectation in a separately authorized repair task.

Local detailed logs:
- /tmp/allocator-investigation-config.log
- /tmp/allocator-investigation-build.log
- /tmp/allocator-investigation-test.log

Functional runner emits work-queue initialization warnings (not running / lp_default unavailable) during setup. No test schedules work; synchronous uORB advertisement, subscription, publication and RateControl calls all succeed. This is another reason not to claim full module execution. Logs are temporary; the checked-in test and commands are the reproducible evidence.

## Static dataflow, checked against experiment

All source references are repository-relative and unchanged from baseline.

```text
ControlAllocator.cpp:610 requested - allocated torque
  → ControlAllocator.cpp:649 status publication
  → uORBDeviceNode.cpp:191 publisher generation increments
  → SubscriptionMultiArray[0] subscriber cursor
  → FixedwingRateControl.cpp:352 first update()
  → Subscription.hpp:147 passes _last_generation by reference
  → uORBManager.cpp:454 requires new data
  → uORBDeviceNode.hpp:231 successful copy sets cursor=current generation
  → first branch consumes sample even when diffthr_enabled is false
  → FixedwingRateControl.cpp:362 non-VTOL selects SAME [0]
  → no new publication: updates_available=0, update=false
  → :365–366 surface saturation setters skipped
  → previous RateControl saturation flags retained
  → rate_control.cpp:219–227 directional suppression absent if flag was false
  → :240 natural integration proceeds
  → :259/:263 absolute integral clamp still applies
```

Subscription operations:

| Operation | Already-subscribed behavior | Cursor effect |
|---|---|---|
| updated(), Subscription.hpp:131 | tests updates_available | no read/consume |
| update(), :144 | copy only if a new sample exists | successful read updates _last_generation |
| copy(), :157 | allows a read even without new sample | successful read also updates _last_generation; same value if already current |

Important qualification: each operation may lazily call subscribe(). Subscription.cpp:45–58 initializes _last_generation when first attaching. Thus “updated never changes generation” is only accurate for an already-attached subscription, not its initialization side effect.
For this depth-1 topic, DeviceNode.hpp:225–234 assigns the current generation. Queued topics have different cursor advancement and may supply another buffered sample; this test explicitly verifies depth 1.
SubscriptionMultiArray.hpp operator[] returns the existing subscription by reference, not a fresh independent cursor.

## Root cause and boundaries

HIGH confidence for the demonstrated deterministic conditions:
- ordinary non-VTOL selects instance 0 in both reads;
- one new sample, no intervening publication;
- differential-thrust mask false on the ordinary surface axes.

The first update advances the cursor regardless of whether the inner axis mask causes any setters to run. The second update cannot observe that already-consumed sample. This is routing/consumption behavior, not a defect in uORB's documented update semantics or in RateControl's directional clamp.

A publication between calls could let the second branch run; it does not make the logic reliable. No claim about its in-flight frequency is made.
Flags are retained, not automatically cleared: the reproduced initial-false case loses assertion of saturation. A stale true flag could conversely suppress integration after saturation ends; that converse is a mechanism inference, NOT separately reproduced here.
VTOL selects different instances for the two paths and is outside the reproduced non-VTOL case.

## Meaning for existing flights and Slow

1. Can Roll I integrate further into saturation? YES under the demonstrated conditions, with nonzero I gain, appropriate rate error, integral updates enabled and capacity remaining. The real integrator experiment distinguishes this from a static inference.
2. Absolute bound affected? NO under valid finite configuration. Directional anti-windup suppression and IMAX are different mechanisms. rate_control.cpp:263 clamps ordinary I; computeRollILimits :55–56 also intersects transfer-context limits with [-IMAX,+IMAX], used at :259. This experiment directly tests the ordinary clamp, not every HR state.
3. I/S conservation broken? NO direct effect on the accepted pair algebra. This routing issue does not alter the transfer transaction; natural I evolution and the closed-loop burden may change. It does not imply I+S is constant over time. Existing safety-exit authority decay exceptions are unchanged.
4. HR/future Tail-Trim safety affected? YES: any claim that native I reliably receives actual allocator directional feedback needs this path closed. Separate authority logic cannot simply assume the feedback is working. This task does not alter those designs or propose a fix.
5. Historical flight attribution: NOT ESTABLISHED. No 9.17 flight data was inspected here; no anomaly is attributed to this mechanism.

## Four-layer evidence and final status

STATIC_DOUBLE_UPDATE_CONFIRMED = YES
UPDATE_CONSUMES_GENERATION = YES
RUNTIME_SECOND_UPDATE_FALSE = YES
CONTROL_SURFACE_FLAG_LOST = YES (extracted non-VTOL path; zero setter calls)
RATE_I_ANTI_WINDUP_IMPACT_REPRODUCED = YES (real RateControl response plus positive control)
FULL_FIXEDWING_MODULE_RUNTIME = NOT_TESTED

ROOT_CAUSE_CONFIDENCE = HIGH (stated deterministic conditions)
CURRENT_V3_MATH_CONTRACT_AFFECTED = NO (accepted I/S algebra)
CURRENT_I_ABSOLUTE_LIMIT_AFFECTED = NO (valid IMAX)
TAIL_TRIM_DESIGN_BLOCKED = YES (feedback-dependent implementation/safety closure)
PRODUCTION_CODE_CHANGED = NO
FIX_PROPOSED = NO
READY_TO_FIX = YES (behavior-level extracted-path reproduction; separate authorization required)

No production repair is proposed or performed. STOP.
