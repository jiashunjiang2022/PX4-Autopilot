# Control allocator saturation feedback fix — 2026-09-17

## Identity and scope

Baseline production: a644ca7868f07004e45658d4ae9a7a8c880c6bb5.
Parent investigation HEAD: 69e6d33f96a26d2f6506a37ed0c6839f441b8e65.
Starting branch adaptive-tail-trim-v1-design-red-20260917 was clean.
New branch: fix-fw-allocator-saturation-feedback-20260917.
Worktree: /Users/jiangjiashun/PX4/PX4-Autopilot-v4-fixed-b0-exp-20260913.

Only production change: FixedwingRateControl.cpp routing/consumption.
No RateControl algorithm, Slow/Bumpless transfer, parameters, Tail-Trim design/tests, Fast, allocator or uORB changes. No push or flash.

## Root cause and RED before production edit

Investigation evidence remains unchanged in its original report and in the original two test cases.
Non-VTOL paths both used subscription instance 0. The first update consumed a fresh generation even when no differential-thrust axis used it. The surface branch's second update then returned false.

New desired-behavior tests compile the actual routing block extracted from FixedwingRateControl.cpp at CMake configure time. Extraction uses stable surrounding anchors and CMAKE_CONFIGURE_DEPENDS; missing anchors fail configuration. The generated include is build-only. This avoids maintaining a separately fixed test copy.
Harness uses real uORB subscriptions and real RateControl. Thin wrappers count update calls and forward setters to RateControl; no fake transport or integrator. It supplies vehicle/mask context without constructing the full module scheduler. The existing tailsitter mask swap is outside the extracted block and unchanged.

Before production edit, functional target compiled and ran: 5 tests, 3 PASS / 2 FAIL.
Failed desired cases:
- DesiredNonVtolSurfaceAndNegativeSymmetry: expected Roll positive-setter calls 1, actual 0; expected one instance-0 read, actual 2. Expected I=0, observed +.0019995896 for positive error and -.0019995896 for negative error. Second no-publication cycle grew magnitude to .0039991792.
- DesiredMixedAxesShareOneSample: Roll differential-thrust path worked, surface Pitch/Yaw setters were missing and their integrals grew instead of staying zero; instance-0 reads=2 rather than 1.
- DesiredVtolKeepsIndependentInstances already PASS, as expected.

RED log: /tmp/allocator-fix-red.log. Failures came from runtime behavioral assertions, not compilation.

## Minimal fix

Cache bool status0_updated from the single instance-0 update.
Process existing differential-thrust mask against that sample.
For surfaces: if VTOL, independently update instance 1 into the status variable; otherwise use status0_updated and its already-read sample.
No second update or copy of instance 0. No setters run for an instance without a fresh update.
The two axis loops, thresholds and setter semantics remain unchanged.

## GREEN and coverage

After production edit, generated test block rebuilt from the changed production source:
functional-AllocatorSaturationFeedback: 5/5 PASS.

| Required case | Evidence |
|---|---|
| A ordinary non-VTOL surfaces | desired surface test: setter called once, real Roll I=0 |
| B differential-thrust Roll | mixed test: Roll setter once, I=0 |
| C mixed Roll/Pitch/Yaw | same status0 drives all three exactly once; all three I=0 |
| D no fresh publication | second run adds no setter call; retained flag keeps I=0 |
| E negative symmetry | negative status/error gives I=0, one read and one setter |
| F absolute IMAX | preserved investigation case: prolonged integration clamps at .2 |
| VTOL independent instances | instance0 Roll-positive, instance1 Pitch/Yaw-negative; corresponding errors all suppressed, each instance read once |

The investigation's old extracted buggy path intentionally remains and still reproduces the old defect. It is labelled investigation-only; its PASS asserts historical evidence, not repaired behavior. Desired cases above use the current source extraction.

Regressions:
- unit-BumplessRollITransfer: 72/72 PASS (golden export environment set).
- unit-rate_control_test: 17/17 PASS.
- unit-FixedB0Experiment: 9/9 PASS.
- modules__fw_rate_control in px4_sitl_test configuration: compile/link PASS.
- Full px4 executable in px4_sitl_test: FAIL, unrelated host compile issue described below.

Commands:
```sh
cmake -S . -B build/px4_sitl_test -DCONFIG=px4_sitl_test
ninja -C build/px4_sitl_test functional-AllocatorSaturationFeedback unit-BumplessRollITransfer unit-rate_control_test unit-FixedB0Experiment
build/px4_sitl_test/functional-AllocatorSaturationFeedback
B2B_ADAPTIVE_GOLDEN_DIR=/tmp/tail-trim-v1-evidence build/px4_sitl_test/unit-BumplessRollITransfer
build/px4_sitl_test/unit-rate_control_test
build/px4_sitl_test/unit-FixedB0Experiment
ninja -C build/px4_sitl_test px4
ninja -C build/px4_sitl_test modules__fw_rate_control
```

Local logs: /tmp/allocator-fix-{config,build,red,green-build,green,b2b,rate,fixed,sitl-build,module-build}.log.
Functional runner reports existing work-queue setup warnings; these synchronous tests do not schedule work. No complete module-loop runtime claim is made.

## Full-build limitation

Full build failed at unchanged src/systemcmds/tests/test_uart_send.c:83:
`fatal error: 'sprintf' is deprecated ... [-Wdeprecated-declarations]`
MacOSX27.0 SDK marks sprintf deprecated; project -Werror promotes it to error.
The modified FixedwingRateControl.cpp compiled successfully in that build, and its module library separately linked.
No unrelated source fix or warning suppression was applied. Therefore SITL_TEST_BUILD is FAIL, not PASS. Full build validation remains outstanding before complete implementation/safety unblocking.

## VTOL and stale flags

VTOL still reads motors/diff-thrust from instance 0 and surfaces from instance 1. Tests use opposite signs to catch accidental instance merging. Tailsitter roll/yaw mask swap is untouched.
STALE_FLAG_RETENTION_EXISTING_BEHAVIOR=YES. Without a fresh status, previous saturation flags remain. The test confirms no new event; no timeout/clearing policy was added. Any stale-data policy is separate work.

## Scientific and safety boundaries

Current V3 accepted I/S conservation is unchanged: neither transfer nor RateControl arithmetic changed. Natural integral history may differ now that direction suppression reaches it; absolute IMAX is unchanged and tested.
No historical 9.17 anomaly attribution: no flight log evidence was analyzed.
This addresses the allocator-message routing gap only; it does not resolve other Tail-Trim design review gaps (maneuver policy, clipping feedback, physical transaction validation). READY_TO_UNBLOCK_TAIL_TRIM=NO pending full build and those independent reviews.

## Diff audit and disposition

Changed: one production .cpp; test-only CMake; existing feedback test; new generated-include template; this report.
git diff --check PASS. Original Tail-Trim documents and RED tests unchanged.
ROOT_CAUSE_REPRODUCED=YES
RED_BEFORE_FIX=YES
ONE_READ_PER_INSTANCE=YES
PRODUCTION_FILES_CHANGED=src/modules/fw_rate_control/FixedwingRateControl.cpp
SITL_TEST_BUILD=FAIL (host SDK deprecation in unrelated unchanged source)
No production changes beyond routing. STOP.
