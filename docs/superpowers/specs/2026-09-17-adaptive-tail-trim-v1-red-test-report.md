# Adaptive Tail-Trim V1 RED test report — 2026-09-17

Historical report below preserves the original design-stage evidence. Current pure-core stage update: RED05/10/16 now perform numeric behavior checks, not field detection. Before core implementation all three failed against a documented legacy V3 migration adapter. After core implementation those three PASS against AdaptiveTailTrimCore; original RED01–04 still fail against unchanged V3. Current filtered contract suite: 16 tests, 12 PASS / 4 expected RED, exit 1. No claim that V3 acquired physical semantics. See 2026-09-17-adaptive-tail-trim-v1-core-implementation-report.md for the full RED→GREEN trace and scope.

Base a644ca7868f07004e45658d4ae9a7a8c880c6bb5; branch adaptive-tail-trim-v1-design-red-20260917.
Worktree /Users/jiangjiashun/PX4/PX4-Autopilot-v4-fixed-b0-exp-20260913.
Production files unchanged. Only docs, one new test translation unit and opt-in test-only CMake changed.

## Execution and scope

Before adding new tests:
- CMake configure and Ninja build succeeded (host unit tests; no firmware implementation/build).
- BumplessRollITransfer: 72/72 PASS, including golden export with B2B_ADAPTIVE_GOLDEN_DIR set.
- RateControl: 17/17 PASS.
- Host lacks optional Gazebo/Java dependencies; neither is needed for these targets. No dependencies installed.

New suite compiles and links. Exit code 1 comes from seven assertions, not compile/fixture failure.
16 probes: **4 Behavior RED + 3 API RED + 9 PASS**. Numeric tolerance 2e-6 declared before execution, unchanged.
RED01–03 intentionally test the migration hypothesis S_numeric=b_numeric. They do not claim current V3 violates its own raw-coordinate conservation. RED04 supplies external pitch context deliberately absent from current API and observes genuine V3 growth. No hypothetical controller was implemented.
API probes use valid C++ SFINAE capability detection on concrete proposed input/output fields; they compile successfully, then assert that the future interface is present. They do not claim behavioral implementation verification. Renaming future fields requires updating adapters, not relaxing contracts.
The opt-in translation unit includes the unchanged existing test source to reuse fixtures. This also registers its 72 tests in that binary; the explicit filter below selects exactly 16 new probes. Ordinary existing targets are untouched.

## Per-case evidence

All test names have prefix AdaptiveTailTrimContract.

| Test | Contract / expected future behavior | Observed current behavior / reason | Classification | Current V3 broken? |
|---|---|---|---|---|
| RED01_PhysicalTrimMeaningIndependentOfG | C2 same stored b, same tail contribution at two g | S=.04 gives .02545454 vs .05090909 tail at g=.7/1.4 | Behavior RED: state meaning differs | NO |
| RED02_BumplessTransferUsesPhysicalTorqueInvariant | C5/C6 accepted physical mismatch zero | raw accepted ΔI+ΔS passes; treating ΔS as Δb gives -.004 torque mismatch | Behavior RED: cannot relabel raw transaction | NO |
| RED03_PhysicalBMaxIsTailCoordinateBound | C1/C2 cap .10 in tail units | valid raw S=.10, g=2.2 gives tail=.20 | Behavior RED: raw cap is not physical cap | NO |
| RED04_SharedAxisReserveBlocksTrimGrowth | C13 pitch .97, reserve .05 forbids growth from zero | actual adaptive gate opens, S grows to .03020051 after 300 updates | Behavior RED: no pitch authority input | NO |
| RED05_PositiveNegativeResidualReserveReportedSeparately | C13 separate tail reserve outputs | Result lacks roll_reserve_pos/neg; existing raw headroom is different | API RED | NO |
| RED06_GrowthRequiresGate | C8 false gate prevents growth | false gate, S remains zero | PASS inherited raw mechanism; physical implementation still pending | NO |
| RED07_ReleaseDoesNotRequireGate | C9 release with false gate | S decreases from .08 | PASS inherited | NO |
| RED08_ReversalMustReachZeroFirst | C10 no direct crossing | unwind reaches zero without negative crossing before zero | PASS inherited | NO |
| RED09_ReversalRequiresFreshFullGateAfterZero | C11 fresh 3 s at .02 s | offsets 0..149 blocked; first gate offset 150 | PASS inherited | NO |
| RED10_ManeuverLearningCanBeDisabledOrFrozen | learning-only freeze while rate feedback remains available | Inputs lacks learning_allowed; eligibility is not a separate learning context | API RED; maneuver behavior must be added later | NO |
| RED11_NormalExitExactHandbackWhenFeasible | feasible handback maintains physical contribution | g*(I+S) preserved with fixed g, S reaches zero | PASS inherited, does not prove b-coordinate transfer | NO |
| RED12_NormalExitDecaysAuthorityWhenHandbackImpossible | C12/C16 native IMAX and bounded decay | at I=.2, HR=1 legal starting state exits without raising I | PASS inherited raw decay; physical API still needed | NO |
| RED13_SafetyExitFinite | C16 pilot abort finite exit | .04 S reaches zero within four .02-s steps at safety slew 1 | PASS inherited; not all triggers proved by this probe | NO |
| RED14_ResetMismatchCannotLeaveHiddenTrim | C15 complete caller handshake | helper requests reset; Roll reset + synchronizeReset clears S before output | PASS inherited complete handshake | NO |
| RED15_NonfiniteGCannotDivide | C14 invalid g never produces nonfinite output | 0,-1,NaN,Inf enter recovery with finite diagnostic torque | PASS inherited; positive g_min inversion boundary deferred | NO |
| RED16_NoPhysicalTrimBeyondServoGeometry | C3/C4 context/API needed to classify arbitrary candidate geometry | Inputs lacks tail_pitch_context and residual_roll_tail | API RED; not a geometry sweep or proof of servo limits | NO |

## Fixture correction, not production change

Initial RED14 incorrectly expected the helper itself to clear state before its caller handled reset_required. Source review of FixedwingRateControl.cpp:405 and :118 confirmed the real recovery handshake. The probe was corrected to exercise that handshake; no assertion threshold changed and no production edit was made. Its initial failure is NOT counted as a contract gap.

## Reproduction

Run from the named worktree. Initial baseline commands precede additions in this task.

```sh
cmake -S . -B build/px4_sitl_test -G Ninja -DCONFIG=px4_sitl_test
ninja -C build/px4_sitl_test unit-BumplessRollITransfer unit-rate_control_test
mkdir -p /tmp/tail-trim-v1-evidence
B2B_ADAPTIVE_GOLDEN_DIR=/tmp/tail-trim-v1-evidence build/px4_sitl_test/unit-BumplessRollITransfer
build/px4_sitl_test/unit-rate_control_test
cmake -S . -B build/px4_sitl_test -DADAPTIVE_TAIL_TRIM_CONTRACT_RED=ON
ninja -C build/px4_sitl_test unit-AdaptiveTailTrimContract
build/px4_sitl_test/unit-AdaptiveTailTrimContract --gtest_filter='AdaptiveTailTrimContract.*'
# Expected exit 1: seven future-contract failures, nine inherited passes.
cmake -S . -B build/px4_sitl_test -DADAPTIVE_TAIL_TRIM_CONTRACT_RED=OFF
```

Default OFF excludes the new target and its CTest registration; explicit ON deliberately adds failing future tests to that testing configuration. This option does not affect firmware source. Local evidence logs reside in /tmp/tail-trim-v1-evidence; paths are ephemeral. Reproducible commands and exact assertion values above are the durable record.
After adding the suite, the unchanged baseline binaries were rerun: 72/72 and 17/17 PASS. Reconfiguration with the option OFF succeeded; `ctest --test-dir build/px4_sitl_test -N -R AdaptiveTailTrim` reported Total Tests: 0. The working build configuration was left OFF. `git diff --check` passed.

## Acceptance boundary

CURRENT_V3_TESTS = PASS (72+17)
TAIL_TRIM_RED_TESTS = RED (16 compile; 7 fail, 9 pass)
BEHAVIOR_RED_COUNT = 4
API_RED_COUNT = 3
CURRENT_V3_REGRESSION = PASS
PRODUCTION_SOURCE_CHANGED = NO
NEW_CONTROL_IMPLEMENTED = NO
REAL_FLIGHT_AUTHORIZED = NO

These probes establish meaningful missing semantics, not full future safety coverage. Future GREEN closure must replace the raw-S adapter with the physical implementation, add partial transfer/gain-freeze tests, tiny-positive-g protection, both-sign reserve sweeps, stale inputs, allocator feedback and all exit triggers. STOP at design.
