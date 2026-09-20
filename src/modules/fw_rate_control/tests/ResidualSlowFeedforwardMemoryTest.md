# V4 test-first contract (e624)

Baseline: e624a99f2955addbf76681e636c44162a0c03055.
Branch: v4-slow-memory-ut-e624. No commit made.

## Status and scope

31 assertion-bearing tests; opt-in COMPILE_RED (specifically LINK_RED).
The test translation unit compiles. Linking exits 1 solely because
v4_contract::Subject methods are deliberately declared but not implemented.
This is API RED, not an observed failure of production V3 behavior.
No reference-model implementation supplies expected outputs.

V4UT04/05 call real RateControl bounds directly; they are inherited primitive
checks, not evidence that V4 exists. Other tests require a future adapter to
real V4/native RateControl. Do not implement the algorithm in that adapter.

The only tracked existing file changed is fw_rate_control/CMakeLists.txt,
adding an OFF-by-default test option. Production control, parameters, uORB,
allocator, Fast, and existing tests are unchanged.

## Interface and oracle conventions

State seeds and readback are test-only access. Timestamp units are microseconds.
sample represents fresh plant/native-I evidence; tick supplies no new evidence.
warm supplies 61 samples spanning a full 3 seconds at 20 Hz, not 60 samples
spanning only 2.95 seconds. q_hat updates before computing that sample's
innovation/learning request. No future sample or gap filling is allowed.
gain=.1 is a test fixture value, NOT a deployed parameter recommendation.

requestPair exercises the joint accepted-pair boundary after permission and
slew selection; it is not an alternative learning entry point. BMAX/native/
total limits and reversal zero barrier still apply. sample must exercise
permission, estimator, gained-increment and slew together (V4UT28).
naturalIntegrate must call the actual RateControl natural integration.
disableStep must exercise real paired handback, not assign the expected result.
output must use the real future raw composition path (full FW integration
remains separate).

Absolute tolerance 2e-7 is declared in the test before execution.
No tolerance was adjusted based on new V4 results: no V4 test body has run.
V4UT22 estimator-only NaN requires invalid estimate/gate and no learning;
it does not invent a compulsory native-I reset for estimator-only invalidity.

## Reproduce

From this repository:

```sh
cmake -S . -B build/px4_sitl_test -G Ninja -DCONFIG=px4_sitl_test
cmake --build build/px4_sitl_test --target unit-BumplessRollITransfer unit-rate_control_test functional-AllocatorSaturationFeedback unit-FixedB0Experiment -j 6
ctest --test-dir build/px4_sitl_test -R '^(unit-BumplessRollITransfer|unit-rate_control_test|functional-AllocatorSaturationFeedback|unit-FixedB0Experiment)$' --output-on-failure -V
cmake -S . -B build/px4_sitl_test -DV4_SLOW_MEMORY_CONTRACT_RED=ON
cmake --build build/px4_sitl_test --target unit-ResidualSlowFeedforwardMemory -j 6
cmake -S . -B build/px4_sitl_test -DV4_SLOW_MEMORY_CONTRACT_RED=OFF
```

The V3 golden exporter requires B2B_ADAPTIVE_GOLDEN_DIR pointing to an existing
temporary directory; otherwise its one existing test skips by design.

## Actual verification

- Final normal configure: exit 0.
- Regression build: exit 0.
- CTest four targets: exit 0; RateControl 17 PASS, FixedB0 9 PASS,
  allocator 5 PASS, B2B initially 71 PASS + 1 golden-export skip.
- B2B rerun with B2B_ADAPTIVE_GOLDEN_DIR=/tmp/v4-test-first-golden.m7VoYm:
  exit 0, 72/72 PASS. Final aggregate: 103 PASS, 0 FAIL, 0 remaining skip.
- Opt-in configure: exit 0.
- V4 object compilation: succeeds. Link/build exit 1: undefined Subject methods.
- Initial configuration was interrupted during dependency acquisition;
  a local-cache attempt exited 1 because the old cache lacked Abseil targets.
  Reverting the cache overrides and using normal fetched dependencies succeeded.
  No production workaround was applied.

Local full logs: /tmp/v4-test-first-config-retry.log,
 /tmp/v4-test-first-regression-build.log, /tmp/v4-test-first-regressions.log,
 /tmp/v4-test-first-b2b-full.log, /tmp/v4-test-first-red-build.log.

## Remaining integration specifications (not implemented)

- Full FixedwingRateControl disabled equivalence across multiple cycles, native
  integral update/output timing, raw B insertion exactly once, frozen g,
  existing trim/clamp and allocator path.
- Real uORB timestamp/freshness routing, no-publication timeout, freshness
  boundary values, unknown-to-fresh full 3-second requalification.
- Actual native-I/owned-B atomic commit visibility and recovery before a changed
  limit reaches output; actual epoch wiring.
- Pilot abort, failsafe, landed/disarm, rate-control authority changes and
  recovery reason/ULog field propagation.
- Full persistent-negative learning trajectory (UT16 tests the accepted
  correction and subsequent hold), positive authorization after the complete
  fresh reverse window (UT18 forbids early authorization), and long-run
  numeric/replay/integration validation.

These are not production/integration PASS claims. Ready for implementation
means ready to replace the undefined adapter with actual tested production
logic in a separately authorized stage, not flight readiness.

## Test names

- V4UT01_NativeDisabledEquivalence
- V4UT02_PositiveAcceptedTransfer
- V4UT03_RequestedNotAccepted
- V4UT04_PositiveResidualBounds
- V4UT05_NegativeResidualBounds
- V4UT06_TotalIMAXPositive
- V4UT07_TotalIMAXNegative
- V4UT08_CapResidualVisible
- V4UT09_DeadbandHoldsMemory
- V4UT10_DuplicateTimestamp
- V4UT11_GapInvalidatesGate
- V4UT12_StaleAllocatorBothDirections
- V4UT13_SaturationBothDirections
- V4UT14_CommonCoordinateHistory
- V4UT15_RebaseNotEvidence
- V4UT16_PartialReverseStops
- V4UT17_ZeroCrossingBarrier
- V4UT18_FreshReverseWindow
- V4UT19_ManeuverHoldApply
- V4UT20_IMAXShrinkRecovery
- V4UT21_BMAXShrinkRecovery
- V4UT22_NonfiniteState
- V4UT23_EpochMismatch
- V4UT24_LegalPairedHandback
- V4UT25_ControlCompositionBumpless
- V4UT26_NoHiddenHR
- V4UT27_BackwardTimestamp
- V4UT28_GainedIncrementAndSlew
- V4UT29_SignFractionAfterRebase
- V4UT30_StabilizedCannotLearn
- V4UT31_RecoveryRequiresWarmup

