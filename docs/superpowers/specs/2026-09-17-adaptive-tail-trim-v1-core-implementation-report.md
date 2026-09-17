# Adaptive Tail-Trim V1 pure-core implementation report — 2026-09-17

## Identity and authorization boundary

START_COMMIT=e624a99f2955addbf76681e636c44162a0c03055
BRANCH=adaptive-tail-trim-v1-core-20260917
WORKTREE=/Users/jiangjiashun/PX4/PX4-Autopilot-v4-fixed-b0-exp-20260913

Allocator prerequisite CLOSED at the starting commit; no routing or anti-windup changes in this stage.
Only known initial dirty state: NuttX submodule untracked tools/jlink-nuttx. Preserved; no clean/reset/stash/removal or submodule commit.
SHA256 before AND after builds: 3c5fbd59358b5efe7586c0be90ebb4cd7b376f9371e889ca36468276b1375c66.
Size=50712; mtime=1789654939, both unchanged. Before FMU build, Makefile.host dry run confirmed no work for this file. Normal FMU .so generation invokes that no-op target, without altering the existing binary.

PHASE A contracts; PHASE B numerical RED; PHASE C unintegrated pure core; PHASE D regression/build verification completed.
No FixedwingRateControl integration, parameter/msg/logger changes, Fast, estimator, allocator or real-flight work.

## Design closure

Design §27 is normative for this stage. Math adds C19–C26:
- one g_cycle snapshot, by-value API;
- accepted partial physical conservation;
- maneuver freeze dominance and safety priority;
- directional reserve formulas;
- small-g rejection before inverse;
- no new maneuver reversal evidence;
- future-only pre-allocator clipping diagnostic requirement.

Release not requiring persistence gate does NOT permit release during normal maneuver. Estimator/gate and causal pitch-envelope generation remain external future obligations.
g_safe_min is explicit test/config input; deployment value TO_BE_SELECTED_BY_REPLAY_SITL. Test .01 is a fixture value, not a flight parameter.

## TDD history and evidence

1. Docs commit 29319e9c02 locks the core contract.
2. Tests commit 3725055427 added a buildable test-only migration adapter using actual V3 and real RateControl, plus numerical desired tests. No new production core existed at that commit.
3. Ran tests before implementation:
   - strengthened old contract suite: 16 total, 9 PASS / 7 RED, exit 1;
   - new core-contract suite against legacy adapter: 25 total, 1 PASS / 24 RED, exit 1.
4. Added production helper and reran without weakening assertions or tolerances:
   - 25/25 GREEN; then five supplemental invalid-input/config/handshake/directional tests, final 30/30 GREEN.
5. Feature commit a8814a0f00 contains helper and final tests.

Two test compiler diagnostics about explicit float-to-double promotions were corrected before collecting runtime RED. They are NOT counted as RED evidence. After the new header was created, Ninja initially retained the __has_include fallback binary; changed selection to an explicit production include with optional TAIL_TRIM_LEGACY_PROBE macro, rebuilt, and verified real helper execution. Stale-binary output is NOT counted as implementation evidence.

Legacy adapter limitation: it probes the invalid migration S_numeric=b_numeric. It exposes actual V3 raw headroom outputs, not a fabricated physical reference; pitch/maneuver inputs have no legacy counterpart and are not secretly implemented in the adapter. Some new diagnostic flags do not exist in legacy and are default false. Numeric RED assertions, not field existence, establish the required new behavior. The adapter is test-only and is bypassed in final core tests. Historical V3 failures are not V3 regression defects.

### Upgraded original API probes

| Probe | Runtime RED before core | GREEN after core |
|---|---|---|
| RED05 | actual raw headroom values 0/0 cannot equal .60/.80 or .80/.60 | numerical directional reserves for b=±.10, pitch=.30 |
| RED10 | V3 updates state during maneuver, including ordinary release; cannot exactly hold .04/.06 | 300-cycle exact holds, shared-axis safety override |
| RED16 | low-pitch case lacks valid physical admission/progress, physical feasibility expectation fails | .90/.15 blocks; .40 allows; no optimistic feasibility |

No remaining API-only checks in these three tests.
The new core suite additionally checks pre-maneuver latched unwind and refuses a new latch inside maneuver.

### Core RED→GREEN matrix

The initial suite's 24 runtime RED cases, all GREEN against the new core:
PhysicalTrimIndependentOfG; PhysicalBMax; DirectionalReservePositiveBias; DirectionalReserveNegativeBias;
SharedAxisBlocksGrowth; SharedAxisAllowsGrowth; GrowthRequiresGate; NormalManeuverFreezesGrowth;
NormalManeuverFreezesNormalRelease; SafetyReleaseOverridesManeuverFreeze; PartialAcceptedTransfer;
BumplessPhysicalInvariant; NativeICapacityLimitsTransfer; SmallGRejected; NaNInfRejected; SlewBound;
ZeroFirstReversal; NoCrossingBeforeZero; SafetyExitCanDecayWithoutExactHandback; ResetLeavesNoHiddenTrim;
FrozenGRecorded; TransferMismatchFiniteSafetyCase; NoNewManeuverReversalAndPriorLatchUnwinds;
GeometrySweepNeverOptimisticallyFeasible.

TransferMismatchNearZeroNormalCase passed against the legacy adapter because its short raw-gate scenarios made no movement. This is not counted as a new RED success. Nonzero accepted movement is independently required by BumplessPhysicalInvariant, PartialAcceptedTransfer and FrozenGRecorded before checking conservation.

Supplemental boundary tests, not claimed as pre-implementation RED:
InvalidConfigAndInitialStateRequireRecovery; NativeBoundsAndInvalidDtRejectWithoutMovement;
NormalReleaseWithoutGateAndLearningDisabledHold; ReversalRearmRequiresFreshGateHandshake;
IndependentDirectionalMinimaLimitBothSigns.

## Pure-core architecture and exact ordering

Header-only AdaptiveTailTrimCore.hpp: fixed scalar state/config; no dynamic allocation, growing history, I/O, uORB, parameters, exceptions or control-chain calls.
Config has b_max, b_slew, g_safe_min, directional minima and verified K_A.
Inputs are values: dt, gain snapshot, pitch context, current I/bounds, external target/gate/maneuver/safety/latch/validity/epoch.
Result contains requested/accepted deltas, state before/after, used gain, physical mismatch, trim torque, three reserves, limiting/mode/validity/reset flags.

Ordering:
1. Copy input; initialize finite output diagnostics.
2. Explicit reset/epoch mismatch clears state and mechanical histories first, even if g invalid. This is identified reset removal, not a normal handoff.
3. Validate immutable config, state validity, inputs, positive dt, g>g_safe_min, finite/ordered I bounds and current I within bounds.
4. Compute directional interval and whether current state violates it.
5. Select safety/established unwind/freeze/normal; bound target by BMAX; apply gate/maneuver/zero-first policy.
6. Record bounded pre-authority/slew/I requested delta and requested matching I.
7. Intersect normal target with shared interval; slew limit.
8. Limit normal b movement to matching I capacity. Safety uses bounded toward-zero b and clips I compensation, never violates native limits.
9. Round float candidate inward at boundaries; up to four fixed attempts. Reject without committing if no finite bounded candidate passes.
10. Compute accepted deltas/mismatch and reserve diagnostics; atomically commit owned b and mechanical mode state.

Double intermediates avoid overflow from finite float input differences and conditioning; returned values/state are float. Extreme unrepresentable requested conversions reject rather than generate infinite outputs. Config is immutable per instance and defaults are deliberately unusable until explicitly set.
No actual RateControl state is mutated. Future integration must apply the accepted I delta against the exact original I snapshot and coordinate b commit/rollback. This essential caller transaction integration is NOT implemented.

### Physical and partial semantics

Stored b=.05 gives tau_trim=.055 at g=.5/.8/1.2. b is never gain-scaled.
Example test: g=.7, requested b step=.010, shared interval limits accepted b to approximately .004, accepted I=-(1.1/.7)*.004. Native I capacity can further shrink b movement.
FrozenGRecorded changes external simulated gain to .8 after capturing .7 in Inputs; g_used remains .7, accepted pair conserves with .7 and deliberately would not conserve with .8.

### Maneuver, release and reversal

Maneuver OR learning_allowed=false freezes ordinary state changes exactly, including release.
Shared violation and safety_release_required override freeze; caller maps pilot abort/failsafe/control invalid/explicit disable into the latter. Invalid numeric inputs take rejection priority.
Safety exits latch until zero under valid inputs; this stage uses b_slew for both normal and safety movement. Exact I compensation when possible; otherwise finite flagged authority decay.
Only trusted non-maneuver context can establish unwind. Existing unwind persists into maneuver and only approaches zero. At zero, reversal_reached_zero is emitted; stale gate/latch cannot grow opposite trim. A trusted false-gate/false-latch handshake rearms mechanics; full GWIN belongs to future estimator, not this core.
Finite-time progress requires valid input/config and representable positive step; no wall-clock convergence claim for rejected or sub-resolution transactions.

### Shared authority

R_pos=1-abs(pitch_context)-b; R_neg=1-abs(pitch_context)+b; R_sym=min.
For .30 pitch and +.10 b: .60/.80; negative b swaps them.
Normal admissible interval = [reserve_neg_min-room, room-reserve_pos_min] intersect [-b_max,b_max].
An empty interval or existing violation triggers only toward-zero safety decay. During bounded recovery it can remain infeasible; feasible=false is preserved. No assertion that future residual Roll is zero or that achieved servo geometry is already solved.
105 pitch/state/target sweep combinations test honest feasibility; independent unequal reserve tests establish different positive/negative caps.

### Small g, invalid state and reset

g=0, negative, NaN, Inf, tiny-positive and exactly g_safe_min reject before division: accepted deltas zero, finite b held, recovery_required true.
Invalid constructor state becomes zero and requires reset; invalid config never enables transactions.
Epoch reset clears b and mechanical histories, sets reset_applied, transaction_valid=false and zero accepted handoff deltas. b_before/b_after disclose removal. Caller/actuator handling is future work.

## Numerical evidence

Predeclared state/reserve tolerance=2e-7; normal torque mismatch tolerance=2e-6. Neither loosened.
Maximum measured normal mismatch=6.12011885615e-10.
Maximum measured safety mismatch=0.0109999999404 in deliberate saturated-I release, bounded by K_A*abs(accepted_delta_b) plus tolerance.
These are suite maxima, not a proof over all possible inputs or a flight latency measurement.

## Verification and exit codes

| Command/target | Result |
|---|---|
| cmake px4_sitl_test configuration; Ninja test-target build | exit 0 |
| unit-AdaptiveTailTrimCore | 30 PASS / 0 FAIL, exit 0 |
| unit-AdaptiveTailTrimContract, filter AdaptiveTailTrimContract.* | 12 PASS / 4 expected RED, exit 1 |
| functional-AllocatorSaturationFeedback | 5 PASS / 0 FAIL, exit 0 |
| unit-rate_control_test | 17 PASS / 0 FAIL, exit 0 |
| unit-BumplessRollITransfer with golden directory | 72 PASS / 0 FAIL, exit 0 |
| make px4_sitl_default | exit 0 |
| make px4_fmu-v6c_default | exit 0 |
| standalone Cortex-M7 hard-float compilation of core::step | exit 0 |

Old-suite remaining RED01–04 deliberately still exercise V3 raw semantics; RED05/10/16 now exercise new core behavior. Thus 12/4 does not mean V3 was changed. Do not relabel the entire historical suite GREEN.
Functional uORB runner still emits its existing work-queue setup warnings; tests are synchronous. No full module runtime/SITL flight loop was run.
Production targets compile the existing flight firmware; new core is not linked into it. Additional arm-none-eabi-g++ -std=c++17 -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-d16 -fno-exceptions -fno-rtti -Wall -Wextra -Werror probe compiles the new core separately, not into firmware.

Reproduction:
```sh
cmake -S . -B build/px4_sitl_test -DADAPTIVE_TAIL_TRIM_CONTRACT_RED=ON
ninja -C build/px4_sitl_test unit-AdaptiveTailTrimCore unit-AdaptiveTailTrimContract functional-AllocatorSaturationFeedback unit-rate_control_test unit-BumplessRollITransfer
build/px4_sitl_test/unit-AdaptiveTailTrimCore
build/px4_sitl_test/unit-AdaptiveTailTrimContract --gtest_filter='AdaptiveTailTrimContract.*'
build/px4_sitl_test/functional-AllocatorSaturationFeedback
build/px4_sitl_test/unit-rate_control_test
B2B_ADAPTIVE_GOLDEN_DIR=<existing-evidence-directory> build/px4_sitl_test/unit-BumplessRollITransfer
make px4_sitl_default
make px4_fmu-v6c_default
cmake -S . -B build/px4_sitl_test -DADAPTIVE_TAIL_TRIM_CONTRACT_RED=OFF
```

Detailed local logs and ARM probe: /tmp/tail-core.2Z62zo/ (temporary, not versioned).
RED logs red.log / contract-red.log; GREEN core-final.log / contract-final.log; regression *-final.log; build sitl.log / fmuv6c.log / arm-core-compile.log.
Historical RED can also be reproduced at tests commit 3725055427 where no core header exists. Do not overwrite the active worktree to do so.

## Resource evidence

FMUv6C actual linker report:
- FLASH 1,847,868 B / 1920 KB = 93.99%.
- AXI_SRAM 91,212 B / 512 KB = 17.40%.
- Other listed ITCM/DTCM/SRAM/BKPRAM regions: 0 used.
ELF sections: .text=1,843,432; .init_section=348; .ARM.exidx=8; .data=4,080; .bss=87,116 bytes.
arm-none-eabi-nm found no AdaptiveTailTrimCore symbol in the flight ELF; source search found no production caller outside the new header. No runtime stack/CPU measurement or isolated before/after firmware size delta claimed.

## Source/diff audit

Relative to e624a99, FixedwingRateControl.cpp, BumplessRollITransfer.hpp, RateControl, allocator, uORB, params/msg and Fast are unchanged.
Allowed diff only: core header, core/contract test files and legacy test adapter, test CMake, Tail-Trim docs/plan/report.
Known NuttX untracked file remains identical. No extra unknown dirty state. git diff --check PASS.
Four commits: docs contracts; behavioral RED tests; pure core; status/report. No push.

## Remaining integration blockers — NOT implemented

A exact torque insertion/timing and atomic native-I/core handshake.
B existing scheduled trim interaction.
C unconstrained vs controller-clipped demand diagnostics and evidence-based feedback decision.
D real causal pitch-envelope generation, availability and invalid-data policy.
E B_hat estimator, maneuver classification, fresh evidence and full reversal GWIN.
F parameter units/migration/deployment numerical selection.
G logging schemas and sample epochs.
H closed-loop SITL dynamics and saturation tests.
I hardware bench, resource/stack/timing checks, then separate flight authorization.

PURE_CORE_PHASE=PASS
FIXEDWING_TAIL_TRIM_INTEGRATED=NO
CURRENT_FLIGHT_BEHAVIOR_CHANGED_BY_TAIL_TRIM=NO
REAL_FLIGHT_AUTHORIZED=NO
FAST_ACTUAL_AUTHORIZED=NO

STOP. Task 7 integration remains NOT STARTED.
