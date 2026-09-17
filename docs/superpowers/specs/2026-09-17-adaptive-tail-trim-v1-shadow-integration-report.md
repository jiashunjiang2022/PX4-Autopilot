# Adaptive Tail-Trim V1 shadow integration report

Completed locally 2026-09-18 Asia/Shanghai, retaining the requested 2026-09-17
cohort/branch identity. SHADOW_INTEGRATION_PHASE = PASS. This is a software-only
shadow integration result, not permission to fly or enable active Tail-Trim.

## 1. Git identity and scope

Repository: `/Users/jiangjiashun/PX4/PX4-Autopilot-v4-fixed-b0-exp-20260913`.
Start: `7cd690c5bcf55e0ab18eb12749d54383bca3b194`.
Branch: `adaptive-tail-trim-v1-shadow-20260917` (reused after architecture refinement).
Allocator fix `e624a99f2955addbf76681e636c44162a0c03055` remains an ancestor and unchanged.
The actual checkout identifies as the v1.17.0-alpha1 development lineage; no
assumption that the older conversation's v1.16 description identifies this build.

Commits before this report:

| Commit | Role |
|---|---|
| 4112909ac2 | Refined automatic Roll trim architecture |
| 52cbcb0cd6 | Buildable RED contracts/scaffolding |
| 596d2af968 | Persistent detector and virtual coordinator |
| 8f748d1d55 | Production shadow-only wiring, parameters and diagnostic logging |
| 35fa80a178 | Isolation, epoch, reversal, invalid-data and resource tests |

Six commits including this report. Nothing pushed, merged or flashed. Only the
pre-existing NuttX untracked `tools/jlink-nuttx` remains outside committed work;
SHA256 unchanged: `3c5fbd59358b5efe7586c0be90ebb4cd7b376f9371e889ca36468276b1375c66`.
No reset/clean/stash or user-file deletion was performed.

## 2. Architecture and interpretation

Adaptive Tail-Trim V1 = automatic Roll working-point trim driven by persistent
closed-loop burden. It is not a second Slow controller or aerodynamic observer.
The analogy to pressing a Roll trim key is functional only: RC trim, centers,
manual inputs, phi_sp and p_sp are untouched.

```text
actual RateControl -> existing V3 S -> existing g/trim -> torque publication -> allocator
          |                     |              |
          +-- current I --------+-- current S --+-- retained pre-update g
                                               |
                        B_obs -> persistent detector -> b_target
                                               |
              maneuver freeze + virtual capacity + pitch safety admission
                                               |
                              pure core -> virtual b / I -> diagnostic ULog ONLY
```

Shadow helper takes values, with no writable controller/allocator/actuator
references. There is no active-control enable parameter. Core inference uses
fixed storage and bounded computation, with no runtime ML or heap containers.

## 3. Physical B and actual-state epochs

`T_actual_raw = I_actual_raw + S_actual_raw`, `B_obs = g_cycle*T_actual_raw/1.1`.
This is controller-equivalent differential-tail burden, not aerodynamic torque
truth, calibrated servo angle or structural-bias truth. S is read from the live
object, including any nonzero existing exit state. Fully OFF naturally has S=0.

Verified source order in `FixedwingRateControl.cpp`:

| Stage | Current source | Epoch meaning |
|---|---|---|
| First g snapshot | line 383 | Current compression gain times airspeed squared |
| Existing B2B update | line 414 | Actual I/S redistribution |
| RateControl update | line 465 | Computes torque from pre-natural-I, then integrates I |
| Retained g snapshot | lines 471–472 | Still pre-compression-update g |
| Compression update | line 491 | Updates gain for next actual cycle |
| Actual torque publish | line 632 | No Tail-Trim contribution |
| Shadow hook | line 637 | Reads current end-of-cycle states |
| Actual I/S getters | lines 709–710 | Post-natural-I and still-current S |

`rate_control.cpp:183–213` computes torque before natural integration. S does not
change during that integration, so the getters at the hook are a coherent
end-of-cycle pair. They are deliberately NOT the integral pair that formed the
just-published torque. Recovery resets synchronize actual I/S before observation.
No cached B2B Result residual is used; no future sample or gain is read.
`SHADOW_T_ACTUAL_EPOCH_CONSISTENT = YES`, with this precise end-of-cycle meaning.
The integration test runs real RateControl/B2B and checks the observed current sum.

## 4. Minimal persistent detector

`AdaptiveTailTrimEstimator` uses physical B only: causal LPF with
`alpha=dt/(tau+dt)`, current-sample initialization, 20 Hz held observations, a
fixed 200-float ring, population std and same-sign fraction relative to B_hat.
Full fresh `ceil(GWIN*20)` samples are required. No catch-up duplicate samples.
Sign crossing clears persistence. Invalid/maneuver gaps clear LPF and history.
Controller dt must be finite, positive and <=50 ms; coordinator also checks
monotonic elapsed wall time so the controller's clamped dt cannot hide a gap.

TAU/GWIN/GSIGN are read from existing B2B parameters, never changed.
`physical_std_limit = g_cycle*FLAP_B2B_GSTD/1.1`. This does NOT reinterpret raw
.025 as physical .025. Historical observations retain historical g; g variation
can therefore inhibit the gate. No claim of constant-g equivalence during gain
changes. Pitch values, envelope, reserve and stale/nonfinite pitch validity are
absent from this detector's inputs. A global invalid airframe configuration
resets the entire shadow, separately from ordinary pitch sample admission.

## 5. Maneuver gate

Only a causal learning exclusion, not a maneuver model. Uses phi_sp from
`vehicle_attitude_setpoint.q_d` and p_sp from `vehicle_rates_setpoint.roll`.
Quaternion must be finite and unit norm within .01. Both publication timestamps
must be nonzero, no later than observation, and <=200 ms old.

Canonical sources audited:
`fw_att_control/FixedwingAttitudeControl.cpp:100–116` generates stabilized q_d;
`fw_lateral_longitudinal_control/FwLateralLongitudinalControl.cpp:321–327`
generates automatic attitude q_d. Their non-VTOL publishers use
`vehicle_attitude_setpoint`. Rate field units are rad/s in the versioned schema.
VTOL/transition and rate-only modes are not eligible for learning.

Provisional internal thresholds: abs(phi) or abs(p)>10 degrees/degrees per
second enters; both below 5 continuously for 1 s exits. Invalid setpoints
block learning and restart exit dwell. Steady bank remains maneuver even at p=0.
Normal maneuver freezes b and target, clears detector evidence, permits no new
reversal or ordinary release. Only safety/reset/disable and already-established
unwind can reduce state. Fresh full evidence is required after exit.

## 6. Causal pitch safety envelope and mapping

`pitch=(c0+c1)/2`, `roll=(c1-c0)/2`, no reversal reapplied. Source
`control_allocator/ControlAllocator.cpp:745–757` fills/publishes normalized
actuator_servos; downstream `mixer_module.cpp:542` applies the output reverse
mask. The unchanged `verify_flap_slow_configuration()` checks this aircraft's
CA_AIRFRAME=1, compatible method 0/2, three surfaces, left roll -.55/pitch1,
right roll+.55/pitch1, rudder yaw1, zero allocator trims, MAIN1=201, MAIN2=202,
MAIN5=203, PWM_MAIN_REV=17. This is not a generic-airframe mapping.

`envelope=max(abs(current pitch), previous envelope*exp(-dt/1s))`.
Fast attack, causal slow release, no future percentile. Envelope is held during
bad samples and cleared on reset. Stale (>200 ms/zero timestamp), nonfinite,
mapping, future timestamp and range violations have distinct bit flags.
Invalid samples stop admission and request virtual safety release using a
conservative pitch context of 1; valid Roll bias evidence is not discarded.
Mapping/config mismatch instead resets all shadow state.

## 7. Virtual-I bookkeeping

Initialize b=offset=0. Each cycle uses `I_shadow=T_actual_current+offset`.
Only accepted core DeltaI updates the virtual offset. Actual I/S are never
written. Normal mismatch tolerance, declared before execution, is 2e-6 torque:
`g_cycle*accepted_delta_i_virtual+1.1*accepted_delta_b`.
`B_shadow_total=g_cycle*I_shadow/1.1+b_shadow` is logged. No permanent equality
to B_obs across changing g/natural T is required.

Virtual I must be in +/-current FW_RR_IMAX. If the actual I+S baseline or later
natural motion makes the virtual state inadmissible, shadow resets and reports
invalid; it does not silently clamp actual burden or widen IMAX. Finite actual
context is retained in reset diagnostics; nonfinite inputs get invalid-marked
zero placeholders. This conservative choice can prevent useful shadow episodes
when current V3's released headroom permits T>IMAX. It is a documented limitation,
not evidence that the future architecture cannot work.

## 8. Reversal and reset

Opposite B_hat with trusted full gate latches unwind; core stops exactly at zero.
The accepted zero transaction clears estimator history and coordinator latch,
then observes the pure-core false-gate rearm handshake. Sampling restarts on
the next cycle; a complete fresh window is required before opposite growth.
Prior unwind may continue through a maneuver, never through zero in that maneuver.

Enable rising, disable, disarm, landed, epoch mismatch, invalid configuration,
invalid g/burden/capacity and time discontinuity clear virtual state. Any PX4
parameter refresh conservatively reconfigures/reset shadow, including unrelated
parameter updates. No actual handback is needed. Safety release preserves the
pure-core exception: if virtual I capacity prevents exact handback, finite
unmatched torque is explicitly logged; do not call that transaction conserved.

## 9. Integration point and memory ownership

The new hook is after actual torque publication. Its g comes from
`_b2b_g_current`, never post-update `_gain_compression.getGains()`. Its sole
publication is `flap_tail_trim_shadow`. Owned ring/core/state and log buffer are
module members; no large ring temporary or growing vector exists. Core config
replacement uses placement construction into the existing owned object, not heap
allocation. Diagnostic uORB topic is advertised at module construction.

FLAP_TTR_SHDW=0 skips computation/logging after clearing any previous episode;
a falling edge publishes one reset record. No active enable exists.

## 10. SHADOW_OUTPUT_DATAFLOW_AUDIT / ACTUAL_CONTROL_PATH_DIFF_AUDIT

| Output | Only consumers | Actual mutation permitted? |
|---|---|---|
| B_hat/target | owned core input and log | No |
| accepted DeltaI | owned virtual offset and log | No |
| b_shadow/offset/I_shadow | virtual diagnostics and log | No |
| pitch envelope/reserves | core admission and log | No |
| actual_tail_trim_torque | log literal 0 | No |

`tests/audit_tail_trim_shadow_isolation.py` removes only the enumerated new
configuration hook, trailing shadow call, topic advertisement and shadow method;
the remaining entire FixedwingRateControl.cpp is byte-identical to baseline.
It also verifies no changes to RateControl, B2B, allocator or existing Fast/Slow
shadow module. This retains the allocator feedback fix, scheduled trim, existing
S and all native-I mutation behavior exactly.

Closest-possible behavior test compiles the CURRENT production output block
via CMake extraction, runs real RateControl/B2B with Slow OFF and nonzero Slow S,
then invokes Shadow OFF/ON. All three actual torque axes, actual I and S match
exactly while b_shadow grows. Gain/parameter cache and uORB delivery are mocked;
this is not a complete scheduler-level/aircraft differential test.

ROLL_OUTPUT_MATH_CHANGED=NO; ACTUAL_I_MUTATION_ADDED=NO;
TAIL_TRIM_TORQUE_ADDED=NO; ALLOCATOR_INPUT_CHANGED=NO.
SHADOW_ACTUATION_EQUIVALENCE=PASS at identical-input arithmetic/state level.
Servo physics/runtime bitwise equivalence is not claimed; there is no new
actuator publisher or data connection, and hardware timing remains to be tested.

## 11. Parameter table (TAIL_TRIM_PARAMETER_COUNT=5)

| Name | Unit | Default | Range | Why needed | Evidence |
|---|---|---:|---|---|---|
| FLAP_TTR_SHDW | boolean | 0 | 0/1 | Explicit diagnostics opt-in | VALIDATED_SEMANTICS, shadow only |
| FLAP_TTR_BMAX | normalized tail | .08 | 0–.10 | Bound physical candidate | PROVISIONAL_TUNING, shadow only |
| FLAP_TTR_BRSV | normalized tail | .03 | 0–1 | Leave residual burden instead of all-trim target | PROVISIONAL_TUNING, shadow only |
| FLAP_TTR_BSLW | normalized tail/s | .005 | .001–.1 | Physical movement slew distinct from raw-I slew | PROVISIONAL_TUNING, shadow only |
| FLAP_TTR_RRSV | normalized tail | .15 | .01–1 | Minimum each-direction dynamic reserve | PROVISIONAL_TUNING, shadow only |

Target is sign(B_hat)*min(BMAX,max(abs(B_hat)-BRSV,0)). The reserve is a
provisional physical working-point choice, not an optimal value. Maneuver
hysteresis/dwell, pitch tau/freshness and g minimum are internal provisional
constants, not another set of runtime tuning knobs. g minimum .01 is only the
existing diagnostic floor used in core tests; no flight logs were retuned or
used to certify it. GMIN_DEPLOYMENT_VALIDATED=NO. No existing V3 parameter
definition/default/value was changed.

## 12. Logging schema and ULog verification

New `msg/FlapTailTrimShadow.msg`: controller timestamp sample and reset epoch;
I/S/T/g; Bobs/Bhat/target/b; virtual offset/I/total; requested/accepted b and
accepted virtual I/mismatch; raw/envelope pitch; positive/negative/symmetric
reserve; detector std/sign/count; validity/maneuver/gate/reversal/safety/reset;
limiter flags; pitch error bits; compute_us; literal-zero actual torque.
20 Hz maximum held-state publishing (250 Hz controller yields 52 ms spacing).
No interpolation. Logger default profile uses `add_topic(...,50)` so enabling
after logger startup is recorded. Custom logger_topics.txt overrides the default
profile and must include this topic explicitly.

Actual ULog test found optional registration excluded the initially silent topic.
It was corrected before completion, and late-enable recording was rerun.
Final ULog: `/tmp/tail-trim-sitl.7PD5td/log/2026-09-17/15_59_34.ulg`.
pyulog assertions, exit 0: 338 samples (337 enabled, one disabled reset), median
52,000 us, all float fields finite, maximum absolute actual trim torque exactly
0, final enabled/b/offset all zero. `pitch_invalid=4` correctly identifies the
generic SIH plane's mapping mismatch. Logging supports diagnosis, not a claim
of valid learning on that generic airframe.

## 13. RED -> GREEN evidence

Buildable empty helper scaffolding ran 21 tests: 19 failed behavioral assertions,
2 passed (empty-state clearing/basic object isolation). Exit 1. An earlier unused
private-field compiler warning was corrected before this run; compilation errors
are not counted as RED. Real implementation passed all 21, exit 0, without
widening tolerances. Integration/edge/resource coverage subsequently expanded to
28 tests, all passing. No placeholder helper remains in production.

## 14. New test coverage

| Suite | Count | Coverage |
|---|---:|---|
| TailTrimEstimator | 6 | LPF, full/fresh gate, invalid/maneuver, std/sign, 10 s capacity, causal prefixes/multiple rates |
| TailTrimManeuver | 3 | phi and p entry, hysteresis/no chatter/dwell, invalid recovery |
| TailTrimPitch | 2 | Geometry/attack/decay/reset, all distinct invalid reasons including future |
| TailTrimShadow | 15 | Observation/virtual handoff/conservation, natural T, freeze, pitch separation, safety, zero/fresh reversal, invalid g/capacity/time/config, disable/re-enable, real objects, prior unwind, coordinate equivalence |
| TailTrimIsolation | 1 | Extracted production torque exact OFF/ON with actual I/S invariants |
| TailTrimResources | 1 | Fixed storage and host timing (not hardware latency certification) |

## 15. Regression commands and exit codes

All test-target builds use the existing `build/px4_sitl_test` infrastructure;
production build verification is separate.

| Command/target | Result | Exit |
|---|---|---:|
| cmake --build build/px4_sitl_test --target unit-AdaptiveTailTrimShadow | 28/28 | 0 |
| unit-AdaptiveTailTrimCore | 30/30 | 0 |
| unit-BumplessRollITransfer with B2B_ADAPTIVE_GOLDEN_DIR set | 72/72, no skipped golden export | 0 |
| unit-rate_control_test | 17/17 | 0 |
| functional-AllocatorSaturationFeedback | 5/5 | 0 |
| unit-AdaptiveTailTrimContract --gtest_filter=AdaptiveTailTrimContract.* | 12 PASS, expected RED01–04 | 1 expected |
| python3 src/modules/fw_rate_control/tests/audit_tail_trim_shadow_isolation.py | Exact baseline/dataflow audit PASS | 0 |
| git diff --check | PASS | 0 |

Raw-V3 physical-semantics RED01–04 remain RED; no claim that current S is physical
b. Functional uORB runner's pre-existing work-queue warnings remain, synchronous
tests passed. No test_uart_send.c workaround or unrelated source fix was made.
Full local command logs: `/tmp/tail-trim-shadow.VH7lAu/` (temporary evidence).

## 16. Production SITL and runtime

`make px4_sitl_default`: final exit 0. Existing SIH fixed-wing startup (model
`sihsim_airplane`, instance17, isolated /tmp workdir) ran with no new Gazebo setup.
Default SHDW=0 confirmed, then set1 and checked topic, then set0 and checked reset.
No arming/takeoff was commanded. Server exited 0 after shutdown; shutdown client
returns 255 when its server closes, which is not a startup/build failure.

Final server log `smoke-final-server.log`; snapshots `final-smoke-default/on/off.log`;
ULog check `final-ulog-check.log`. An initial restart using a relative rootfs with
-w failed to locate rcS (exit255); the absolute-path invocation succeeded.
This is a disarmed generic-airframe smoke, not flight-learning evidence.
Full deterministic dynamics OFF/ON comparison NOT_RUN; extracted identical-input
production arithmetic is the primary equivalence evidence.

## 17. FMUv6C production build and pure-core portability

`make px4_fmu-v6c_default`: final exit 0. Initial attempt exit 2 because previously
unlinked pure-core included unavailable NuttX `<limits>`. The only core edits are
semantically equivalent embedded compatibility substitutions: `<float.h>`/FLT_MAX
for numeric_limits<float>::max(), and float `nextafterf` for std::nextafter(float,float).
No algorithm/config/contracts changed; 30/30 core tests rerun successfully.

Tested firmware in `build/px4_fmu-v6c_default/`:

| Artifact | SHA256 |
|---|---|
| px4_fmu-v6c_default.px4 | 5639ca257c9036132872b09474178595713b01f7eca1e70a15c908c066107142 |
| px4_fmu-v6c_default.elf | 96549e7aa5695084c683fe43ee76d58d6994ad63f85f0a17159622d42b2dce23 |

Embedded source commit string is `8f748d1d5551b7f6b0f5a6d8b73d515150656e0b`.
The later test/report commits do not change firmware source. Build artifacts are
local/ignored, not committed or uploaded. No flash performed.

## 18. Resources and stack

| Resource | Baseline saved ELF | Shadow | Delta |
|---|---:|---:|---:|
| FLASH linker usage | 1,847,868 B | 1,855,420 B / 1,966,080 B (94.37%) | +7,552 B |
| AXI static RAM | 91,212 B | 91,212 B / 524,288 B (17.40%) | 0 |
| .data | 4,080 B | 4,080 B | 0 |
| .bss | 87,116 B | 87,116 B | 0 |

Other reported RAM regions remain zero. Baseline ELF SHA256
`97039e8a7a64c3f53725c950b6e0f9eeb715cdc2773e3735eea6bf89d3047bed` was measured
before the build in the same unchanged target/configuration; usage matches the
previous pure-core report. These link deltas include metadata as well as code.
Static RAM does NOT account for increased module object storage or uORB buffers.
Host sizeof Shadow=944 B, detector=836 B. Member storage is part of the existing
module allocation; normal PX4 object/uORB infrastructure allocation still exists.
SHADOW_HEAP_ALLOCATION=NO means no separate/in-loop estimator/core heap allocation,
not that all PX4 messaging is allocation-free.

ARM compiler -fstack-usage audit with production flags (exit0): actual Run body
864 B plus wrapper8; new hook352 B; coordinator336 B (core inlined); detector64 B;
pitch56 B. Simple local frame chain ~1,624 B before library/work-queue/interrupt
overhead, versus configured 2,240 B work-queue stack. Log buffer moved to member
storage reduced hook from488 to352. No frame-size build warning. This is not a
measured hardware high-water proof: stack bench check remains mandatory before
flight authorization. No stack-size parameter was changed.

Fixed worst-case detector scan: <=200 entries once each 50 ms; no heap/sort in
production. Host ASan test (512 timed calls, full10s ring): median250 ns, P95 875 ns,
max1000 ns, illustrative only. SITL compute_us is zero under lockstep simulated
time, so it is NOT evidence of zero CPU cost. Cortex-M7 runtime timing and logger
write overhead remain unmeasured. Generated topic payload is 132 B (136 B with
struct padding), ~2.64 kB/s at20 Hz plus ULog framing; compute_us excludes publish overhead.

## 19. Limitations

No aerodynamic identification, optimization, tracking/path/authority benefit or
active closed-loop safety is established. SIH smoke used invalid mapping and
disarmed reset, not real learning; unit-level evolving state provides software
coverage only. Virtual baseline over IMAX conservatively resets; published
20Hz instantaneous transaction deltas are held snapshots, not a complete ledger
of every controller tick. State/offset traces remain available, but every single
handoff cannot be reconstructed from downsampled deltas alone. Custom logger
profiles require explicit topic inclusion. Diagnostic resets have valid=false;
finite actual context is retained but no inferred trim claim is made for them.

## 20. Provisional tuning and frozen scientific boundaries

BMAX/BRSV/BSLW/RRSV, maneuver thresholds/dwell, pitch tau/freshness and GMin remain
provisional. No performance fitting, old/new-wing retraining or future-log access
was performed. Existing V3 and Fast definitions/coefficients/parameters/messages
are unchanged. Fast actual remains unmodified and unauthorized.

TAIL_TRIM_CORE_INTERPRETATION=AUTOMATIC_ROLL_TRIM
SECOND_SLOW_CONTROLLER=NO
PITCH_USED_FOR_BIAS_ESTIMATION=NO
PITCH_USED_FOR_ACTUATOR_SAFETY=YES
CURRENT_V3_TOTAL_BURDEN_USED_IN_SHADOW=YES
SHADOW_T_ACTUAL_EPOCH_CONSISTENT=YES
RC_TRIM_SIGNAL_MODIFIED=NO
ROLL_SETPOINT_BIASED=NO
TAIL_TRIM_PARAMETER_COUNT=5

## 21. Future flight-shadow protocol (design only)

After separate authorization and bench checks, compare A: current V3 actual,
Shadow OFF; B: same current V3 actual, Shadow ON. Record commit/build/artifact
hashes, fixed parameters and verified geometry. Include straight/low maneuver,
sustained bank, transitions, natural reversal if observed, pitch demand changes,
and safe disarm/reset. Do not intentionally create unsafe invalid data in flight.

ULog analysis should assess sign/magnitude of Bobs/Bhat/target/b; gate sample
count/stability and learning only in straight segments; maneuver freeze; pitch
envelope limiter frequency; Rpos/Rneg; virtual I bounds; actual S versus b;
fresh reversal window; normal mismatch and safety exceptions; actual torque
field always zero; logging completeness, real compute latency and stack high-water.
Current V3 remains the actual controller in both runs. A successful shadow cohort
supports causal prospective diagnostics only, not closed-loop superiority.

## 22. Blockers before any active integration

A. Real-flight prospective shadow evidence.
B. Maneuver threshold validation.
C. Pitch envelope tuning/latency study.
D. Pre-allocator clipping feedback design.
E. Atomic actual I/b handoff, including capacity/recovery.
F. Actual physical torque insertion ordering.
G. Scheduled trim interaction.
H. Closed-loop SITL dynamics/saturation validation.
I. Hardware bench, stack high-water, timing and log bandwidth.
J. Flight authorization review.

SHADOW_INTEGRATION_PHASE=PASS (software shadow gates only).
ACTIVE_TAIL_TRIM_AUTHORIZED=NO; FAST_ACTUAL_AUTHORIZED=NO;
REAL_FLIGHT_AUTHORIZED=NO; PUSHED=NO. Stop at human review.
