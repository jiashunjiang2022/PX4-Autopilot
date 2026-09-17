# Tail-Trim V3 learning parity pre-bench review

## Finding and scope

The d626 Shadow retained LPF/persistence/std/sign gating but omitted V3's robust
pre-entry initialization and discarded learned B_hat at every learning exclusion.
It also admitted learning outside Mission and incorrectly marked invalid-setpoint
samples valid. These are now corrected in the physical automatic-Roll-trim
shadow path only. This does **not** demonstrate better flight bias acquisition:
all nine frozen-cohort replays produced zero growth for both old and revised
Shadow, primarily because the unchanged maneuver policy leaves insufficient
continuous trusted evidence. Do not declare the learner flight-ready from the
unit/build results.

START_COMMIT=d6266c8acfff903da8c670e3b853398ee2ee635c
BRANCH=adaptive-tail-trim-v1-prebench-review-20260918
FROZEN_V3=a644ca7868f07004e45658d4ae9a7a8c880c6bb5

Both `git rev-parse <commit>:src/modules/fw_rate_control/BumplessRollITransfer.hpp`
returned `2b6d40f137bb1e6b4afce99c791ea6b23683389b` at a644 and starting HEAD.
The file remains unchanged. Pure core, RateControl, allocator, actual S, RC trim,
setpoints, Fast and all parameter definitions/defaults remain unchanged.
Initial/final dirty state outside this task is only user NuttX `tools/jlink-nuttx`;
SHA256 remains `3c5fbd59358b5efe7586c0be90ebb4cd7b376f9371e889ca36468276b1375c66`.
No clean/reset/stash/push/flash was performed.

This report supersedes the previous integration report's maneuver-reset,
eligibility and pre-entry descriptions. That report remains the historical d626
snapshot, not a description of the revised estimator.

## Code-level mechanism comparison

Source references below use `BumplessRollITransfer.hpp` (B2B, identical at a644),
`FixedwingRateControl.cpp` (FW), and `AdaptiveTailTrimShadow.cpp` (Shadow).
Old Shadow means the exact d626 source, also compiled as a mechanically renamed
test-only baseline. It is not approximated in Python.

| Mechanism | Frozen V3 actual semantics | d626 Shadow | Reviewed physical Shadow |
|---|---|---|---|
| Observation | Pre-entry native I; Hold raw T=I+S (B2B:238–239, 557) | g*(current I+S)/1.1, end-of-cycle snapshot | Same physical B observation; no raw-S relabeling |
| Eligibility | Module requires armed, airborne, non-VTOL FW, no failsafe, AUTO_MISSION and verified mapping (FW:387–391) | Rates+attitude enabled, non-VTOL FW; no nav restriction | Same control validity plus Mission for normal learning; other modes still observe/log |
| Pre-entry sampling | Disabled, not eligible, adaptive/parameters valid, 20 Hz native I (B2B:237–239,419–437) | None; normal learning can already grow outside Mission | Trusted non-Mission straight physical B samples only; no target/state movement |
| 5 s entry median | Last EWIN samples; full window required, otherwise instantaneous capped I fallback (B2B:250–257,440–463) | None; LPF initialized from first sample | Full trusted EWIN median seeds B_hat on Mission entry; otherwise retain valid B_hat or initialize from first trusted sample; never bypass fresh gate |
| Entry target/cap | Latched signed min(median,CAP); sign-opposed current I can cancel entry | Physical target minus BRSV, bounded BMAX | Unchanged; no separate raw entry cap or entry state machine copied |
| Entry slew | B2B_SLEW raw/s during TransferIn, without Hold growth gate | BSLW physical/s after full gate | Unchanged; no speed tuning |
| Hold LPF | alpha=dt/(TAU+dt), seed from entry median if valid (B2B:531–565) | Same alpha, physical B; reset on !learning | Same alpha; preserve B_hat during maneuver/mode exclusion |
| Persistence | 20 Hz full GWIN, default3s (B2B:478–493,575–585) | Same time semantics | Same; clear evidence on every exclusion and accepted reversal zero |
| Std | Population std of raw T, GSTD raw units | Population std of B, limit=g*GSTD/1.1 | Unchanged, historical g retained in each B sample |
| Sign | Fraction of raw samples agreeing with T_hat; sign crossing clears gate (B2B:515–528,566–578) | Same physical-B criterion | Unchanged |
| Growth | Entry transfer bypasses Hold gate; later Hold growth needs gate | All normal growth requires gate | Retained stricter all-growth fresh gate |
| Release | Hold release does not need gate (B2B:587–604) | Same-direction ordinary release allowed without gate when learning eligible | Retained, but not during maneuver/non-Mission exclusion |
| Reversal | Opposite filtered target can unwind without growth gate; cannot cross zero in same handoff | Establish unwind only with trusted full gate | Retained stricter trusted latch; no new maneuver reversal |
| Fresh after zero | Clear persistence, retain T_hat, next update samples (B2B:610–618) | Clear entire estimator, including B_hat | Clear only evidence; retain B_hat, preserve false-gate core rearm and full next window |
| Maneuver | No explicit phi/p exclusion; Mission turns can continue Hold learning | phi/p hysteresis, full estimator reset while excluded | Same thresholds/freeze; B_hat frozen, not zeroed; no turn evidence |
| Reset | State machine safety/disable/recovery, synchronized native reset epoch | Disarm/landed/config/enable/epoch/numeric/time reset all virtual state | Same hard resets; invalid control/setpoint/safety clears estimator; ordinary maneuver is not invalid data |

WHY_V3_LEARNS_BIAS_WELL = it has a robust pre-entry seed, a separate bounded
entry transfer, a persistent LPF of total I+S burden, sustained/sign-consistent
growth admission, permissive burden-release, and zero-first reversal with a
fresh gate. These are supported software mechanisms; they do not prove every
persistent Mission correction is structural bias. V3's lack of a turn exclusion
must not be restored merely to reproduce nonzero learning in these logs.

CURRENT_SHADOW_LOST_V3_MECHANISMS = no pre-entry median/entry acquisition path,
no preserved LPF across exclusion or reversal-zero, and no Mission-only learning
domain. Gate/std/sign and same-sign permissive release were **not** lost. The
separate fast entry path is intentionally still not copied, because it conflicts
with this shadow core's fresh-growth gate policy and would need a separate
review of entry authority. Reference behavior is not silently called exact parity.

## Four questions and minimal changes

### A. Pre-entry robust median

Losing the median loses robust initialization; it does not unconditionally slow
learning. With a constant unbiased first sample, both LPFs initialize correctly
immediately. A startup/entry outlier or short sign spike is the useful counterexample.
The tests seed an opposite startup transient, gather a complete5s trusted physical
history, then place an opposite entry spike. The median retains the positive
working-point evidence and the first update remains positive. Exact boundary
tests use 249 versus 250 ticks at 20 ms (4.98 versus 5 s), as well as 500 ticks
for 10 s, and ensure missing/untrusted history cannot seed a median.

Implementation: `Shadow:30–65` adds a200-element causal entry ring and member
scratch, both fixed capacity. `enterMission(EWIN)` seeds only with a complete
window; incomplete history preserves a valid prior LPF or falls back to the first
trusted sample. It clears growth evidence regardless. Pre-entry samples are
physical `g_at_sample*(I+S)/1.1`, not a raw median multiplied by a later gain.
Existing FLAP_B2B_EWIN is read; no new parameter or changed time constant.

PREENTRY_MEDIAN_NEEDED = YES for the identified robust-entry contract, not a
demonstrated universal speed improvement. The candidate still requires3s fresh
Mission evidence before any normal growth.

### B. Mission qualification

`FW:728` passes value-only `mission_eligible = nav_state==AUTO_MISSION`.
`Shadow:250–261` separates trusted observation from Mission normal learning.
Trusted straight non-Mission samples may enter the pre-entry buffer, but cannot
move b, chase target or perform ordinary release. Observation/logging continue.
Existing explicit safety events and a previously latched reversal may still unwind.
This restores the V3 actual learning domain without admitting normal turn learning.

### C. Maneuver B_hat policy

`clearEvidence()` clears only gate count/head/timing/std/sign; LPF remains frozen.
No maneuver sample updates B_hat or the entry buffer. On exit, the unchanged1s
hysteresis dwell and then a complete fresh GWIN are still required for growth.
The previous target and b hold throughout normal maneuver. At reversal zero,
only persistence clears; B_hat remains visible even on the zero transaction.
Invalid/nonfinite/stale control or setpoint, safety and hard reset still invalidate
evidence/LPF; missing data is not mistaken for an ordinary planned turn.

There is a tradeoff: a long turn may leave stale B_hat, and unchanged permissive
ordinary release can resume before a new full growth gate. This is explicit V3-
style LPF persistence/release semantics, not evidence of optimal reacquisition.
Further tuning or reweighting long-turn memory was not authorized or performed.

### D. valid semantics

The RED test produced valid=true after setpoint_valid=false on d626. Revised
`Shadow:303` requires transaction_valid, pitch.valid, control_valid,
setpoint_valid and !safety. Valid means a trustworthy observation/virtual-state
transaction, **not** gate maturity or flight-mode permission to learn. A valid
frozen non-Mission observation may have learning=false/gate=false. Invalid
setpoints still trigger virtual control-invalid safety release, never actual I
or torque changes.

## Time-scale audit (no parameter change)

`physical_rate = g*raw_rate/1.1`. Rates below are tail coordinate/s:

| g | V3 entry raw .02/s | V3 Hold raw .01/s | Shadow .005/s | Shadow / V3 Hold |
|---:|---:|---:|---:|---:|
| .60 | .01091 | .00545 | .00500 | .917 |
| .70 | .01273 | .00636 | .00500 | .786 |
| .80 | .01455 | .00727 | .00500 | .688 |
| 1.10 | .02000 | .01000 | .00500 | .500 |

Shadow Hold slew matches V3 at g=.55; below .55 it is faster, above .55 slower.
Frozen flights' Mission-airborne median g ranges .7106–.7654, making .005/s
approximately72–77% of V3 Hold physical rate. No log-based optimization was done.
V3 entry cap in these logs is raw .03 (physical .01909 at g=.7), distinct from
Hold cap .10 (physical .06364). Candidate BMAX .08 and reserve .03 are different
physical constraints; final state differences alone cannot rank estimator quality.

CURRENT_SHADOW_ACQUISITION_SLOWER_THAN_V3 = YES when compared from matched
Mission-start availability in the declared constant/noisy fixture; NOT universally
true over the original unrestricted lifecycle. d626 begins learning before
Mission, and that eligibility difference can reverse the elapsed-time ranking.
The missing median is not the sole cause; fresh-growth gate and slew/entry policy
matter independently. These distinctions are retained in the final summary.

## Executable synthetic comparison

`TailTrimLearningReviewTest.cpp` runs the actual unchanged V3 class with real
RateControl fixtures. B2B internal hold logic is not rewritten. Exogenous natural
burden is imposed by test-only I setter so each learner sees the same sequence;
this is not closed-loop simulation. imax=1 avoids capacity clipping as a confound.
All physical comparison outputs use g=.7: `g*S/1.1`, `g*T_hat/1.1` and
`g*target_raw/1.1`; V3 GSTD stays .025 raw and Shadow uses its equivalent .01591
physical threshold. Input noise .012 raw is .007636 physical. TAU5s/GWIN3s/
GSIGN.9/EWIN5s/CAP.03/HCAP.10/RSVD.05/raw slews.02/.01 are explicit.

Time t=0 is Mission entry after6s prehistory; fixtures run to93.98s at50Hz.
Four traces: exact d626 (unrestricted pre-Mission), exact d626 enabled at Mission
as a qualification sensitivity, revised physical candidate, and real V3 semantics.
Frozen test sources are generated from git d626, renamed only, not from the new
implementation. New candidate is the production C++ helper.

| Scenario | d626 | Revised candidate | Actual V3 reference |
|---|---|---|---|
| Constant .18 raw | gate t=-3; b=.02 at .98; final target .08 | gate3.00; b=.02 at6.98; final .08 | Hold gate4.54; b=.02 at4.68; final .0636364 |
| Noisy .18±.012 raw | same metrics, correct sign | same metrics, correct sign | same metrics, correct sign |
| .5s transient at t10 | no gate, max b=0, final0 | no gate, max b=0, final0 | no gate, max b=0, final0 |
| Turn t20–30 with opposite burden | b freezes but B_hat=0 during turn | b freezes and B_hat=.114545 throughout turn | continues learning; b declines .0636364→.00356262 in turn |
| Burden drops to .03 raw at20 | release latency .24s, final0 | .24s, final0 |1.10s, final0 |
| Sign reversal at20 | zero37.56; fresh gate3s later; final-.08 | zero37.56; fresh gate3s later; final-.08 | zero30.40; fresh gate3s later; final-.0636364 |

Mission-start d626 sensitivity: gate3.02 and b=.02 at7.00s, compared with V3
4.68s. Revised median does not improve the constant/noisy speed materially;
its benefit is the tested initialization robustness and retained B_hat across
turns. Release latency differences reflect reserves/caps as well as filtering.
Reversal transient sign-agreement fractions are not100%: they include unavoidable
positive-state unwind after the burden reverses, and are supplied in the JSON.
Candidate zero-first and the full150 ticks fresh delay have explicit assertions.
Do not interpret a reference that learns during turns as a desirable candidate.

All metrics, including first gate, b=.02 time, final target, sign correctness,
transient false learning, release latency, zero event and fresh delay:
[synthetic-summary.json](2026-09-18-tail-trim-review-evidence/synthetic-summary.json).

## Historical replay: frozen cohort only

Nine ULogs were found: 9.16 flights2,3,4 and9.17 flights7,8,9,10,11,12.
Every `ver_sw` is exactly a644 and every resolved B2B blob matches the frozen
file. No later/prospective cohort was read. Full paths, SHA256, firmware identity,
parameter values and change lists are in
[replay-inventory.json](2026-09-18-tail-trim-review-evidence/replay-inventory.json).
There were no changes to the audited B2B/IMAX parameters during these logs.

Export uses a20ms grid and `searchsorted(..., side=right)-1` only. No interpolation,
future samples, centered filtering or fitting. It uses logged g, actual I and S,
reset epoch, control/status/setpoint states, actual pre-reversal servo0/1 and the
logged exact airframe-verifier result. The same causal rows feed the compiled old
and revised C++ helpers; historical actual V3 physical S is read from the log.
Timestamp freshness is checked (setpoint/servo200ms, control/status1s, land1.5s,
rate-status50ms). Mission=3 and Stabilized=15 follow VehicleStatus.msg; Mission→
Stabilized triggers the existing safety event. Parameter changes, if present,
would be applied causally. Both replayers are enabled for observation; actual
I/S/g trajectory is not recomputed as if this counterfactual trim had flown.

This is a counterfactual estimator/state replay, not scheduler-exact parity or a
prediction of closed-loop flight outcome. Common trusted comparison rows require
airborne Mission, valid controls/setpoints/mapping/pitch, no safety, and no
maneuver in either helper. N/A is reported if no such rows exist; no turn burden
is substituted to manufacture a bias-learning score.

| Flight | Trusted rows | Max contiguous trusted seconds | Median B_obs | Median actual g*S/1.1 | Median candidate b | Reset episodes (rows) | Maneuver freeze fraction |
|---|---:|---:|---:|---:|---:|---|---:|
| 9.16/2 |0|0|N/A|N/A|N/A|2 (2638)|1.0|
| 9.16/3 |80|1.60|-.059824|-.02258665|0|3 (3055)|1.0|
| 9.16/4 |0|0|N/A|N/A|N/A|7 (2251)|1.0|
| 9.17/7 |0|0|N/A|N/A|N/A|4 (2898)|1.0|
| 9.17/8 |0|0|N/A|N/A|N/A|2 (2196)|1.0|
| 9.17/9 |0|0|N/A|N/A|N/A|2 (2401)|1.0|
| 9.17/10 |66|1.32|-.115383|-.0657712|0|3 (2548)|1.0|
| 9.17/11 |41|.82|-.101421|-.0308762|0|5 (4242)|1.0|
| 9.17/12 |0|0|N/A|N/A|N/A|2 (2536)|1.0|

Old and revised metrics are identical here: first_gate_time=N/A for every flight,
time_to_50pct=N/A, sign_agreement=N/A because b remains zero (sign coverage0 where
trusted rows exist). Both helpers' maximum |b| is0 over the whole replay and neither
ever obtains a full gate. Reset counts include disarmed/landed periods; they are
episodes, with reset rows separately counted, not invented fault events.

`time_to_50pct` is elapsed since first trusted sample to half the magnitude of the
median trusted physical target min(.08,max(abs(median Bobs)-.03,0)), with matching
sign. Zero target or missing support yields N/A. Sign agreement counts nonzero b
only and reports coverage separately. Freeze fraction counts unchanged b during
non-reset airborne maneuver rows, so a fraction1 with b=0 is vacuous—not evidence
of successful learned-trim retention on these flights.

The principal exclusion is the unchanged maneuver hysteresis/dwell: e.g. 9.16/2
has5022/5022 Mission-airborne rows marked maneuver; 9.16/3 has3600/3680. No mapping
failures and essentially no setpoint/control freshness failures explain this
result. Pitch/state-reset can exclude remaining small segments. Full per-flight
counts and g quantiles:
[replay-exclusions.json](2026-09-18-tail-trim-review-evidence/replay-exclusions.json).
Every requested per-flight metric for BOTH learners:
[replay-summary.json](2026-09-18-tail-trim-review-evidence/replay-summary.json).

Do not loosen maneuver thresholds, remove dwell or restore turn learning to
produce a nonzero result. This is an unresolved acquisition-opportunity limitation
for a future threshold/domain review, not evidence that physical coordinates or
the median are ineffective. V3-style physical learner is better supported for
the repaired software semantics only; historical superiority is NOT_SUPPORTED.

## RED -> GREEN and regression verification

Before changing production behavior, buildable tests ran: initial2 RED (B_hat
retention, setpoint valid), then4 RED including outside-Mission growth and robust
pre-entry contract; the synthetic fixture itself ran successfully. The new
mission field was first an unused value-only input; no initial behavior fix was
hidden in the RED run. Red logs are retained in the local evidence directory.
After minimal changes, the9 review tests and28 existing Shadow tests pass.
Additional tests exercise full/incomplete5s and10s histories, untrusted-gap
clearing, pitch independence, Mission re-entry, unchanged real I/S, unchanged
torque composition, zero-first/fresh reversal, and transient rejection.

| Validation | Result | Exit code |
|---|---|---:|
| New TailTrimLearningReview filtered suite (includes synthetic and actual historical rows) |9/9 PASS, no skipped replay|0|
| AdaptiveTailTrimShadow |28/28 PASS|0|
| AdaptiveTailTrimCore |30/30 PASS|0|
| BumplessRollITransfer (golden directory set) |72/72 PASS, no skipped export|0|
| RateControl |17/17 PASS|0|
| AllocatorSaturationFeedback |5/5 PASS|0|
| make px4_sitl_default |production build PASS|0|
| make px4_fmu-v6c_default |production build PASS|0|
| shadow isolation script |PASS|0|
| git diff --check |PASS|0|

No test tolerance was relaxed. Only the existing synthetic helper's inputs now
explicitly select Mission, as required by the new value-only qualification.
No test_uart_send.c or unrelated warning workaround. No SITL flight/runtime
claim is added by this review; previous smoke evidence applies to the prior
integration, and this task's new runtime evidence is deterministic C++ replay.

Reproduction (repo root; output directory must exist):

```sh
cmake --build build/px4_sitl_test --target unit-TailTrimLearningReview unit-AdaptiveTailTrimShadow
python3 src/modules/fw_rate_control/tests/tail_trim_learning_replay.py export /tmp/tail-trim-prebench.S5CnRe
TAIL_TRIM_REVIEW_DIR=/tmp/tail-trim-prebench.S5CnRe build/px4_sitl_test/unit-TailTrimLearningReview --gtest_filter='TailTrimLearningReview.*'
python3 src/modules/fw_rate_control/tests/tail_trim_learning_replay.py summarize /tmp/tail-trim-prebench.S5CnRe
build/px4_sitl_test/unit-AdaptiveTailTrimShadow
python3 src/modules/fw_rate_control/tests/audit_tail_trim_shadow_isolation.py
make px4_sitl_default
make px4_fmu-v6c_default
```

Raw row CSVs/build/test logs are local `/tmp/tail-trim-prebench.S5CnRe/`;
the four compact JSON evidence artifacts are committed alongside this report.
The test-only V3 fixture include also registers its72 tests, hence the review
filter is explicit; the separate B2B regression binary is run independently.

## Resource and zero-actuation audit

Fixed storage grew by two200-float arrays plus small metadata: host sizeof Shadow
2560B versus944B, estimator2444B versus836B. No new heap allocation or large
automatic scratch. Median insertion sort is bounded by200 elements and runs only
at Mission entry; it does not run every controller tick. ARM production-flag
stack-usage compilation exit0: enterMission12B, coordinator344B, hook344B;
the actual Run body remains864B plus wrapper8. Hardware high-water and worst-case
entry latency still require bench measurement; no stack setting was increased.
FMUv6C flash1,856,204B/1920KiB (94.41%), AXI static91,212B/512KiB (17.40%).
Static RAM excludes existing module heap-object growth and uORB infrastructure.

Only two existing module call-site changes: append read-only EWIN to shadow
configuration, and pass Mission eligibility into shadow input. No new parameters.
The unchanged isolation script removes enumerated shadow additions and verifies
the remainder of the entire actual controller file is byte-identical to pre-shadow
7cd690. Protected actual B2B/RateControl/allocator/Fast paths are unchanged.
No value from estimator, virtual I, b, pitch reserve or target is connected to
actuation. Actual tail-trim torque remains literal0, tested in every replay row.

PITCH_USED_FOR_BIAS_ESTIMATION=NO; PITCH_USED_FOR_ACTUATOR_SAFETY=YES.
T_actual=I+S and B_obs=g*T_actual/1.1 are unchanged; future physical coordinate
remains B_total=g*I_residual/1.1+b_R. Actual I/S never receive virtual handoffs.

## Review disposition

Semantic fixes and software verification are complete. No claim of recovered
real-flight learning ability: the frozen cohort cannot supply a full trusted
window under current thresholds. Median initialization, LPF retention and Mission
qualification have direct tests; learning-opportunity thresholds and performance
remain unresolved. A subsequent human review should decide how to validate the
maneuver domain using independent evidence, without automatically restoring
V3 turn learning or changing provisional parameters in this task.

ACTUAL_CONTROL_CHANGED=NO; ACTIVE_TAIL_TRIM=NO;
REAL_FLIGHT_AUTHORIZED=NO; PUSHED=NO. Stop at review.
