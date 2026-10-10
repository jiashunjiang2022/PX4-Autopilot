# Pitch FF Phase Shaper V1 audit

## Provenance and delivery scope

Frozen base branch: `roll-tbr-v2-pitch-diagnostic-20261010`.
Exact base: `49d49b63c62c9eb0a8df018337f50995aec75455`.
Base worktree: `/Users/jiangjiashun/PX4/PX4-roll-tbr-v2-pitch-diag-20261010`.
The base branch, SHA and clean status were verified before implementation and again before commit.
Child branch: `pitch-ff-phase-shaper-v1-20261010`.
Child worktree: `/Users/jiangjiashun/PX4/PX4-pitch-ff-phase-shaper-v1-20261010`.

The final commit SHA is `git rev-parse HEAD` in this worktree. The exact SHA,
postcommit firmware SHA256, build results, resource sizes, reference hashes,
diff inventory and clean-status evidence are recorded in
`build/pff-verification/FINAL_PROVENANCE.json` and `FINAL_SUMMARY.txt`.
These external evidence files are deliberately outside the commit: embedding a
commit's own SHA or its postcommit firmware hash into that commit would require
a new commit and invalidate the identity. This audit belongs to the implementation
commit; its sidecar binds that commit to the subsequently built package.

No push, PR, hardware connection, flash, bench run, arming or flight was performed.
The source/build qualification ends at readiness for future short bench validation.

## Scientific purpose and authoritative dependency update

This method does not replace the Native Pitch controller. It retains Native
q/q_sp/P/I/D feedback and only subtracts an identified, causal, phase-locked 1f
component from the command-equivalent Native Pitch FF contribution.

The user's final dependency audit overrides the earlier two-state/1f-only estimator
requirement. The production estimator contains internal 1f **and** 2f nuisance
regression. Applied shaping is exclusively 1f. Removing internal 2f materially
changes the winning 1f estimate (reported removal RMSE 0.001844133431712545,
maximum 0.008023033458382801). No alpha2 parameter, selectable 2f shaping or applied
2f correction exists. This remains 1F-TARGETED PITCH FF SHAPING with 2F INTERNAL
NUISANCE REGRESSION.

Authority directory: `/Users/jiangjiashun/Documents/pitch_pff_handoff_20261010/`.

| Authority | SHA256 |
| --- | --- |
| winning_nlms_reference.py | ad4324fe074237d2b435576e4ec4b095209c0b38ce8629336faed826311be67d |
| PITCH_PHASE_FF_SHAPER_IMPLEMENTATION_SPEC.md | ca63062d58db7558611c5f6472ec2c652d8126aba81683b06f8cf90970640553 |

Winning configuration: `NLMS|CYCLES|1|W1|A1|B0`, binary64.
Phase is the existing publisher's increasing `[0,2*pi)` signal, equivalent to
`mod(atan2(phase_sin,phase_cos),2*pi)`; no new phase/frequency estimator is added.

## Exact estimator equations and update order

The 11 authoritative states are a1, b1, a2, b2, baseline, previous_phase,
warm_cycles, fade, correction1, correction2, was_valid. Reset sets numeric states
to zero except previous_phase=NaN, and was_valid=false. C++ uses double throughout
this estimator to preserve binary64 semantics, including normalization epsilon.

For a valid sample, increment warm_cycles before adaptation when
`was_valid && finite(previous_phase) && phi < previous_phase - pi`.
Then set previous_phase=phi and was_valid=true. The exact authority equations are:

```text
tau = max(A / f, dt)
c1 = cos(phi)       s1 = sin(phi)
c2 = cos(2*phi)     s2 = sin(2*phi)
beta0 = 1 - exp(-dt / max(0.5, 2/f))
baseline <- baseline + beta0 * (q_sp - baseline)
estimate = a1*c1 + b1*s1 + a2*c2 + b2*s2
error = q_sp - baseline - estimate
mu = min(0.25, 2*dt/tau)
leak = dt / max(50*tau, 1e-6)
normalization_denominator = 2.0 + 1e-9
a1 <- (1-leak)*a1 + mu*error*c1/(2.0 + 1e-9)
b1 <- (1-leak)*b1 + mu*error*s1/(2.0 + 1e-9)
a2 <- (1-leak)*a2 + mu*error*c2/(2.0 + 1e-9)
b2 <- (1-leak)*b2 + mu*error*s2/(2.0 + 1e-9)
```

The baseline is already updated when forming error; estimate uses the **unfaded,
pre-update** four-coefficient prediction. All four updates use the same error.
Basis squared norm is 2. The authority performs no coefficient clipping.
At warm_cycles>=1, fade becomes `min(1,fade+dt*f/max(1.0,1e-6))`; otherwise fade=0.
The newly updated coefficients produce `correction1=fade*(a1*c1+b1*s1)` and
`correction2=fade*(a2*c2+b2*s2)`.

For valid production samples the control/status naming is:

```text
qhat_1f = a1*cos(phi) + b1*sin(phi)              # unfaded
qhat_2f_internal = a2*cos(2phi) + b2*sin(2phi)  # internal only
alpha_effective = alpha_command * fade
qhat_1f_applied = alpha_effective * qhat_1f
ff_delta = -Kff_effective * qhat_1f_applied
delta_u_pitch = compression_pitch * airspeed_scaling_sq * ff_delta
u_pitch_shaped = u_pitch_native + delta_u_pitch
```

The `qhat_1f_reference` and `qhat_2f_reference_internal` status aliases are the
reference's faded correction1/correction2. This naming resolves the handoff's
faded-qhat convention against the user's explicit applied=alpha_effective*qhat1
requirement without applying fade twice. Replay compares the reference aliases
and reference alpha*fade, as well as the actual applied correction and states.

## Safety wrappers, dropout and recovery

Production activation requires EN=1, armed, fixed-wing, non-VTOL, non-tailsitter,
not transitioning, AUTO_MISSION, !landed, both rate and auto controllers enabled,
valid/fresh phase, finite inputs/state/config, f>0.5 Hz, valid dt, completed warmup.
Freshness uses the actual WingPhase timestamp: nonzero, not in the future,
age<=50000 us inclusive. It is a hard upper bound, not a normal latency target.
No stale-phase extrapolation occurs.

Raw measured dt is captured before Native dt clamping. First/missing/backward
intervals, dt<=0, dt>0.1 s or nonfinite raw intervals reject PFF; valid samples
use the actual Native controller dt. The Native clamp and scheduling are unchanged.

Disabled/disarmed/not Mission/landed/unsupported/controller-inactive conditions,
nonfinite numeric input, invalid config/dt/frequency/state and output-mapping
overflow immediately reset the estimator and produce zero **correction**, leaving
Native control and Native FF intact. Numerical faults take precedence over a
simultaneous phase dropout. Mapping also checks float representability and trim.
Default-off and exactly-zero-correction paths skip the Pitch output addition/cast.

Ordinary phase invalid/stale dropout reproduces authority:
`step=0.8*dt`; both corrections move toward zero by at most step; adaptation states
reset while retaining the just-slewed correction residuals. No internal2f residual
is applied. During dropout qhat_1f is the residual and alpha_effective=alpha_command,
so applied=alpha_effective*qhat_1f remains exact. The reference's diagnostic
alpha*fade is zero during dropout; it is not the production effective multiplier
for this residual. active=false while a dropout residual may still be fading.
Recovery starts zero coefficients, repeats the full one-cycle warmup and one-cycle
fade-in. Recovery clears any retained residual on the next valid update, exactly
as the authority does; no claim of additional recovery blending is made.

An internal numerical containment wrapper uses
`4*radians(max(FW_P_RMAX_POS,FW_P_RMAX_NEG))`. Both harmonic coefficient amplitudes
and baseline must remain finite and within this bound, otherwise immediate
fail-native reset occurs. This is a generous sanity bound, not coefficient
clipping or an aerodynamic tuning parameter. The authoritative reference has no
such bound. Reference trajectories never reach it; abnormal excitation is tested.
No fourth parameter and no modification to Native setpoint limits is introduced.

## Production insertion and isolation

The effective FF gain is the exact current `scaled_gain_ff(1)` used by RateControl.
Compression is captured from `_gain_compression.getGains()(1)` **before** its update;
airspeed_scaling_sq uses current `_airspeed_scaling` twice. These are production
cycle variables, not reconstruction from diagnostic topics.

Native RateControl.update and its complete P+I_before+D+FF sum remain unchanged.
The existing Native control_u is computed first. `_gain_compression.update(control_u,dt)`
receives that original Native vector. Only afterward PFF writes `control_u(1)`.
Existing trim, constrain and torque publication handle the shaped command.
No separate allocator or output pipeline is added. Roll/Yaw components are untouched.

Before old diagnostics and Roll TBR validity evaluation, the local Pitch scratch
control_u is restored to its captured Native value **after** final torque output
and new status publication. Old pitch_rate_control_sum, pitch_control_after_scaling
and pitch_output_preclip retain their Native provenance. The existing old
pitch_output_final continues observing the actual final published command, now
potentially shaped; its preclip/clip fields remain Native. New status provides
explicit Native versus shaped quantities. Roll validity input remains Native.

Removing all complete PITCH_FF_SHAPER marker blocks yields byte-for-byte exact BASE
FixedwingRateControl.cpp and .hpp. Only the new topic registration and logger
registration lines are added outside these controllers. RateControl, PitchController,
RollTbr helper/params, old message schemas/tools, attitude outer loop, allocator,
board/platform configuration, wing phase publisher, CRSF/PWM/AS5600/Hall/RPM/EKF,
airspeed, navigator, gains, FW_PSP_OFF and ROMFS are unchanged.

The verifier enforces changed-path allowlist, exact parent, marker parity, PFF-only
assignments, no native controller mutation, production-variable gates/provenance,
insert/restore order, four-basis denominator, applied1f equation and no alpha2/2f
application. Together with paired real RateControl tests, this guards against
q_sp, measured q, integrator and gain-compression feedback contamination.

## Parameters and diagnostic topic

| Parameter | Type | Default | Range | Meaning |
| --- | --- | --- | --- | --- |
| FLAP_PFF_EN | int32 | 0 | 0..1 | Mission-gated enable |
| FLAP_PFF_ALPHA | float | 0.5 | 0..1 | Applied 1f fraction |
| FLAP_PFF_ADAPT | float | 1.0 | 1..8 | Adaptation wing cycles |

The actual authority search tests CYCLES 1,2,4,8. Its SECONDS .25,.5,1,2 are a
different configuration; they do not determine the cycles parameter range.
A failing boundary test first exposed the missing 8-cycle range; implementation
then expanded the range to 1..8. Existing generated parameter records are compared
in full against the exact BASE package; exactly these three are added, with no
existing defaults or metadata changed.

PitchFfShaperStatus has 51 semantic fields. Timestamp/sample/setpoint/phase origins
and update_counter support synchronization. controller_dt/measured_dt, phase age,
phase/frequency, gates and reset_reason show qualification. Explicitly named 1f
and internal2f coefficients, baseline, fade/warmup, unfaded estimates, reference
aliases and applied1f expose identification. Native FF, delta, shaped FF, compression,
scaling, correction, Native/shaped preclip and shaped final establish the whole
command evidence chain. Values are float32 diagnostics of binary64 helper states.
The reset enum is defined in PitchFfPhaseShaper.hpp with NONE=0.

Logger interval is 10 ms; 100 Hz is a target maximum, not guaranteed actual ULog
sample rate. Existing Roll, Pitch and wing-phase logger settings are unchanged.
Target ARM sizeof(helper)=88 bytes; sizeof(generated status)=192 bytes including
7 padding bytes. Semantic payload excluding padding=185 bytes. ESTIMATED raw
payload at 100 Hz is 18500 bytes/s (19200 bytes/s using padded in-memory records).
Neither figure includes ULog headers/metadata/dropouts and neither is measured
bandwidth. The helper has O(1) state; no per-cycle allocation, prints, file I/O,
blocking, mutex, history growth, FFT or extra scheduler operation is introduced.
CPU time and total heap/topic/logger allocations have not been measured on hardware;
static linker RAM delta alone does not describe runtime memory cost.

## TDD and verification evidence

RED first was observed for absent production helper, absent integrated status,
nonfinite output-mapping acceptance, and the missing 8-cycle config boundary;
production changes followed those failures. Final helper tests cover default-off,
alpha0, pure1f/task-frequency mixture/frequency sweep, ordinary dropout/stale,
freshness boundary/future/zero timestamps, warmup/recovery, NaN/Inf, invalid dt,
gates/config, amplitude containment, state corruption, reset determinism and
alpha scaling. Independent Python-winner reconstruction bounds are fixed in tests.

The output-chain test uses the actual unchanged RateControl for paired 2000-sample
runs at three airspeed scales, checks integrator/native sum equality, FF mapping,
Pitch-only modification, trim/clipping, restoration, default-off and mapping faults.

Commands (each must exit zero):

```sh
git diff --check
sh Tools/pitch_ff_phase_shaper/run_pitch_ff_phase_shaper_host_tests.sh
python3 -B Tools/pitch_ff_phase_shaper/replay_pitch_ff_phase_shaper_reference.py --json build/pff-verification/replay.json
python3 -B Tools/pitch_ff_phase_shaper/verify_pitch_ff_phase_shaper_source.py --source-only
sh Tools/roll_tbr_validation/run_roll_tbr_host_tests.sh
sh Tools/pitch_native_diagnostic/run_pitch_diag_host_tests.sh
python3 -B Tools/pitch_ff_phase_shaper/verify_pitch_diag_regression.py
```

The old diagnostic source verifier remains unmodified, including its original-parent
check. A temporary detached BASE worktree receives the child's marker-stripped
Native controller/logger/message projection (each asserted exact BASE), plus the
child's frozen diagnostic files/tools. Its unchanged verifier runs there and the
projection is removed. This does not reinterpret an old verifier failure as success
or weaken its invariants. Paired new tests separately exercise the shaping path.

Replay consumes every explicit dt_s sample, including the reference first interval,
with no trajectory exclusions, and compares all 16 reference fields. Fixed tolerance
is 2e-12; measured errors are many orders smaller. Numerical wrapper differences
are tested separately; excessive-dt vector rows also agree because reference
residuals there are already zero. Host binary64 replay is not hardware execution
and does not establish bitwise identity of ARM/libm/compiler output.

| Vector | Rows | qhat RMSE | Maximum absolute error | Correlation |
| --- | --- | --- | --- | --- |
| smoke | 1550 | 1.1095522605957073e-17 | 5.551115123125783e-17 | 1 |
| production | 5700 | 1.2836745884082543e-17 | 6.938893903907228e-17 | 1 |

Production 1f cos/sin coefficient maxima are 4.163336342344337e-17 and
6.938893903907228e-17; both internal2f coefficient maxima are 3.469446951953614e-17.
Active/reset/warmup/fading parity PASS. Vector hashes:

- smoke: d397122f268d6734edd7d04edfe3e20d45647c202b62d18454a3ff316bf90eb3
- production: afd13c322cd0b33bca3a7063aff4780b37a020853aed89962f53531c02739364

Precommit complete suite PASS is recorded in `build/pff-verification/precommit-tests.log`.
The complete suite repeats after the final commit; final evidence resides in
`postcommit-tests.log`, `replay.json` and `FINAL_PROVENANCE.json`.

## FMUv6C clean builds and resources

An initial clean attempt exposed NuttX portability errors: missing C++ limits,
limited std math namespace and strict double-promotion/float-equality diagnostics.
They were fixed using FLT_MAX, C math fmax/fmin/hypot, explicit conversion and
magnitude>0 comparison. Compiler flags and board config were not relaxed.
Failed build files were retained recoverably under build/pff-verification.

After all tests passed, the entire build directory was moved aside and a fresh
`make px4_fmu-v6c_default` succeeded (1177 steps):
`build/pff-verification/first-successful-clean-build.log`.
First successful clean build FLASH=1807364 bytes, AXI_SRAM=62476 bytes.
Trusted BASE package git_hash is the exact frozen SHA, board_id=56, and its ELF
shows FLASH=1801924 bytes, AXI_SRAM=62476 bytes. First-build deltas: +5440 flash,
0 static RAM. FLASH includes initialized data image; RAM includes linker alignment.
Other linker memory regions have zero allocated bytes in these builds.

Only after the first successful clean build and generated parameter/topic/package
checks is the local implementation commit created. The entire build directory
is then moved aside again, followed by a second fresh make. The **postcommit**
package is the final artifact at `build/px4_fmu-v6c_default/px4_fmu-v6c_default.px4`.
Final resource sizes/hash are in the external provenance sidecar; no precommit
firmware hash is presented as final. Full verification checks package git_hash==HEAD,
board_id==56 and decompressed package image==generated .bin. Build logs may contain
the platform's existing RWX linker warning; compilation keeps all Werror checks.
Generated NuttX jlink-nuttx helper and PFF Python cache are moved into ignored
verification storage to keep the final source worktree and submodule clean.

## Limits and future qualification

qhat_1f is a causal phase-synchronous estimate, not measured aerodynamic force.
Pitch normalized command is not physical servo angle or aerodynamic pitching moment.
Wing phase is an exogenous synchronization signal. The method does not filter
measured q or modify Native P/I/D; it does not prove all wingbeat-scale motion is
disturbance. V1 shapes only 1f; internal2f is intentionally not shaped. Shared-tail
allocation remains downstream and unchanged. Source/build readiness does not equal
flight validation or demonstrate efficacy, margins or actuator adequacy.

Default firmware EN=0, ALPHA=0.5, ADAPT=1.0. Parameter migration may retain values;
explicitly verify FLAP_PFF_EN=0 before any future initial flash/bench qualification.
Future first Pitch qualification should use FLAP_TBR_EN=0. This task changes no
user parameter/migration/flight files and carries out no hardware action.

Required source facts: INTERNAL_2F_ESTIMATOR_PRESENT=YES, APPLIED_2F_PATH_PRESENT=NO,
ALPHA2_PARAMETER_PRESENT=NO, APPLIED_QHAT_EQUALS_ALPHA_TIMES_QHAT1F=YES.
Final READY_FOR_BENCH is conditional on the postcommit sidecar's passing tests,
second clean build and clean worktree. READY_FOR_FLASH=NO; READY_FOR_FLIGHT=NO.
