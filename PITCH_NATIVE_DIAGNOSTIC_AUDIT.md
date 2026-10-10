# Passive Native Pitch Diagnostic Audit

## Lineage and Scope

Frozen base branch: `roll-tbr-flight-validation-clean-v2-20261009`.
Exact direct parent: `690b3b0f5e0890a50ce170dc2751520c528c654d`.
Roll TBR implementation ancestor: `2b62415fec727b98f62e017f9a7c33377beeb5a6`.
Clean Native V2 ancestor: `c42789a3f0143d4be9d327f6c83c00896c0b4df7`.
Diagnostic branch: `roll-tbr-v2-pitch-diagnostic-20261010`.
Worktree: `/Users/jiangjiashun/PX4/PX4-roll-tbr-v2-pitch-diag-20261010`.

Purpose: observe pitch attitude error, outer-loop rate setpoint production,
the rate setpoint actually consumed by Native RateControl, and P/I/D/FF,
integration, limiting and normalized output clipping during Roll TBR A/C trials.
This is passive logging, with no Pitch intervention or new parameters.

Allowed changes are two messages and their CMake registration, marker-wrapped
additions in the four attitude/rate module source/header files, two default
logger entries, diagnostic tests/verifier and this document. Every controller
file stripped of `PITCH_DIAG_BEGIN/END` blocks must match the base byte for byte.
PitchController, RateControl, RollTbr, TBR parameters, allocator, sensors,
airspeed logic, handover, board configuration and the original TBR tests/verifier
remain unchanged. The original pure-TBR source verifier is not adapted or
claimed to pass on this diagnostic branch; the new verifier checks this branch.

## Topic Semantics

`pitch_outer_diag` is published by `fw_att_control` on each actual, valid
attitude-control cycle. `pitch_inner_diag` is published by `fw_rate_control`
on each actual Native RateControl update. Publishers are advertised at module
construction; diagnostic structs/snapshots are cycle-local. Counters are
independent. No new scheduler, filter, subscription update, blocking I/O,
filesystem operation or explicit allocation is added to the control cycle.

| Group | Meaning / units |
| --- | --- |
| Outer pitch setpoint, pitch, error | Actual controller-cycle Euler angles, rad; error is unwrapped setpoint minus pitch |
| Euler/body rate fields | rad/s; yaw Euler rate is captured before PitchController and before this cycle's yaw update |
| Raw body q | Diagnostic reconstruction using `cos(roll)*pitch_euler_rate + cos(pitch)*sin(roll)*yaw_euler_rate_used` |
| Limited q | Actual `_pitch_ctrl.get_body_rate_setpoint()` after the unchanged production call |
| Autotune fields | Actual added vector's pitch component, else zero; active flag identifies valid addition in any supported autotune axis state |
| After-autotune q | Actual pitch setpoint before tailsitter mapping |
| Final q | Actual `_rates_sp.pitch` published by the outer module |
| Parameter snapshots | Native units: `FW_P_TC` seconds, `FW_P_RMAX_*` deg/s; inner gains retain parameter units |
| Inner q, q setpoint, q error, q derivative | Actual Native RateControl inputs, rad/s and rad/s^2 |
| P/I/D/FF and sum | Native RateControl command frame before compression, scaling and trim |
| Output/trim fields | Normalized command proxies; not physical deflection or aerodynamic torque |
| Pitch trim | Output-side `TRIM_PITCH` plus airspeed trim scheduling; distinct from `FW_PSP_OFF` attitude/TECS trim |
| Limit/clip flags | 1e-6 tolerance for rate/I limits; output clipping is local normalized clipping, not allocator/servo saturation |
| controller_dt | Actual dt passed through the production cycle, seconds |
| Timestamps | Microseconds; outer attitude sample is the last successful copy, or zero if none |
| Mode fields | Actual rates/attitude/manual, fixed-wing/transition, Mission, landed and nav state snapshots |

Outer provenance is actual attitude setpoint and attitude sample, unchanged
PitchController call, its Euler/limited body rates, valid autotune addition,
final `_rates_sp.pitch`, and `_rates_sp.timestamp`. Inner uses actual subscribed
`_rates_sp`, `body_rates_setpoint(1)`, `rates(1)` and `angular_accel(1)`.
Join `outer.rate_sp_timestamp == inner.rate_sp_source_timestamp` and compare
`outer.q_sp_final_published == inner.q_sp_used`. These are asynchronous modules,
so timestamp equality indicates setpoint origin, not a shared CPU cycle.

The inner sum uses the integral before update:
`P*(q_sp-q) + I_before - D*q_dot + (FW_PR_FF/airspeed_scaling)*q_sp`.
The return value is captured directly; I-after is read immediately following
the production update. I-delta is after minus before. Gain compression is read
before its adaptation update. The actual scaled command, scheduled trim,
preclip sum and production normalized result are captured independently.
`reset_integral` records the actual setpoint flag, not an inferred reset cause.

## Logging and Tests

Both default logger entries use 10 ms: target maximum approximately 100 Hz,
not a promise of exactly 100.000 Hz. Publisher/controller/logger scheduling,
SD performance and ULog dropouts can reduce the observed rate. These targets
cover the <=0.625 Hz task band and approximately 3-4.5 Hz wingbeat and 2f bands.
The existing `roll_tbr_status`, including `native_pitch_output`, is unchanged.

Generated host struct sizes: outer 112 bytes, inner 152 bytes. The conservative
raw estimate using `sizeof` at 100 Hz each is 26,400 bytes/s before ULog
compression/framing; this is not a measured SD bandwidth figure.

TDD red evidence is preserved externally in
`/Users/jiangjiashun/PX4/pitch-native-diagnostic-audit-20261010/tdd_red.log`:
the source verifier failed on missing Pitch topics before implementation.
`PITCH_DIAG_TDD_RED=PASS`.

Production PitchController host tests cover time constants, both pitch-error
signs, nonzero roll/yaw contribution, asymmetric body rate limits and invalid
inputs retaining native state. Production RateControl tests close the PID/FF
sum using I-before and test I-after bounds, landed freeze, reset and effective
FF at airspeed scaling below/equal/above one. Output math tests cover scaled
sum plus trim, clipping and no clipping. Original Roll tests retain their
10,000-cycle disabled parity and 55-case frequency response checks.

Reproducible commands:

```sh
sh Tools/pitch_native_diagnostic/run_pitch_diag_host_tests.sh
python3 Tools/pitch_native_diagnostic/verify_pitch_diag_logging_only.py --source-only
sh Tools/roll_tbr_validation/run_roll_tbr_host_tests.sh
git diff --check
make px4_fmu-v6c_default
python3 Tools/pitch_native_diagnostic/verify_pitch_diag_logging_only.py
```

Host tests require the target's generated headers/configuration. The first
fresh build supplies these; subsequent tests run before the clean rebuild.
The final candidate is built again from an empty target build directory after
commit. The full verifier checks the exact parent, allowed paths, marker
parity, frozen source bytes, assignment/publication isolation, provenance,
logger registration, host tests, package identity, packaged binary and the
entire parameter JSON against the frozen baseline build.

## Firmware Identity and Build Evidence

Firmware path: `build/px4_fmu-v6c_default/px4_fmu-v6c_default.px4` (board ID 56).
Diagnostic SHA is resolved by `git rev-parse HEAD`; the verifier requires the
firmware package `git_hash` to equal it. Final diagnostic SHA, firmware SHA-256,
clean-build resource counts, deltas and verification log are recorded in
`/Users/jiangjiashun/PX4/pitch-native-diagnostic-audit-20261010/final_manifest.json`
and `final_verification.log`. These post-commit facts live outside the source
tree because embedding a commit's own SHA and firmware hash in that commit
creates a circular identity dependency. The preliminary pre-commit package
is not the candidate firmware.

Trusted baseline package source identity is
`690b3b0f5e0890a50ce170dc2751520c528c654d`, SHA-256
`532237e143121e09488e24756a67e7314a110177a184313c7d43d1d59952563e`.
Baseline ELF/map: flash 1,800,060 bytes and AXI SRAM 62,476 bytes.
These linker counts are static use; they exclude runtime heap and stack use.

Parameter JSON comparison must pass with exactly five TBR parameters:
`FLAP_TBR_EN=0`, `FLAP_TBR_FC=0.625`, `FLAP_TBR_K=0.01`,
`FLAP_TBR_MAX=0.025`, `FLAP_TBR_SLEW=0.035031594` defaults. Existing V2
preflash/final parameters remain compatible. All A/C qualification runs must
use the same diagnostic firmware: A uses EN=0, C uses EN=1. Diagnostic logging
must not become the A/C treatment variable.

## Known Limitations and Readiness

1. Raw body q is a production-formula diagnostic reconstruction, not a new
   internal PitchController state getter. Invalid native inputs may retain
   previous setpoints; nonfinite diagnostic inputs do not create control inputs.
2. 100 Hz is a logger maximum target; actual ULog frequency may be lower.
   Decimation can leave some inner source timestamps without a recorded outer
   counterpart. A missing join is not proof of a different controller input.
3. Command fields are normalized/controller proxies, not actual servo angles
   or aerodynamic torque. If any production command axis is nonfinite, the
   existing fallback sends trim and resets I; the diagnostic preclip can be
   nonfinite and should not be interpreted as ordinary clipping.
4. Outer and inner modules execute asynchronously. The timestamp key joins
   setpoint provenance, not a shared CPU cycle.
5. This is not a closed-loop Pitch intervention experiment. Logging can support
   or reject a propagation chain; correlation alone cannot establish
   aerodynamic causality.
6. Passive source parity does not remove logging CPU/heap/SD cost. No exact
   runtime CPU benchmark or real hardware logging measurement is claimed.

Source/build readiness permits a bench review only. `READY_FOR_FLASH=NO` and
`READY_FOR_FLIGHT=NO`. A future user-authorized short bench logging sanity check
must confirm both topics, reasonable finite-field ratios, timestamp joins and
normal Roll TBR behavior before qualification flight readiness is assessed.
No flash, arming, flight, reset, rebase, force-push or remote push is performed
in this task. Frozen V1, Clean Native V2 and Roll TBR V2 branches stay intact.
