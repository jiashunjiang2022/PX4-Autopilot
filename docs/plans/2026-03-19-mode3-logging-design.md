# Mode3 Split-Cycle Logging and Metadata Design

**Date:** 2026-03-19

**Context**

The `FUSION_new_modulation` branch already contains the intended split-cycle control path in `FLAP_FM_MODE=3`: the user-facing command is the downstroke time ratio `Δ = T_down / T`, and the controller converts that ratio into different upstroke/downstroke instantaneous frequencies while keeping the effective cycle period constant. That matches the intended hardware bring-up direction more closely than the separate duty-shaping plan on `feature/qgc-duty-cycle`.

The remaining gap is not the control law. The immediate needs are:

1. make the `mode3` parameter metadata explicit and easy to tune from QGC,
2. log enough information in ULog to reconstruct the commanded and measured flapping waveform directly,
3. increase logging cadence for the most relevant flap-control topics so the recorded waveform is useful on real hardware.

**Approved Scope**

Do not change the `mode3` control law in this iteration.

Only make the following changes:

- add explicit waveform-oriented fields to `FlapControlStatus`,
- publish those new fields from `RpmPid.cpp`,
- rewrite the `mode3` parameter descriptions around `Δ = T_down / T`,
- raise the logger rate for `flap_control_status`, `flap_motor_setpoint`, and `wing_phase`.

**Non-Goals**

- no new control mode,
- no conversion from `mode3` back to `mode2`,
- no new closed-loop `Δ` controller,
- no changes to the existing split-cycle dynamics, blend law, current limiting, or slew logic,
- no new actuator routing or QGC UI beyond clearer parameter descriptions.

## Existing Baseline

`mode3` already computes:

- `delta_cmd`, `delta_slewed`, `delta_applied`, and `delta_meas`,
- `phase_deg`, `phase_ref_deg`, and `phase_err_deg`,
- `f_inst_sp_hz`, `rpm_sp`, `rpm_meas`,
- `u_ref`, `u_pid`, and `u_out`.

These are published on `flap_control_status`, but the current topic does not explicitly expose a waveform sample or the separate upstroke/downstroke target frequencies. That makes post-flight interpretation less direct than necessary.

## Design

### 1. Extend `FlapControlStatus` with waveform-centric fields

Add the following fields to `msg/FlapControlStatus.msg`:

- `float32 f_up_cmd_hz`
- `float32 f_down_cmd_hz`
- `bool in_downstroke`
- `float32 wave_cmd`
- `float32 wave_meas`

**Definitions**

- `f_up_cmd_hz`: instantaneous target frequency associated with the upstroke half-cycle for the current `Δ` command.
- `f_down_cmd_hz`: instantaneous target frequency associated with the downstroke half-cycle for the current `Δ` command.
- `in_downstroke`: controller-side half-cycle classification for the current sample after phase alignment.
- `wave_cmd`: normalized commanded flap waveform sample reconstructed from the internal phase reference, using `cos(phase_ref_deg)`.
- `wave_meas`: normalized measured flap waveform sample reconstructed from the measured phase, using `cos(phase_deg)`.

These fields are intended for visualization and correlation, not for control.

### 2. Publish the new fields in `RpmPid.cpp`

Within the existing `mode3` branch:

- compute `f_up_cmd_hz = f_eff_sp / (2 * (1 - delta_applied))`,
- compute `f_down_cmd_hz = f_eff_sp / (2 * delta_applied)`,
- derive `in_downstroke` from the same phase classification already used by the split-cycle scheduler,
- compute `wave_meas` from the aligned measured phase,
- compute `wave_cmd` from the internal phase reference when valid, otherwise publish `NaN`.

Outside valid `mode3`/phase-reference conditions:

- publish `NaN` for unavailable floating-point fields,
- publish `false` for `in_downstroke`.

This keeps the topic self-descriptive without changing any control output.

### 3. Rewrite parameter descriptions around `Δ`

Update `src/modules/rpm_pid/rpm_pid_params.c` so `mode3` is described using the real operator mental model:

- `FLAP_FM_MODE = 3` should explicitly say: split-cycle frequency modulation by downstroke ratio `Δ = T_down / T`.
- `FLAP_SC_DELTA` should explicitly define:
  - `0.5 = symmetric`,
  - `>0.5 = longer downstroke / shorter upstroke`,
  - `<0.5 = shorter downstroke / longer upstroke`,
  - total cycle period remains constant.
- `FLAP_SC_SLEW` should be described as a rate limit on the commanded `Δ`.
- `FLAP_SC_BLEND` should state that it smooths the transition near reversal without redefining `Δ`.
- `FLAP_SC_PH_K` should note that `0` best matches the ideal open-loop split-cycle schedule.
- `FLAP_SC_ILIM_A`, `FLAP_SC_REC_TAU`, and `FLAP_SC_U_SLEW` should be labeled as engineering protection parameters.

The goal is to make QGC tuning language match the simulation and paper vocabulary.

### 4. Increase logging cadence for the key flap topics

Update `src/modules/logger/logged_topics.cpp`:

- `flap_motor_setpoint`: 20 ms -> 10 ms
- `flap_control_status`: 20 ms -> 10 ms
- `wing_phase`: 20 ms -> 10 ms

No control topic behavior changes are required. This is purely to improve waveform reconstruction in ULog.

`flap_frequency` remains unchanged in this iteration because the richer state now lives in `flap_control_status`.

## Validation

### Build Validation

- Rebuild the firmware and confirm the new uORB message compiles.
- Confirm `flap_control_status` still logs successfully.

### Bench / Hardware Validation

Run with:

- `FLAP_FM_MODE = 3`
- `FLAP_SC_DELTA = 0.50`
- then `0.35`
- then `0.65`

Inspect:

- `listener wing_phase 5`
- `listener flap_frequency 5`
- `listener flap_motor_setpoint 5`
- `listener flap_control_status 5`

Verify in ULog:

- `delta_cmd`, `delta_applied`, `delta_meas`,
- `f_up_cmd_hz`, `f_down_cmd_hz`,
- `wave_cmd`, `wave_meas`,
- `in_downstroke`.

## Risks and Mitigations

- **Phase alignment is wrong**
  - Symptom: `Δ` changes in the opposite physical direction.
  - Mitigation: adjust `FLAP_PHASE_SHIFT` before changing control logic.

- **Waveform still looks ambiguous in logs**
  - Symptom: `phase` is present but hard to interpret quickly.
  - Mitigation: the new explicit waveform fields avoid repeated offline reconstruction.

- **Higher logger rate increases log bandwidth**
  - Symptom: larger ULog files.
  - Mitigation: raise only the three flap-critical topics to 10 ms, not all related telemetry.

## Recommended Next Step

Implement this as a small metadata/logging patch on `FUSION_new_modulation`, verify the build, then perform a short hardware smoke test with `Δ = 0.50`, `0.35`, and `0.65` before making any further control-law changes.
