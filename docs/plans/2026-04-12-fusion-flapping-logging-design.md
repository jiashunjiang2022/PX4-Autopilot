# FUSION Flapping Logging Design

## Goal

Add the `air` branch's useful flapping-data capture improvements to `FUSION` without importing the airspeed-robustness feature set and without breaking existing `FUSION` flapping control behavior.

## Scope

This design includes four areas:

1. A `logger` flapping dataset profile for high-value flapping topics.
2. A `Tools/topic_dump.py` helper adapted for `FUSION` flapping data collection.
3. Low-risk AS5600 publishing cleanup for flap-frequency logging.
4. A compatible `wing_phase` accuracy upgrade using hall-timestamp interpolation and unit-tested math helpers.

This design explicitly excludes:

- `Ekf2AirspeedQuality`
- `airspeed_injector`
- pitot blockage fallback
- airspeed gating or fusion behavior changes

## Constraints

- Existing `FUSION` consumers of `wing_phase.phase_deg` and `wing_phase.hall_pulse_count` must continue to work.
- `rpm_pid` and `FunctionFlapMotor` behavior must remain source-compatible.
- Logging additions should be useful even if no airspeed sensor is present.
- The current branch already has unrelated submodule pointer changes; those must not be reverted.

## Design

### 1. Flapping Dataset Logging Profile

Add a new `SDLOG_PROFILE` bit for flapping dataset capture in the existing `FUSION` logger parameter implementation.

The profile will include:

- `sensor_combined`
- `vehicle_acceleration`
- `vehicle_angular_velocity`
- `vehicle_attitude`
- `vehicle_local_position`
- `vehicle_odometry`
- `actuator_motors`
- `actuator_servos`
- `vehicle_thrust_setpoint`
- `vehicle_torque_setpoint`
- `flap_frequency`
- `rpm`
- `encoder_count`
- `wing_phase`
- `debug_vect`
- `sensor_gps`
- `sensor_gnss_relative`

It will not include airspeed-diagnostics topics from `air`.

### 2. FUSION-Oriented Topic Dump Tool

Add `Tools/topic_dump.py`, but adapt the field presets to `FUSION` topics rather than copying the airspeed-centric version unchanged.

The tool will support CSV dumping for:

- `rpm`
- `flap_frequency`
- `encoder_count`
- `wing_phase`
- `actuator_motors`
- `flap_motor_setpoint`
- optionally `vehicle_attitude` for correlation

This keeps the tool aligned with how `FUSION` is actually debugged.

### 3. AS5600 Logging Enhancement

Port the low-risk AS5600 changes that improve flap-frequency publication quality:

- refresh `FLAP_RATIO` through a helper
- publish `flap_frequency` as magnitude
- clamp very small values to zero instead of noisy near-zero oscillation
- keep current encoder-count and RPM publishing intact

This is treated as a logging/observability change, not a control change.

### 4. Compatible Wing Phase Accuracy Upgrade

Port `WingPhaseMath` and hall-timestamp interpolation, but do not replace the `FUSION` message contract.

Instead:

- keep `phase_deg`
- keep `hall_pulse_count`
- keep existing consumers unchanged
- extend `WingPhase.msg` with additional fields from `air` where useful
- compute interpolated hall zero count internally
- publish both legacy degree output and richer derived fields

`RPMCapture` will also be updated so `hall_event` is published only on a fresh pulse and uses the pulse timestamp, matching the interpolation logic.

## Risks

### Logger profile risk

Low. Mostly parameter and topic-list plumbing.

### AS5600 risk

Low. It changes flap-frequency semantics near zero and sign handling, but not the main control path.

### Wing phase risk

Medium. This affects the data consumed by glide-stop alignment and phase-based feedforward. Compatibility is preserved by keeping `phase_deg` behavior available to current modules.

## Verification Strategy

- Add unit tests for `WingPhaseMath`.
- Build or test affected modules where practical.
- Verify the new logger profile compiles and is selectable via parameter metadata.
- Verify `topic_dump.py --help`.
- Verify `wing_phase` message and consumers compile together.

## Recommendation

Implement in this order:

1. logger profile
2. topic dump tool
3. AS5600 updates
4. compatible wing-phase upgrade

This keeps the highest-risk behavior change last and preserves a usable logging improvement even if later work needs refinement.
