# Flapping Dataset Logging Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Add a QGC-switchable PX4 logging profile for flapping-wing dataset collection so flight logs contain high-rate state, actuation, wing kinematics, airdata, and RTK quality signals needed for offline dataset generation.

**Architecture:** Extend the existing `SDLOG_PROFILE` bitmask with a new `FLAPPING_DATASET` profile and centralize its topic selection in `LoggedTopics::add_flapping_dataset_topics()`. Phase 1 reuses existing uORB topics and only changes logging coverage and rates; Phase 2 optionally adds a dedicated `WingKinematics.msg` published by the AS5600 driver to avoid reconstructing phase from `debug_vect` and `encoder_count`.

**Tech Stack:** PX4 logger, uORB, message generation via `msg/CMakeLists.txt`, QGroundControl parameter UI, ULog, `pyulog`

---

## Scope

This plan is intentionally biased toward low-risk logging changes:

- Keep the existing logger architecture.
- Make recording controllable from QGC through `SDLOG_PROFILE`.
- Avoid `logger_topics.txt` because it replaces the default topic list and cannot be toggled from QGC.
- Reuse existing topics first.
- Treat a new `WingKinematics.msg` as an optional Phase 2 cleanup, not a prerequisite for the first usable dataset.

## Phase Summary

### Phase 1: Minimum Viable Dataset Logging

Add a new logger profile bit and make it record the existing topics that are already published but currently missing or downsampled too aggressively.

### Phase 2: Clean Wing Kinematics Topic

If offline phase reconstruction from `encoder_count` and `debug_vect` is too awkward, add a dedicated `WingKinematics.msg` and publish it from the AS5600 driver.

## Target Logged Signals

### Required state and dynamics signals

- `sensor_combined`
- `vehicle_acceleration`
- `vehicle_angular_velocity`
- `vehicle_attitude`
- `vehicle_local_position`
- `vehicle_odometry`

### Required actuation signals

- `actuator_motors`
- `actuator_servos`
- `vehicle_thrust_setpoint`
- `vehicle_torque_setpoint`

### Required wing kinematics signals

- `flap_frequency`
- `rpm`
- `encoder_count`
- `debug_vect`

### Required airdata and quality signals

- `airspeed_validated`
- `ekf2_airspeed_quality`
- `vehicle_air_data`
- `wind`
- `airspeed_wind`

### Required RTK and GPS quality signals

- `sensor_gps`
- `sensor_gnss_relative`

## Recommended Logging Rates

Use full rate (`interval_ms = 0`) for the flapping dataset profile unless a topic is known to be too expensive or too noisy to justify full-rate logging.

### Full-rate topics

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
- `debug_vect`
- `airspeed_validated`
- `ekf2_airspeed_quality`
- `vehicle_air_data`
- `wind`
- `airspeed_wind`
- `sensor_gps`
- `sensor_gnss_relative`

### Notes

- `sensor_combined` is already logged at full rate in the default set, but include it explicitly so the new profile is self-contained.
- `wind` is currently logger-limited to `1000 ms` in the default set even though EKF2 publishes it during normal updates; record it at full rate in the new profile.
- `sensor_gps` and `sensor_gnss_relative` should be logged at full rate in this profile instead of relying on the separate `HIGH_RATE_SENSORS` bit.

## Task 1: Add a QGC-Switchable Logger Profile Bit

**Files:**
- Modify: `/home/zn/PX4-Autopilot/src/modules/logger/logged_topics.h`
- Modify: `/home/zn/PX4-Autopilot/src/modules/logger/logged_topics.cpp`
- Modify: `/home/zn/PX4-Autopilot/src/modules/logger/module.yaml`

**Step 1: Extend the profile enum**

Add a new enum value to `SDLogProfileMask`, for example:

```cpp
FLAPPING_DATASET = 1 << 12
```

Place it after `HIGH_RATE_SENSORS` in [logged_topics.h](/home/zn/PX4-Autopilot/src/modules/logger/logged_topics.h#L46).

**Step 2: Add a helper declaration**

Declare a new private helper in `LoggedTopics`:

```cpp
void add_flapping_dataset_topics();
```

**Step 3: Wire the new profile into initialization**

In `LoggedTopics::initialize_configured_topics()` in [logged_topics.cpp](/home/zn/PX4-Autopilot/src/modules/logger/logged_topics.cpp#L551), add:

```cpp
if (profile & SDLogProfileMask::FLAPPING_DATASET) {
    add_flapping_dataset_topics();
}
```

Keep it near the end so its rates override slower defaults when topics overlap.

**Step 4: Expose the bit in parameter metadata**

Update [module.yaml](/home/zn/PX4-Autopilot/src/modules/logger/module.yaml#L71):

- Add bit `12: Flapping dataset`
- Extend the long description to mention the new bit
- Raise `max` from `4095` to `8191`

This is what makes the profile visible and toggleable from QGC through `SDLOG_PROFILE`.

**Step 5: Build-verify metadata and logger code**

Run:

```bash
cd /home/zn/PX4-Autopilot
make px4_sitl_default -j"$(nproc)"
```

Expected:

- Build succeeds
- No enum or metadata generation errors

**Step 6: Commit**

```bash
git -C /home/zn/PX4-Autopilot add src/modules/logger/logged_topics.h src/modules/logger/logged_topics.cpp src/modules/logger/module.yaml
git -C /home/zn/PX4-Autopilot commit -m "feat: add flapping dataset logger profile"
```

## Task 2: Implement the Phase 1 Topic Set

**Files:**
- Modify: `/home/zn/PX4-Autopilot/src/modules/logger/logged_topics.cpp`
- Modify: `/home/zn/PX4-Autopilot/src/modules/logger/logged_topics.h`

**Step 1: Create `add_flapping_dataset_topics()`**

Implement the helper in [logged_topics.cpp](/home/zn/PX4-Autopilot/src/modules/logger/logged_topics.cpp).

Use `add_topic()` or `add_topic_multi()` for topics that should be required. Use `add_optional_topic()` or `add_optional_topic_multi()` only when the publisher may legitimately be absent on some builds or airframes.

**Step 2: Add the core dynamics topics**

Add these at full rate:

```cpp
add_topic("sensor_combined");
add_topic("vehicle_acceleration");
add_topic("vehicle_angular_velocity");
add_topic("vehicle_attitude");
add_topic("vehicle_local_position");
add_topic("vehicle_odometry");
```

**Step 3: Add the actuation topics**

Add these at full rate:

```cpp
add_topic("actuator_motors");
add_topic("actuator_servos");
add_topic_multi("vehicle_thrust_setpoint", 0, 2);
add_topic_multi("vehicle_torque_setpoint", 0, 2);
```

**Step 4: Add the wing-kinematics topics**

Add these at full rate:

```cpp
add_topic("flap_frequency");
add_optional_topic_multi("rpm");
add_topic("encoder_count");
add_topic("debug_vect");
```

`rpm` should stay optional because some boards or builds may not publish it.

**Step 5: Add the airdata and RTK topics**

Add these at full rate:

```cpp
add_optional_topic("airspeed_validated");
add_topic_multi("ekf2_airspeed_quality");
add_topic("vehicle_air_data");
add_topic("wind");
add_optional_topic_multi("airspeed_wind");
add_topic_multi("sensor_gps", 0, 4);
add_topic_multi("sensor_gnss_relative", 0, 1);
```

**Step 6: Rebuild and verify the topic list compiles**

Run:

```bash
cd /home/zn/PX4-Autopilot
make px4_sitl_default -j"$(nproc)"
```

Expected:

- Logger compiles
- Added topics resolve to valid uORB metadata

**Step 7: Commit**

```bash
git -C /home/zn/PX4-Autopilot add src/modules/logger/logged_topics.cpp src/modules/logger/logged_topics.h
git -C /home/zn/PX4-Autopilot commit -m "feat: add flapping dataset topic set"
```

## Task 3: Validate QGC Parameter Workflow

**Files:**
- Modify: `/home/zn/PX4-Autopilot/src/modules/logger/module.yaml`
- Optional: `/home/zn/PX4-Autopilot/docs/en/dev_log/logging.md`

**Step 1: Verify the parameter semantics are operator-friendly**

Ensure `module.yaml` clearly explains:

- The new bit is intended for flapping dataset collection
- It can be OR-ed with existing bits
- It increases bandwidth and log size
- It requires reboot

**Step 2: Optionally document the intended value combinations**

If you want in-repo operator guidance, add a short note to [logging.md](/home/zn/PX4-Autopilot/docs/en/dev_log/logging.md#L53) with example `SDLOG_PROFILE` values, for example:

- `1`: default
- `1 + 4096`: default + flapping dataset
- `1 + 2 + 4096`: default + estimator replay + flapping dataset

**Step 3: Verify from QGC on hardware**

After flashing firmware:

1. Open QGC parameter editor
2. Find `SDLOG_PROFILE`
3. Confirm the new bit appears in the metadata text
4. Enable it
5. Reboot the flight controller

Expected:

- QGC accepts the new parameter value
- The value survives reboot

## Task 4: Validate Runtime Publishers Before Flight

**Files:**
- No code changes

**Step 1: Check wing-sensor topics on target**

On the PX4 shell, run:

```bash
listener encoder_count 5
listener rpm 5
listener flap_frequency 5
listener debug_vect 5
```

Expected:

- `encoder_count` updates when the wing moves
- `rpm` and `flap_frequency` are finite during operation
- `debug_vect` contains the AS5600 samples

**Step 2: Check estimator and RTK topics on target**

Run:

```bash
listener airspeed_validated 5
listener ekf2_airspeed_quality 5
listener wind 5
listener sensor_gps 5
listener sensor_gnss_relative 5
listener vehicle_odometry 5
```

Expected:

- Topics are published on the aircraft build
- GPS and RTK validity fields are populated when hardware is available

## Task 5: Flight-Test the New Logging Profile

**Files:**
- No code changes

**Step 1: Enable the profile and collect a short test log**

Collect one short hover or tethered flapping run after enabling the new `SDLOG_PROFILE` bit.

**Step 2: Inspect topic presence and effective rates**

Run:

```bash
python - <<'PY'
from pyulog import ULog
import numpy as np
path = "test.ulg"
topics = [
    "sensor_combined",
    "vehicle_acceleration",
    "vehicle_angular_velocity",
    "vehicle_attitude",
    "vehicle_local_position",
    "vehicle_odometry",
    "actuator_motors",
    "actuator_servos",
    "vehicle_thrust_setpoint",
    "vehicle_torque_setpoint",
    "flap_frequency",
    "rpm",
    "encoder_count",
    "debug_vect",
    "airspeed_validated",
    "ekf2_airspeed_quality",
    "vehicle_air_data",
    "wind",
    "airspeed_wind",
    "sensor_gps",
    "sensor_gnss_relative",
]
u = ULog(path, message_name_filter_list=topics)
for d in sorted(u.data_list, key=lambda x: (x.name, x.multi_id)):
    t = d.data["timestamp"]
    hz = float("nan")
    if len(t) > 1:
        dt = np.median(np.diff(t)) / 1e6
        hz = 1.0 / dt if dt > 0 else float("nan")
    print(f"{d.name:24s} inst={d.multi_id} n={len(t):6d} hz={hz:8.2f}")
PY
```

Expected:

- `encoder_count` is present
- `debug_vect` is present
- `rpm` and `flap_frequency` are significantly faster than before
- `sensor_gps` is no longer stuck at `1 Hz`
- `wind` and `airspeed_wind` are no longer stuck at `1 Hz`

**Step 3: Check for logger overload**

Review the log for dropped samples or logger warnings. If write bandwidth becomes a problem, back off these topics in this order:

1. `debug_vect`
2. `airspeed_wind`
3. `wind`
4. `vehicle_local_position` if `vehicle_odometry` is already sufficient

## Task 6: Optional Phase 2 Cleanup With `WingKinematics.msg`

**Files:**
- Create: `/home/zn/PX4-Autopilot/msg/WingKinematics.msg`
- Modify: `/home/zn/PX4-Autopilot/msg/CMakeLists.txt`
- Modify: `/home/zn/PX4-Autopilot/src/drivers/encoder/as5600/AS5600.hpp`
- Modify: `/home/zn/PX4-Autopilot/src/drivers/encoder/as5600/AS5600.cpp`
- Modify: `/home/zn/PX4-Autopilot/src/modules/logger/logged_topics.cpp`

**Step 1: Define a clean message**

Create `WingKinematics.msg` with fields similar to:

```plain
uint64 timestamp
float32 angle_rad
float32 phase_rad
float32 phase_norm
float32 omega_rad_s
float32 rpm_raw
float32 rpm_estimate
float32 flap_frequency_hz
int64 total_count
uint32 position_raw
```

**Step 2: Register the message**

Add `WingKinematics.msg` to [msg/CMakeLists.txt](/home/zn/PX4-Autopilot/msg/CMakeLists.txt).

**Step 3: Publish from AS5600**

Update [AS5600.hpp](/home/zn/PX4-Autopilot/src/drivers/encoder/as5600/AS5600.hpp#L50) and [AS5600.cpp](/home/zn/PX4-Autopilot/src/drivers/encoder/as5600/AS5600.cpp#L117) to publish the new message inside `RunImpl()`.

Derive:

- `angle_rad` from the instantaneous encoder angle
- `phase_rad` from the wrapped angle in `[0, 2*pi)`
- `phase_norm` from `phase_rad / (2*pi)`
- `omega_rad_s` from the unwrapped derivative already computed for `rpm_raw`

**Step 4: Log the new topic in the flapping profile**

Add:

```cpp
add_topic("wing_kinematics");
```

to `add_flapping_dataset_topics()`.

**Step 5: Rebuild and verify**

Run:

```bash
cd /home/zn/PX4-Autopilot
make px4_sitl_default -j"$(nproc)"
```

Then verify on target:

```bash
listener wing_kinematics 5
```

**Step 6: Commit**

```bash
git -C /home/zn/PX4-Autopilot add msg/WingKinematics.msg msg/CMakeLists.txt src/drivers/encoder/as5600/AS5600.hpp src/drivers/encoder/as5600/AS5600.cpp src/modules/logger/logged_topics.cpp
git -C /home/zn/PX4-Autopilot commit -m "feat: publish wing kinematics for dataset logging"
```

## Task 7: Freeze the Dataset Logging Contract

**Files:**
- Create or modify: `/home/zn/PX4-Autopilot/docs/en/dev_log/logging.md`
- Optional: `/home/zn/PX4-Autopilot/docs/en/flapping_dataset_logging.md`

**Step 1: Document the final topic contract**

Write down:

- Required `SDLOG_PROFILE` value
- Required hardware publishers
- Expected minimum rates
- Quality fields to use for filtering airspeed and RTK data offline

**Step 2: Document the offline assumptions**

State explicitly:

- `airspeed_validated` is an input and quality-gated measurement, not a ground-truth target
- `sensor_gps.fix_type`, `heading_accuracy`, and `sensor_gnss_relative` validity bits must be used for filtering
- Effective wrench labels are generated offline from rigid-body dynamics, not onboard

## Exit Criteria

- A new `SDLOG_PROFILE` bit can be enabled from QGC.
- Enabling the bit causes the aircraft to record the required high-rate dataset topics without needing `logger_topics.txt`.
- A short validation flight confirms the presence of `encoder_count`, `rpm`, `flap_frequency`, `ekf2_airspeed_quality`, `sensor_gps`, and `wind` at usable rates.
- Optional Phase 2 produces a clean `wing_kinematics` topic if the existing AS5600 topics are not sufficient.

## Risks and Mitigations

### Risk: SD card write bandwidth becomes the bottleneck

Mitigation:

- Prefer Phase 1 first
- Measure actual logged rates after a short test
- Remove `debug_vect` first if bandwidth is tight

### Risk: RTK topics exist but validity bits are false

Mitigation:

- Log `sensor_gps` and `sensor_gnss_relative` anyway
- Use offline gating by validity flags
- Do not assume all flights have RTK-fixed quality

### Risk: Airspeed remains noisy even at higher log rate

Mitigation:

- Keep `ekf2_airspeed_quality`
- Use `airspeed_source`
- Treat airspeed as an auxiliary feature instead of a truth signal

Plan complete and saved to `/home/zn/PX4-Autopilot/docs/plans/2026-03-21-flapping-dataset-logging.md`.

Two execution options:

1. Subagent-Driven (this session) - I dispatch fresh subagent per task, review between tasks, fast iteration
2. Parallel Session (separate) - Open new session with executing-plans, batch execution with checkpoints

Which approach?
