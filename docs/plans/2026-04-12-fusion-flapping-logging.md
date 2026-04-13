# FUSION Flapping Logging Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Add FUSION-focused flapping data logging, dump tooling, AS5600 observability cleanup, and compatible wing-phase interpolation without importing airspeed-gating behavior.

**Architecture:** Keep logging features independent from airspeed robustness work. Preserve the current `FUSION` `wing_phase` interface used by `rpm_pid` and `FunctionFlapMotor`, while improving the phase estimate internally through hall-timestamp interpolation.

**Tech Stack:** PX4 logger, uORB messages, C++ modules, PX4 Python tooling, gtest.

---

### Task 1: Add Logger Profile Plumbing

**Files:**
- Modify: `src/modules/logger/logged_topics.h`
- Modify: `src/modules/logger/logged_topics.cpp`
- Modify: `src/modules/logger/params.c`

**Step 1: Add a failing expectation checklist**

Confirm the codebase currently has no `FLAPPING_DATASET` logger profile and no dedicated flapping dataset topic list on `FUSION`.

**Step 2: Implement minimal profile support**

Add:

- a new `SDLogProfileMask::FLAPPING_DATASET`
- a new `add_flapping_dataset_topics()` helper
- `initialize_configured_topics()` wiring
- `SDLOG_PROFILE` docs/bit metadata in `params.c`

**Step 3: Keep profile airspeed-free**

Only log flapping and state-estimation context topics useful to `FUSION`, excluding `ekf2_airspeed_quality` and other airspeed-specific diagnostics.

### Task 2: Add FUSION Topic Dump Tool

**Files:**
- Create: `Tools/topic_dump.py`

**Step 1: Create a small failing/manual verification target**

Plan to verify the script with:

```bash
python3 Tools/topic_dump.py --help
```

and ensure the listed choices match FUSION flapping topics.

**Step 2: Implement the script**

Start from the `air` version structure, but use FUSION-focused topic presets and CSV field lists.

### Task 3: Add AS5600 Logging Improvements

**Files:**
- Modify: `src/drivers/encoder/as5600/AS5600.cpp`
- Modify: `src/drivers/encoder/as5600/AS5600.hpp`
- Modify: `src/drivers/encoder/as5600/CMakeLists.txt`
- Create: `src/drivers/encoder/as5600/module.yaml`

**Step 1: Preserve current outputs**

Keep RPM, encoder_count, and debug publishing intact.

**Step 2: Implement low-risk changes**

Add:

- `update_flap_ratio_param()`
- absolute `flap_frequency`
- near-zero clamp
- module metadata for `FLAP_RATIO`

### Task 4: Add Wing Phase Math Tests First

**Files:**
- Create: `src/modules/wing_phase/WingPhaseMath.hpp`
- Create: `src/modules/wing_phase/WingPhaseMath.cpp`
- Create: `src/modules/wing_phase/WingPhaseMathTest.cpp`
- Modify: `src/modules/wing_phase/CMakeLists.txt`

**Step 1: Write the failing tests**

Add tests for:

- invalid result before hall lock
- wrapped phase calculation
- unwrapped phase calculation
- fractional hall zero count
- timestamp interpolation inside encoder sample bracket
- rejection outside the bracket

**Step 2: Run the test and confirm failure**

Run a targeted test/build command after creating the test.

**Step 3: Implement minimal math helpers**

Add standalone pure helpers for interpolation and phase computation.

### Task 5: Upgrade Wing Phase Compatibly

**Files:**
- Modify: `msg/WingPhase.msg`
- Modify: `src/modules/wing_phase/WingPhase.cpp`
- Modify: `src/drivers/rpm_capture/RPMCapture.cpp`
- Modify: `src/drivers/rpm_capture/RPMCapture.hpp`

**Step 1: Preserve existing consumer contract**

Keep `phase_deg` and `hall_pulse_count` in `WingPhase.msg`.

**Step 2: Extend message**

Append richer fields such as:

- `phase_rad`
- `phase_unwrapped_rad`
- `phase_sin`
- `phase_cos`
- `flap_frequency_hz`
- `encoder_position_raw`
- `encoder_total_count`
- `phase_valid`

**Step 3: Implement interpolation**

Update `WingPhase.cpp` to:

- track previous/current encoder samples
- latch hall timestamps
- interpolate zero count when the hall timestamp falls between encoder samples
- compute `phase_deg` from the improved estimate for legacy consumers

**Step 4: Update hall-event publication**

Update `RPMCapture.cpp` to publish `hall_event` only on a new hall pulse and with the pulse timestamp.

### Task 6: Verify End-to-End

**Files:**
- Verify: `src/modules/logger/*`
- Verify: `src/drivers/encoder/as5600/*`
- Verify: `src/modules/wing_phase/*`
- Verify: `src/drivers/rpm_capture/*`
- Verify: `Tools/topic_dump.py`

**Step 1: Run targeted verification**

Run targeted commands that prove:

- `topic_dump.py` parses and shows help
- `WingPhaseMath` tests pass
- affected code compiles cleanly

**Step 2: Inspect final diff**

Confirm no unrelated submodule pointer changes were modified by this work.
