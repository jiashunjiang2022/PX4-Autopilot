# Duty-Cycle Control Implementation Plan

> **For Codex:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Replace the current open-loop `FLAP_PHASE_DUTY` amplitude bias in `origin/FUSION` with a QGC-adjustable duty-cycle controller that can command, estimate, and log asymmetric downstroke/upstroke timing while preserving the existing mean-frequency loop that already works on hardware.

**Architecture:** Keep `src/modules/rpm_pid/RpmPid.cpp` as the owner of the flapping motor override. Split the behavior into two paths: one path keeps the current mean-frequency PID around `rpm_sp`, and a second path uses `wing_phase` edge timing to estimate `delta_meas` and drive a zero-mean within-cycle shaping command. Preserve the current cosine shaping as a legacy mode so flight bring-up can instantly fall back if the new controller misbehaves.

**Tech Stack:** PX4 C++ modules, uORB messages, PX4 parameter system / QGC parameter UI, AS5600 + hall + `wing_phase`, gtest via `make tests`, FMU-v6c firmware build via `make px4_fmu-v6c_default`.

---

### Task 1: Extract the duty-cycle math into a tested helper

**Files:**
- Create: `src/modules/rpm_pid/DutyCycleWarp.hpp`
- Create: `src/modules/rpm_pid/DutyCycleWarp.cpp`
- Create: `src/modules/rpm_pid/DutyCycleWarpTest.cpp`
- Modify: `src/modules/rpm_pid/CMakeLists.txt`

**Step 1: Write the failing test**

Create `src/modules/rpm_pid/DutyCycleWarpTest.cpp` with unit tests for the pure math only:

```cpp
TEST(DutyCycleWarp, Wrap360HandlesNegativeAngles);
TEST(DutyCycleWarp, ShiftedPhaseDetectsDownstrokeHalfCycle);
TEST(DutyCycleWarp, CrossingDetectorHandlesZeroDegreeWrap);
TEST(DutyCycleWarp, SymmetricDutyProducesSymmetricScaling);
```

Use concrete checks such as:
- `wrap360(-10.f) == 350.f`
- phase ranges `90..270` map to downstroke after shift
- edge crossing detection works when `359 -> 1` crosses `0`
- `duty_cmd = 0.5` yields zero net asymmetry bias

**Step 2: Run test to verify it fails**

Run:

```bash
cd /home/zn/PX4-Autopilot
make tests TESTFILTER=DutyCycleWarp
```

Expected: FAIL because the helper and test target do not exist yet.

**Step 3: Write the minimal implementation**

Create a small helper API in `DutyCycleWarp.hpp` / `DutyCycleWarp.cpp` so all tricky logic is testable without running PX4 modules:

```cpp
struct StrokePhaseState {
	float phase_shifted_deg;
	bool in_downstroke;
	float stroke_progress;
};

float wrap360(float deg);
bool crossed_phase_edge(float prev_deg, float curr_deg, float edge_deg);
StrokePhaseState evaluate_stroke_phase(float phase_deg, float phase_shift_deg);
float shape_envelope(float stroke_progress);
float duty_to_asymmetry(float duty_cmd);
```

Design rules for this helper:
- keep all outputs continuous across `0/360`
- define stroke boundaries in one place only
- make `shape_envelope()` zero at reversals and peak near stroke midpoint
- keep `duty_to_asymmetry(0.5f) == 0`

**Step 4: Run test to verify it passes**

Run:

```bash
cd /home/zn/PX4-Autopilot
make tests TESTFILTER=DutyCycleWarp
```

Expected: PASS and produce a unit test binary for the helper.

**Step 5: Commit**

```bash
git add src/modules/rpm_pid/CMakeLists.txt \
        src/modules/rpm_pid/DutyCycleWarp.hpp \
        src/modules/rpm_pid/DutyCycleWarp.cpp \
        src/modules/rpm_pid/DutyCycleWarpTest.cpp
git commit -m "feat: add tested duty-cycle warp helper"
```

### Task 2: Add measured duty-cycle estimation to `rpm_pid`

**Files:**
- Modify: `src/modules/rpm_pid/RpmPid.hpp`
- Modify: `src/modules/rpm_pid/RpmPid.cpp`
- Modify: `src/modules/rpm_pid/DutyCycleWarp.hpp`
- Modify: `src/modules/rpm_pid/DutyCycleWarp.cpp`
- Modify: `src/modules/rpm_pid/DutyCycleWarpTest.cpp`

**Step 1: Extend the failing test**

Add tests that simulate phase edge timestamps and verify the estimator math:

```cpp
TEST(DutyCycleWarp, DutyEstimatorReportsHalfForSymmetricStrokeTiming);
TEST(DutyCycleWarp, DutyEstimatorTracksLongerDownstroke);
TEST(DutyCycleWarp, DutyEstimatorRejectsIncompleteCycles);
```

Use synthetic edge times such as:
- `down = 0.12 s`, `up = 0.12 s` -> `delta_meas = 0.5`
- `down = 0.16 s`, `up = 0.08 s` -> `delta_meas = 0.667`
- missing edge -> estimator invalid

**Step 2: Run test to verify it fails**

Run:

```bash
cd /home/zn/PX4-Autopilot
make tests TESTFILTER=DutyCycleWarp
```

Expected: FAIL because the estimator logic does not exist yet.

**Step 3: Write the minimal implementation**

Extend the helper with a small estimator state that `RpmPid` can own:

```cpp
struct DutyCycleEstimate {
	float duty_meas;
	bool valid;
};

struct DutyCycleEstimatorState {
	hrt_abstime last_down_start_us;
	hrt_abstime last_up_start_us;
	hrt_abstime last_edge_us;
	float duty_meas_filtered;
	bool valid;
};

bool update_duty_cycle_estimator(
	DutyCycleEstimatorState &state,
	float prev_phase_deg,
	float curr_phase_deg,
	hrt_abstime now_us,
	float phase_shift_deg,
	DutyCycleEstimate &out);
```

Integrate it into `RpmPid` by:
- storing previous shifted phase
- detecting stroke-edge crossings every RPM callback
- computing `delta_meas = T_down / (T_down + T_up)`
- low-pass filtering `delta_meas`
- invalidating the estimate when `wing_phase` is stale or edges are missing

**Step 4: Run test to verify it passes**

Run:

```bash
cd /home/zn/PX4-Autopilot
make tests TESTFILTER=DutyCycleWarp
```

Expected: PASS, including the new estimator cases.

**Step 5: Commit**

```bash
git add src/modules/rpm_pid/RpmPid.hpp \
        src/modules/rpm_pid/RpmPid.cpp \
        src/modules/rpm_pid/DutyCycleWarp.hpp \
        src/modules/rpm_pid/DutyCycleWarp.cpp \
        src/modules/rpm_pid/DutyCycleWarpTest.cpp
git commit -m "feat: estimate measured flapping duty cycle from wing phase"
```

### Task 3: Replace the current open-loop duty scaling with a closed-loop duty controller

**Files:**
- Modify: `src/modules/rpm_pid/RpmPid.hpp`
- Modify: `src/modules/rpm_pid/RpmPid.cpp`
- Modify: `src/modules/rpm_pid/rpm_pid_params.c`
- Modify: `src/modules/rpm_pid/DutyCycleWarpTest.cpp`

**Step 1: Extend the failing test**

Add focused controller-side helper tests for the new asymmetry command:

```cpp
TEST(DutyCycleWarp, DutyErrorGeneratesSignedAsymmetryCommand);
TEST(DutyCycleWarp, SymmetricCommandProducesZeroAsymmetry);
TEST(DutyCycleWarp, AsymmetryCommandRespectsAmplitudeLimit);
```

The checks should verify:
- `duty_cmd > duty_meas` increases downstroke assistance and decreases upstroke assistance
- `duty_cmd = duty_meas` returns zero extra asymmetry
- command saturates cleanly at the configured limit

**Step 2: Run test to verify it fails**

Run:

```bash
cd /home/zn/PX4-Autopilot
make tests TESTFILTER=DutyCycleWarp
```

Expected: FAIL because the asymmetry controller and limits do not exist yet.

**Step 3: Write the minimal implementation**

Refactor `RpmPid` so it has two explicit control outputs:

```cpp
const float u_pid = _kp * rpm_error + _ki * _integral + _kd * d_rpm;
const float duty_error = _duty_cmd - _duty_meas;
const float u_dc = constrain(_duty_kp * duty_error + _duty_ki * _duty_integral,
                             -_phase_ff_amp, _phase_ff_amp);
const float u_phase = u_dc * shape_envelope(stroke_progress) * (in_downstroke ? 1.f : -1.f);
const float u_out = constrain(u_ref + u_pid + u_phase, 0.f, 1.f);
```

Controller rules:
- preserve the existing frequency loop around `rpm_sp`
- keep `FLAP_PHASE_DUTY` as the QGC-exposed duty command parameter
- add a mode parameter so `mode=0` keeps legacy cosine shaping and `mode=1` enables the new duty controller
- keep `FLAP_PHASE_DUTY = 0.5` as the symmetric neutral point
- freeze the duty integrator when `delta_meas` is invalid
- reset duty integrator when disarmed or when `wing_phase` goes stale

Add these new parameters in `rpm_pid_params.c`:

```c
PARAM_DEFINE_INT32(FLAP_PHASE_MODE, 0);
PARAM_DEFINE_FLOAT(FLAP_DC_KP, 0.0f);
PARAM_DEFINE_FLOAT(FLAP_DC_KI, 0.0f);
PARAM_DEFINE_FLOAT(FLAP_DC_IMAX, 0.2f);
```

Keep and reuse these existing parameters:
- `FLAP_PHASE_DUTY` -> commanded duty ratio from QGC
- `FLAP_PHASE_AMP` -> max within-cycle shaping magnitude
- `FLAP_PHASE_SHIFT` -> phase alignment to real mechanical up/downstroke

**Step 4: Run test to verify it passes**

Run:

```bash
cd /home/zn/PX4-Autopilot
make tests TESTFILTER=DutyCycleWarp
make px4_fmu-v6c_default
```

Expected:
- unit tests PASS
- firmware build PASS with the refactored `rpm_pid`

**Step 5: Commit**

```bash
git add src/modules/rpm_pid/RpmPid.hpp \
        src/modules/rpm_pid/RpmPid.cpp \
        src/modules/rpm_pid/rpm_pid_params.c \
        src/modules/rpm_pid/DutyCycleWarpTest.cpp
git commit -m "feat: add closed-loop duty-cycle control mode to rpm_pid"
```

### Task 4: Publish a status topic so ULog can prove the controller did what it commanded

**Files:**
- Create: `msg/FlapDutyStatus.msg`
- Modify: `msg/CMakeLists.txt`
- Modify: `src/modules/logger/logged_topics.cpp`
- Modify: `src/modules/rpm_pid/RpmPid.hpp`
- Modify: `src/modules/rpm_pid/RpmPid.cpp`

**Step 1: Write the failing integration check**

First change `RpmPid.cpp` to publish a `flap_duty_status_s` message before the message type exists yet. This is intentional: the next build should fail and prove the wiring is incomplete.

Expected status fields:

```text
timestamp
duty_cmd
duty_meas
phase_deg
stroke_progress
in_downstroke
duty_valid
rpm_sp
rpm_meas
u_ref
u_pid
u_phase
u_out
```

**Step 2: Run build to verify it fails**

Run:

```bash
cd /home/zn/PX4-Autopilot
make px4_fmu-v6c_default
```

Expected: FAIL because `FlapDutyStatus.msg` and the generated uORB headers do not exist yet.

**Step 3: Write the minimal implementation**

Create `msg/FlapDutyStatus.msg`, add it to `msg/CMakeLists.txt`, add it to `src/modules/logger/logged_topics.cpp`, and publish it from `RpmPid.cpp` every RPM callback.

The message should let post-processing answer all three questions directly:
- what duty ratio was commanded
- what duty ratio was actually realized
- what control effort created the change

Do not overload `flap_motor_setpoint`; keep `FlapDutyStatus` as the analysis/debug topic and `flap_motor_setpoint` as the actuator override topic.

**Step 4: Run build to verify it passes**

Run:

```bash
cd /home/zn/PX4-Autopilot
make px4_fmu-v6c_default
```

Expected: PASS, with `flap_duty_status` compiled and logged.

**Step 5: Commit**

```bash
git add msg/FlapDutyStatus.msg \
        msg/CMakeLists.txt \
        src/modules/logger/logged_topics.cpp \
        src/modules/rpm_pid/RpmPid.hpp \
        src/modules/rpm_pid/RpmPid.cpp
git commit -m "feat: publish flap duty control status for logging"
```

### Task 5: Final parameter polish and hardware smoke test

**Files:**
- Modify: `src/modules/rpm_pid/rpm_pid_params.c`
- Modify: `src/modules/rpm_pid/RpmPid.cpp`

**Step 1: Tighten the parameter descriptions**

Make the QGC-visible descriptions explicit:
- `FLAP_PHASE_DUTY = 0.5` means symmetric timing
- `FLAP_PHASE_DUTY > 0.5` means longer downstroke / shorter upstroke
- `FLAP_PHASE_DUTY < 0.5` means shorter downstroke / longer upstroke
- `FLAP_PHASE_MODE = 0` means legacy shaping
- `FLAP_PHASE_MODE = 1` means closed-loop duty control

Update the module usage text in `RpmPid.cpp` so a future user knows the new mode requires valid `wing_phase`.

**Step 2: Build and verify the final firmware**

Run:

```bash
cd /home/zn/PX4-Autopilot
make tests TESTFILTER=DutyCycleWarp
make px4_fmu-v6c_default
```

Expected: both commands PASS.

**Step 3: Run the hardware smoke test**

On the target:

```bash
param set FLAP_PHASE_MODE 1
param set FLAP_PHASE_DUTY 0.50
listener wing_phase 5
listener flap_frequency 5
listener flap_duty_status 5
```

Then repeat with:

```bash
param set FLAP_PHASE_DUTY 0.35
param set FLAP_PHASE_DUTY 0.65
```

Acceptance criteria:
- `flap_duty_status.duty_cmd` follows the parameter value
- `flap_duty_status.duty_meas` moves in the same direction
- `u_phase` changes sign between downstroke and upstroke
- `rpm_meas` remains bounded; no runaway saturation
- `FLAP_PHASE_MODE 0` still restores the old behavior

**Step 4: Commit**

```bash
git add src/modules/rpm_pid/rpm_pid_params.c \
        src/modules/rpm_pid/RpmPid.cpp
git commit -m "docs: clarify qgc duty-cycle parameters and smoke-test flow"
```

## Notes for execution

- The branch for this work is `feature/qgc-duty-cycle`, created from `origin/FUSION`.
- Do not rename `FLAP_PHASE_DUTY` in the first implementation; it already gives the user a QGC entry point.
- Keep the first implementation reversible. The legacy mode is not optional; it is the safety fallback for flight bring-up.
- The paper-facing telemetry requirement is non-negotiable: without `duty_cmd`, `duty_meas`, and the split `u_pid` / `u_phase`, the controller will be hard to tune and harder to publish.
