# Mode3 Split-Cycle Logging and Metadata Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Improve `FUSION_new_modulation` `mode3` usability by logging explicit split-cycle waveform data, clarifying `Δ=T_down/T` parameter descriptions, and increasing ULog cadence for the core flap topics.

**Architecture:** Keep the existing `FLAP_FM_MODE=3` control law unchanged. Extend `FlapControlStatus` with waveform-centric telemetry, publish those fields from `RpmPid.cpp`, rewrite the mode3 parameter descriptions around `Δ`, and raise logger cadence for `flap_control_status`, `flap_motor_setpoint`, and `wing_phase`.

**Tech Stack:** PX4 C++ modules, uORB message generation, PX4 parameter metadata, PX4 logger topic configuration, FMU-v6c firmware build.

---

### Task 1: Extend the flap status topic with waveform fields

**Files:**
- Modify: `msg/FlapControlStatus.msg`

**Step 1: Add the new message fields**

Update `msg/FlapControlStatus.msg` to add:

```text
float32 f_up_cmd_hz
float32 f_down_cmd_hz
bool in_downstroke
float32 wave_cmd
float32 wave_meas
```

Place them near the existing split-cycle fields so the topic stays readable.

**Step 2: Run a firmware build to verify it fails if publishers are not updated yet**

Run:

```bash
cd /home/honor/PX4-Autopilot
make px4_fmu-v6c_default
```

Expected: FAIL because `RpmPid.cpp` still publishes the old `flap_control_status_s` layout.

**Step 3: Commit the message-only change locally**

Do not commit to git yet; continue directly to the publisher update.

### Task 2: Publish waveform telemetry from `rpm_pid`

**Files:**
- Modify: `src/modules/rpm_pid/RpmPid.cpp`

**Step 1: Add the new local state variables**

In `RpmPid::Run()`, add local values initialized to `NAN` / `false`:

```cpp
float f_up_cmd_hz = NAN;
float f_down_cmd_hz = NAN;
bool in_downstroke = false;
float wave_cmd = NAN;
float wave_meas = NAN;
```

**Step 2: Compute the values in the existing `mode3` branch**

Within the `_fm_mode == 3` path:

```cpp
f_down_cmd_hz = f_eff_sp / (2.f * _sc_delta_applied);
f_up_cmd_hz = f_eff_sp / (2.f * (1.f - _sc_delta_applied));
in_downstroke = (phase_deg_used > 90.f && phase_deg_used < 270.f);
wave_meas = cosf(phase_deg_used * (M_PI_F / 180.f));
wave_cmd = PX4_ISFINITE(_phase_ref_deg) ? cosf(_phase_ref_deg * (M_PI_F / 180.f)) : NAN;
```

Reuse the same phase alignment and half-cycle conventions already used by the controller.

**Step 3: Populate the new `flap_control_status_s` fields**

When filling `control_status`, assign:

```cpp
control_status.f_up_cmd_hz = f_up_cmd_hz;
control_status.f_down_cmd_hz = f_down_cmd_hz;
control_status.in_downstroke = in_downstroke;
control_status.wave_cmd = wave_cmd;
control_status.wave_meas = wave_meas;
```

**Step 4: Run the firmware build to verify it passes**

Run:

```bash
cd /home/honor/PX4-Autopilot
make px4_fmu-v6c_default
```

Expected: PASS with regenerated uORB headers and the updated publisher.

### Task 3: Rewrite `mode3` parameter descriptions around `Δ`

**Files:**
- Modify: `src/modules/rpm_pid/rpm_pid_params.c`

**Step 1: Update `FLAP_FM_MODE` description**

Rewrite the `mode=3` entry so it says:

```text
3: split-cycle frequency modulation by downstroke ratio Δ = T_down / T
```

**Step 2: Update `FLAP_SC_DELTA` description**

Make the description explicitly state:

```text
0.5 = symmetric
>0.5 = longer downstroke / shorter upstroke
<0.5 = shorter downstroke / longer upstroke
effective cycle period stays constant
```

**Step 3: Update the engineering-protection parameter descriptions**

Clarify:

- `FLAP_SC_SLEW` limits commanded `Δ` change rate
- `FLAP_SC_BLEND` smooths reversal transitions without redefining `Δ`
- `FLAP_SC_PH_K` is phase-error correction and should be `0` for ideal open-loop split-cycle behavior
- `FLAP_SC_ILIM_A`, `FLAP_SC_REC_TAU`, and `FLAP_SC_U_SLEW` are engineering protection parameters

**Step 4: Run the firmware build to verify metadata changes compile**

Run:

```bash
cd /home/honor/PX4-Autopilot
make px4_fmu-v6c_default
```

Expected: PASS.

### Task 4: Increase flap-topic logging cadence

**Files:**
- Modify: `src/modules/logger/logged_topics.cpp`

**Step 1: Update the flap topic intervals**

Change:

```cpp
add_topic("flap_motor_setpoint", 20);
add_topic("flap_control_status", 20);
add_topic("wing_phase", 20);
```

to:

```cpp
add_topic("flap_motor_setpoint", 10);
add_topic("flap_control_status", 10);
add_topic("wing_phase", 10);
```

Leave `flap_frequency` unchanged in this task.

**Step 2: Run the firmware build to verify the logger configuration compiles**

Run:

```bash
cd /home/honor/PX4-Autopilot
make px4_fmu-v6c_default
```

Expected: PASS.

### Task 5: Verify runtime telemetry on hardware

**Files:**
- No source changes required

**Step 1: Set the core split-cycle parameters**

On the target:

```bash
param set FLAP_FM_MODE 3
param set FLAP_SC_DELTA 0.50
param set FLAP_SC_BLEND 0
param set FLAP_SC_PH_K 0
```

Use the current hardware-safe defaults for other protection parameters.

**Step 2: Inspect the live topics**

Run:

```bash
listener wing_phase 5
listener flap_frequency 5
listener flap_motor_setpoint 5
listener flap_control_status 5
```

Verify that the new fields appear on `flap_control_status`.

**Step 3: Sweep `Δ`**

Run:

```bash
param set FLAP_SC_DELTA 0.35
param set FLAP_SC_DELTA 0.65
```

Expected:

- `delta_cmd` changes accordingly,
- `f_up_cmd_hz` / `f_down_cmd_hz` move in opposite directions,
- `wave_cmd` and `wave_meas` remain finite when phase is valid,
- `delta_meas` moves in the same direction as the command.

**Step 4: Review the ULog**

Open the resulting log and confirm:

- `flap_control_status.wave_cmd`
- `flap_control_status.wave_meas`
- `flap_control_status.f_up_cmd_hz`
- `flap_control_status.f_down_cmd_hz`
- `flap_control_status.in_downstroke`

are present at the increased logging cadence.

### Task 6: Final verification and handoff

**Files:**
- Modify only if hardware findings require small follow-up corrections

**Step 1: Run the final build one last time**

Run:

```bash
cd /home/honor/PX4-Autopilot
make px4_fmu-v6c_default
```

Expected: PASS.

**Step 2: Review the diff**

Run:

```bash
cd /home/honor/PX4-Autopilot
git diff -- msg/FlapControlStatus.msg \
          src/modules/rpm_pid/RpmPid.cpp \
          src/modules/rpm_pid/rpm_pid_params.c \
          src/modules/logger/logged_topics.cpp
```

Expected: only the planned logging and metadata changes appear.

**Step 3: Prepare the handoff**

Summarize:

- the added waveform fields,
- the clarified `Δ` parameter semantics,
- the new log cadences,
- any hardware observations from the `Δ=0.50/0.35/0.65` sweep.
