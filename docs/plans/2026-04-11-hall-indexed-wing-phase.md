# Hall-Indexed Wing Phase Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Restore Hall-indexed `wing_phase` so the most recent Hall pulse defines `0 deg` and AS5600 provides continuous phase between Hall events.

**Architecture:** `rpm_capture` publishes `hall_event`, `as5600` publishes `encoder_count` and `flap_frequency`, and a dedicated `wing_phase` module combines them. `wing_phase` is invalid until the first Hall event is latched.

**Tech Stack:** PX4 uORB, PX4 modules, NuttX board configs, PX4 unit gtest

---

### Task 1: Add a focused phase math test

**Files:**
- Create: `src/modules/wing_phase/WingPhaseMath.hpp`
- Create: `src/modules/wing_phase/WingPhaseMath.cpp`
- Create: `src/modules/wing_phase/WingPhaseMathTest.cpp`
- Create: `src/modules/wing_phase/CMakeLists.txt`

**Step 1: Write the failing test**
- Cover Hall zero latching, wrapped phase in `[0, 2*pi)`, and invalid/no-Hall cases.

**Step 2: Run test to verify it fails**
- Run: `make tests TESTFILTER=WingPhaseMath`

**Step 3: Write minimal implementation**
- Add a tiny helper that computes Hall-indexed phase from encoder counts.

**Step 4: Run test to verify it passes**
- Run: `make tests TESTFILTER=WingPhaseMath`

### Task 2: Restore Hall event publication

**Files:**
- Create: `msg/HallEvent.msg`
- Modify: `msg/CMakeLists.txt`
- Modify: `src/drivers/rpm_capture/RPMCapture.hpp`
- Modify: `src/drivers/rpm_capture/RPMCapture.cpp`

**Step 1: Publish latched Hall pulse count**
- Increment pulse count on each RPM capture interrupt handling cycle.
- Publish `hall_event` alongside RPM output.

### Task 3: Restore the dedicated wing_phase module

**Files:**
- Create: `src/modules/wing_phase/Kconfig`
- Create: `src/modules/wing_phase/WingPhase.cpp`

**Step 1: Subscribe to `encoder_count`, `flap_frequency`, and `hall_event`**

**Step 2: Latch zero offset on new Hall pulse**

**Step 3: Publish current `wing_phase` topic fields**
- Use current `WingPhase.msg` fields rather than the older message layout.
- Set `phase_valid=false` before first Hall lock.

### Task 4: Remove direct AS5600 wing_phase publication

**Files:**
- Modify: `src/drivers/encoder/as5600/AS5600.hpp`
- Modify: `src/drivers/encoder/as5600/AS5600.cpp`

**Step 1: Keep encoder/rpm/flap-frequency publication**

**Step 2: Remove direct `wing_phase` publication from AS5600**

### Task 5: Enable and start the restored path on target boards

**Files:**
- Modify: `boards/px4/fmu-v6c/default.px4board`
- Modify: `boards/px4/fmu-v6c/init/rc.board_sensors`
- Modify: `boards/cuav/7-nano/default.px4board`
- Modify: `boards/cuav/7-nano/init/rc.board_sensors`

**Step 1: Enable `CONFIG_DRIVERS_RPM_CAPTURE=y` and `CONFIG_MODULES_WING_PHASE=y`**

**Step 2: Start `wing_phase` where AS5600 is started**
- `rpm_capture` remains controlled by `RPM_CAP_ENABLE` in common `rcS`.

### Task 6: Verify builds

**Files:**
- Modify: none

**Step 1: Run host unit test**
- Run: `make tests TESTFILTER=WingPhaseMath`

**Step 2: Run hardware builds**
- Run: `make px4_fmu-v6c_default`
- Run: `make cuav_7-nano_default`

**Step 3: Verify generated uORB artifacts**
- Check `build/<target>/uORB/topics/hall_event.h`
- Check `build/<target>/uORB/topics/wing_phase.h`
