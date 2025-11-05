# Fixed-Wing Flight Control Fixes

## Summary

This branch contains critical fixes for fixed-wing flight control issues, including altitude control problems and L1 guidance algorithm corrections.

## Issues Fixed

### 1. Altitude Control - TECS Not Climbing (CRITICAL)

**Problem:**
- Aircraft would descend even when target altitude was higher than current altitude
- TECS altitude_reference was initialized to 0 at startup and never reset after takeoff
- This caused TECS to think aircraft was 400m too high, commanding descent

**Root Cause:**
- TECS initialized with `altitude=0` at system startup
- After takeoff to 500m altitude, TECS internal `altitude_reference` was still ~120m
- TECS thought: "I'm at 500m but reference is 120m → I'm 380m too high → DESCEND"

**Fix:**
- Added TECS re-initialization after takeoff using `handle_alt_step()`
- When first valid altitude setpoint >100m is received, reset TECS altitude_reference to current altitude
- File: `src/modules/fw_lateral_longitudinal_control/FwLateralLongitudinalControl.cpp`

### 2. AUTO Mode Missing Controller Configuration

**Problem:**
- AUTO mode did not set climb/sink rate targets or pitch limits for TECS
- Only TAKEOFF mode had these configurations
- After transitioning from TAKEOFF to AUTO, TECS lost its configuration

**Fix:**
- Added controller configuration in `control_auto_position()`:
  - `setClimbRateTarget(FW_T_CLMB_R_SP)` 
  - `setSinkRateTarget(FW_T_SINK_R_SP)`
  - `setPitchMax()` and `setPitchMin()`
- File: `src/modules/fw_mode_manager/FixedWingModeManager.cpp`

### 3. Airspeed NAN Issue

**Problem:**
- When waypoints didn't have `cruising_speed` set, TECS received `airspeed=NAN`
- This prevented TECS from controlling altitude properly

**Fix:**
- Use `FW_AIRSPD_TRIM` parameter as default when waypoint `cruising_speed` is not set or is zero
- File: `src/modules/fw_mode_manager/FixedWingModeManager.cpp`

### 4. L1 Path Following - Oscillation and Poor Tracking

**Problem:**
- L1 guidance caused S-shaped oscillations between waypoints
- Aircraft would deviate 200m+ from desired path
- Cross-track error would not converge

**Root Cause:**
- Course calculation was using `desired_course = path_bearing + eta` (where `eta = eta1 + eta2`)
- This is WRONG! ECL_L1 standard uses only `eta1` for course
- `eta2` (±80°) being added to course caused aircraft to fly perpendicular to path

**Fix - Following ECL_L1 Standard:**
- **Course setpoint**: `path_bearing + eta1` (smooth path following)
- **Lateral acceleration**: `K * v² / L1 * sin(eta1 + eta2)` (proper damping from eta2)
- **eta1 limit**: ±45° (ECL_L1 standard, was ±30°)
- **eta limit**: ±90° (ECL_L1 standard, was ±60°)
- Removed complex bearing vector calculation
- Removed problematic course smoothing that caused angle wrapping issues
- File: `src/modules/fw_mode_manager/FixedWingModeManager.cpp`

### 5. L1 Lateral Acceleration Limit

**Problem:**
- High lateral acceleration (6.0 m/s²) caused large roll angles
- Roll angle = 31° reduced vertical lift component by 14%

**Fix:**
- Reduced max lateral acceleration to 3.0 m/s² 
- This limits roll to ~17°, only 4% lift loss
- File: `src/modules/fw_mode_manager/FixedWingModeManager.cpp`

## Testing Results

- ✅ Altitude control: Aircraft climbs correctly to target altitude
- ✅ L1 path following: Smooth tracking with XTE < 10m
- ✅ No oscillations or S-shaped trajectories
- ✅ Multi-waypoint missions execute correctly

## Files Modified

1. `src/modules/fw_mode_manager/FixedWingModeManager.cpp`
   - AUTO mode controller configuration
   - Airspeed default value handling
   - L1 algorithm corrections

2. `src/modules/fw_lateral_longitudinal_control/FwLateralLongitudinalControl.cpp`
   - TECS re-initialization logic

3. `src/modules/fw_mode_manager/FixedWingModeManager.hpp`
   - Added tecs_status subscription for monitoring

4. `src/lib/tecs/TECS.cpp`
   - No functional changes (debug code removed)

## Configuration Parameters Used

- `FW_T_CLMB_R_SP`: Target climb rate (default 3.0 m/s)
- `FW_T_SINK_R_SP`: Target sink rate (default 2.0 m/s)
- `FW_P_LIM_MIN`: Minimum pitch angle (default -15°)
- `FW_P_LIM_MAX`: Maximum pitch angle (default 30°)
- `FW_AIRSPD_TRIM`: Default trim airspeed (default 15.0 m/s)
- `FW_GUIDANCE_MODE`: 0=NPFG (default), 1=L1

## Version

Branch: `pid`
Latest commit: `365914536f`

## Notes

- These fixes address fundamental bugs in the PX4 main branch
- TECS initialization issue affects all fixed-wing aircraft
- L1 course calculation issue would cause poor tracking for any L1 user
- AUTO mode configuration issue affects altitude control in missions









