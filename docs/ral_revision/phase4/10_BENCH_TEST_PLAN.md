# Bench test plan

## 1. Parameter and device gate

1. Copy and freeze the JSON parameter template for one experiment mode.
2. Reboot, start MS4525DO/AS5600/Hall producers, and capture a disarmed ULog.
3. Run `preflight_check.py` with explicit expected mode and RCST. Any FAIL blocks further testing.

## 2. Producer and spectral validation

1. Drive pressure signals at 2, 3, 4, 5, 5.5, 6.8, and 8 Hz plus broadband noise and slow trend.
2. Verify physical source rate >=70 Hz, quality grid 50 Hz, no duplicates/non-monotonic times, and the documented gap resets.
3. Compare online spectral ratio, invalid reason, temporal terms, q, nominal R, and used R with offline reconstruction.
4. Confirm 6.8 Hz is valid and the default 8 Hz edge case is explicitly invalid.

## 3. Mode isolation

Run identical input playback after a reboot in modes 0--3. Confirm estimator diagnostics match, BASELINE uses nominal R, CONSTANT_R uses the frozen variance multiplier, VARIANCE_ONLY changes only R, and FULL_PROPOSED alone changes gate/selector behavior.

## 4. Selector transitions

Inject original PX4 invalidity, stale quality, low quality, source timeout, alternate Pitot availability, ground-wind fallback, synthetic fallback, and no fallback. Measure final source, status reason, TECS input continuity, throttle/pitch response, hold, and recovery dwell. Repeat with blockage active.

## 5. Runtime and logging soak

1. Measure EKF2 work-queue WCET/missed deadlines and stack high-water during worst-case 200-sample spectral evaluation.
2. Run at least 30 minutes with the final SD card and exact logger profile.
3. Run `check_ulog_rates.py`; require all P0 topics/rates, monotonic timestamps, acceptable max gaps, and zero logger lost messages.
4. Archive firmware hash, parameter snapshot, checker CSV, ULog, SD card identity, and results.

Only after every bench gate passes may status advance to `READY_FOR_BENCH`; Phase 4 itself never authorizes flight.
