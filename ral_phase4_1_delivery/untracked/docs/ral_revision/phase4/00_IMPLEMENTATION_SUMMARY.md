# Phase 4 implementation summary

Repository state audited and patched:

- Branch: `air`
- Base HEAD: `650836c48937f654a0a81fa894b381ff5b130cc2`
- Target: `px4_fmu-v6c_default`
- Mechanical flap ratio: 8.0
- Maximum required flap frequency: 6.8 Hz

Implemented outcomes:

1. `FLAP_RATIO` defaults, fallback behavior, startup reporting, and tests use 8.0.
2. A single `airspeed_quality_input` path derives IAS from calibrated differential pressure after a 10 Hz anti-alias LPF and interpolation onto a uniform 50 Hz grid. It does not publish held samples.
3. The 4 s/200-sample Goertzel estimator has parameterized 0.5--8 Hz reference bounds, explicit invalid reasons, and 6.8 Hz coverage.
4. `EKF2_ASP_MODE` implements BASELINE, CONSTANT_R, VARIANCE_ONLY, and FULL_PROPOSED from one truth table. Mode and `EKF2_ASP_RCST` are reboot-required and frozen by EKF2/selector at startup.
5. Only FULL_PROPOSED enables persistent quality fusion gating and selector quality fallback. Original PX4 validity remains independently logged.
6. RA-L logging now covers the pressure-to-actuator evidence chain, including EKF multi-instance wind data.
7. Offline parameter/device/profile and ULog timing/dropout tools are supplied.

Verification completed: five focused host tests pass; uORB generation passes in host and target builds; `px4_fmu-v6c_default` builds at 93.00% FLASH. No real flight was run.

Current status: `READY_FOR_CODE_REVIEW`. Bench producer rate, WCET/stack, hardware device presence, source-transition behavior, and SD soak evidence remain required.
