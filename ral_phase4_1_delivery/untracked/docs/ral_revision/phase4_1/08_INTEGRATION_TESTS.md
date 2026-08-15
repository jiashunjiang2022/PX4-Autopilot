# Integration tests

The final host run executed seven CTest targets and all passed:

- `unit-AS5600Math`
- `unit-WingPhaseMath`
- `unit-AirspeedQualityMode`
- `unit-AirspeedSelectorQuality`
- `unit-test_AirspeedQuality`
- `unit-AirspeedQualityInput`
- `unit-test_EKF_airspeed`

Coverage includes the four-mode truth table and unknown-mode baseline fallback;
source/device/instance identity; signed pressure and range faults; real-sample
bracketing; source-rate validity/reconfiguration; strict 200-point window and
ring wrap; quality latch/fallback choices; exact diagnostic sample fields; and
multiple noise variances propagated through the EKF buffer to the airspeed aid
source.

Final result: 7/7 targets passed, 0 failures. Log:
`artifacts/ral_revision_phase4_1/test_logs/relevant-tests-final.log`.

Runtime uORB transition matrices, sensor reconnect timing, and real logger joins
remain bench tests; host tests do not claim hardware equivalence.
