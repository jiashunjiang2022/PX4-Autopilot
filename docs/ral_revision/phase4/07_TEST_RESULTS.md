# Test results

Final focused host results:

| Test target | Result | Coverage |
|---|---|---|
| `unit-AirspeedQualityMode` | PASS | four-mode truth table, unknown-mode fail-closed, variance multipliers |
| `unit-AirspeedSelectorQuality` | PASS | original validity, rejection, timeout, fallback, recovery, source transitions |
| `unit-test_AirspeedQuality` | PASS | tones through 6.8 Hz, 8 Hz policy, noise/trend, timestamps, Nyquist, zero energy |
| `unit-AS5600Math` | PASS | RPM to flap frequency at 8:1, invalid ratio |
| `unit-WingPhaseMath` | PASS | counts/cycle, 8:1 wrapping, Hall interpolation boundaries |

Logs:

- `artifacts/ral_revision_phase4/test_logs/phase4-airspeed-final.log`
- `artifacts/ral_revision_phase4/test_logs/mechanical-math-final.log`
- `artifacts/ral_revision_phase4/test_logs/selector-quality-final.log`

The full SITL test build compiled the changed sensors, EKF2, selector, logger, messages, and tests. CTest reports 3/3 airspeed tests, 2/2 mechanical tests, and the final selector rerun passing.

No real flight, HIL, hardware sensor, WCET/stack, or SD soak test was run.
