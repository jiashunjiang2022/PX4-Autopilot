# Test-coverage review

All five supplied targets pass. The assertions, not only names, were inspected.

| Target | Tests | What is asserted | Negative/time cases | Integration level |
|---|---:|---|---|---|
| `unit-AirspeedQualityMode` | 3 | Four switches, unknown fallback, variance multiplication | Unknown mode | Pure helper |
| `unit-AirspeedSelectorQuality` | 8 | Finite original validity, fallback choices, stale/invalid/low q, latch recovery | No fallback, stale q, invalid q | Pure helper; “transitions” are independent calls |
| `unit-test_AirspeedQuality` | 7 | 2/3/4/5/5.5/6.8 Hz, 8 Hz policy, trend/noise, zero/Nyquist, timestamp class | Boundary, zero energy, duplicate/non-monotonic/long gap | Spectral/timestamp helpers only |
| `unit-AS5600Math` | 2 | 8:1 RPM conversion | Invalid ratio | Pure helper |
| `unit-WingPhaseMath` | 7 | Counts/cycle, wrap and Hall interpolation boundaries | Invalid/boundary cases | Pure helper |

No test runs the complete differential-pressure filter/resampler; no full estimator test checks the 95% fill policy, ring wrap, consumer drops, repeated reset, q/gate timing, or parameter-invalid behavior. No test places `noise_var` through the EKF buffer and compares it with aid-source output. No selector module test checks real IAS/CAS/TAS values, status fields, concurrent blockage+quality, source identity, no-fallback publication, or TECS-facing transitions. No startup, HIL/hardware, logger schema/rate, WCET/stack, or SD soak test is supplied.

## Finding

| ID / severity | File / function / lines | Observed behavior | Risk | Required correction | Verification |
|---|---|---|---|---|---|
| P1-06 / P1 | `src/modules/ekf2/test/test_AirspeedQuality.cpp`, tests, 24-115; `src/modules/airspeed_selector/AirspeedSelectorQualityTest.cpp`, tests, 5-87; runtime `EKF2.cpp:2793-2938` and `airspeed_selector_main.cpp:830-957` | Passing tests exercise isolated helpers, not producer-to-EKF-to-selector wiring or stateful runtime transitions. | Wiring, timestamp, source, queue, and published-value defects can pass all five targets; current evidence cannot support formal bench/flight claims. | Add module/integration tests covering all four modes, producer resets/rates, exact EKF R, source/device switch, stale/no fallback, concurrent causes, and startup. | Tests must assert actual topic fields and sample timestamps/values through the runtime modules; target tests then close WCET/logger/hardware gaps. |

Test result: useful algorithm regression coverage, but **insufficient for bench/flight release**.
