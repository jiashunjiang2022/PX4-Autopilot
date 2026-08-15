# Four-mode path review

## Control flow

```text
EKF2_ASP_MODE (captured at EKF2/selector construction)
  -> airspeed_quality::mode_config()
     -> adaptive_r_enabled ------> observation_variance()
     -> quality_fusion_gate_enabled -> airspeedSample.fuse_enabled
     -> selector_quality_enabled ---> selector latch/fallback
  -> unknown value: BASELINE config + EKF2 error log
```

The single derivation point is `airspeed_quality::mode_config()` at `src/lib/airspeed/AirspeedQualityMode.hpp:26-43`. No older independent quality-enable parameter was found. `EKF2_ASP_MODE` and `EKF2_ASP_RCST` are reboot-required and are captured in `EKF2::EKF2()` at `src/modules/ekf2/EKF2.cpp:839-840`; the selector captures mode at `src/modules/airspeed_selector/airspeed_selector_main.cpp:272-274`. Armed parameter updates therefore do not change these cached mode values.

## Truth table

| Mode | Estimator/logging | EKF variance | Quality fusion gate | Selector quality | PX4 validity/fallback |
|---|---|---|---|---|---|
| BASELINE | Runs | nominal `R0` | Bypassed (`true`) | Bypassed | Preserved |
| CONSTANT_R | Runs | `R0 * RCST` | Bypassed (`true`) | Bypassed | Preserved |
| VARIANCE_ONLY | Runs | `R0 * f(q)` | Bypassed (`true`) | Bypassed | Preserved |
| FULL_PROPOSED | Runs | `R0 * f(q)` | Persistent q/freshness gate | Latch plus explicit fallback | Original validity recorded independently |

Evidence: variance selection is `AirspeedQualityMode.hpp:46-61`; quality publication and mode flags are `EKF2.cpp:2739-2833`; the sample gate is `EKF2.cpp:2883-2904` and `2916-2937`; selector bypass/FULL branch is `airspeed_selector_main.cpp:830-955`. CONSTANT_R multiplies variance, not standard deviation. Unknown mode returns BASELINE switches and is reported at `EKF2.cpp:2731-2737`.

BASELINE is functionally equivalent at the airspeed variance/gate/selector decisions, but not timing- or resource-identical: the producer, quality estimator, extra publications, and logger metadata still execute. This distinction must be retained in experiment claims.

Mode isolation helper tests cover the truth table and variance arithmetic (`AirspeedQualityModeTest.cpp:7-51`). There is no integrated test that runs all four modes through producer, EKF buffer, aid source, selector, and published messages (`P1-06`).
