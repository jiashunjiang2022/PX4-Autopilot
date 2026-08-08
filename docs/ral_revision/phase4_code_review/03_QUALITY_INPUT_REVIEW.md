# Quality-input review

## Data path

```text
differential_pressure instance 0
  -> SENS_DPRES_OFF calibration
  -> 2-pole low-pass on pressure
  -> interpolation only between two real source timestamps
  -> 20 ms timestamp_sample grid
  -> calc_IAS_corrected()
  -> airspeed_quality_input
  -> EKF2 AirspeedQualityEstimator
```

`Sensors::_diff_pres_sub` is a non-multi subscription at `src/modules/sensors/sensors.hpp:183`. `diff_pres_poll()` consumes `timestamp_sample` and calls the quality producer at `sensors.cpp:389-417`. Publication time is only `output.timestamp`; the uniform grid is `output.timestamp_sample` (`sensors.cpp:365-378`).

The resampler does not synthesize held samples: the `while` loop emits only grid timestamps in the closed interval between the previous and current real samples, using linear interpolation (`sensors.cpp:339-382`). With no new source message there is no output, and there is no future extrapolation. Duplicate, non-monotonic, and gaps over 40 ms reset state (`sensors.cpp:299-328`). NaN/Inf pressure and invalid air data reset. The reset clears filter/timestamp/interpolation state (`sensors.cpp:271-279`). IAS uses the existing `calc_IAS_corrected()` conversion and selected sensor model (`sensors.cpp:346-362`), avoiding a second density formula.

## Findings

| ID / severity | File / function / lines | Observed behavior | Risk | Required correction | Verification |
|---|---|---|---|---|---|
| P0-01 / P0 | `src/modules/sensors/sensors.hpp`, `Sensors` members, 183-210; `sensors.cpp`, `update_airspeed_quality_input`, 281-386; `src/modules/airspeed_selector/airspeed_selector_main.cpp`, `AirspeedModule::Run`, 845-938 | q always derives from instance 0; no previous `device_id` is retained/reset; selector applies that global q to any selected physical source and can choose another unassessed physical source. | A good sensor can be rejected from another sensor's q, or a bad selected/fallback sensor can be trusted. Mixed-device filter history invalidates ULog evidence. | Bind q to the selected physical instance/device, or enforce and verify a single-sensor configuration; reset on `device_id` change; carry device/instance identity through EKF2 and selector status. | Multi-instance test with differing waveforms plus same-instance device switch; ULog must prove q device equals pre-quality/fused device. |
| P1-01 / P1 / NEEDS_BENCH_VERIFICATION | `src/modules/sensors/sensors.hpp`, constants/filter, 200-203; `sensors.cpp`, parameter/filter update, 177-184 and 318-336 | Filter coefficients are always configured for 83.3333 Hz. Measured rate is logged but neither drives coefficients nor rejects an incompatible rate; gaps up to 40 ms are accepted. | Effective cutoff/attenuation varies with producer rate and 50 Hz output can exceed information bandwidth; anti-alias claims are not established. | Measure and enforce a valid source-rate envelope or retune from stable measured rate with explicit reset; specify the rate/cutoff contract. | Inject swept tones at actual rates/jitter and measure 4/5/5.5/6.8 Hz passband and >=25 Hz suppression. |
| P1-03 / P1 / NEEDS_BENCH_VERIFICATION | `src/modules/sensors/sensors.cpp`, `update_airspeed_quality_input`, 284-290 | Every calibrated pressure below 0 Pa causes a complete producer reset. | Normal zero-offset noise at startup/near zero speed can repeatedly suppress the quality stream and trigger FULL-mode stale behavior. | Define a physically justified signed/deadband policy consistent with the normal PX4 conversion; reserve resets for truly invalid data. | Zero-flow and low-speed pressure sweep with calibrated sensor noise; verify continuity, reset count, and no false selector action. |

Conclusion: the topic is a genuinely bracketed uniform time-grid sequence, not a held-sample nominal 50 Hz topic. It is not yet a source-identifiable, rate-qualified flight input.
