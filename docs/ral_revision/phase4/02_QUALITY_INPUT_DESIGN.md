# Quality input design

The estimator has one input contract:

`differential_pressure` -> offset calibration -> 2-pole anti-alias LPF -> uniform 50 Hz interpolation -> PX4 `calc_IAS_corrected()` -> `airspeed_quality_input` -> EKF2 quality estimator.

Key properties:

- Source time is `differential_pressure.timestamp_sample`.
- `SENS_DP_QCUT` defaults to 10 Hz and is constrained to 7--15 Hz. A disarmed parameter change resets the filter/resampler.
- The output interval is exactly 20,000 us. Output is produced only when two real source samples bracket a grid time; no held samples are synthesized.
- The reported `measured_source_rate_hz` is a 1 s time-constant estimate and is reset after discontinuities.
- IAS conversion uses the existing PX4 compensation model, sensor model selection, tube geometry, barometric pressure, and temperature.
- The existing `airspeed` and `airspeed_validated` producers retain their rates and meanings. They are not fed into the spectral ring.
- The normal EKF airspeed observation remains the original PX4 raw/validated sample. The new topic changes only quality-derived R/gating according to experiment mode.

Fail behavior:

- Duplicate, non-monotonic, >40 ms gap, invalid/negative calibrated pressure, invalid air data, and parameter changes reset interpolation state.
- Reset reason and cumulative `gap_count` are published on the next valid bracketed output.
- An invalid IAS conversion does not produce a sample.

Interpolation provides a deterministic processing grid, not additional physical bandwidth. Bench data must demonstrate the MS4525DO producer rate and validate the 10 Hz cutoff before formal experiments.
