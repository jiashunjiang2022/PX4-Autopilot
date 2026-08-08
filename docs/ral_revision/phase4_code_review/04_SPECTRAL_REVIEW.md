# Spectral and temporal review

## Static result

- Input grid: 50 Hz; requested 4 s nominal window; ring capacity 256.
- Frequency step: `max(0.1, 1/window)` = 0.25 Hz at 4 s.
- Reference band default: 0.5-8 Hz; flap half-width: 0.5 Hz; evaluation interval: 0.5 s.
- Goertzel scans the reference band and explicitly rejects invalid/Nyquist/reference/flap-band/zero-energy/nonfinite cases (`AirspeedQualityMath.cpp:49-120`).
- 6.8 +/- 0.5 Hz lies inside the default band and is directly asserted (`test_AirspeedQuality.cpp:24-35`). An 8 +/- 0.5 Hz band is correctly invalid for `RU=8`; the 8 Hz positive test uses `RU=9` (`test_AirspeedQuality.cpp:38-55`). No 4 Hz clamp or silent flap-band clamp was found.
- Timestamp helper saturates ordering before subtraction and distinguishes duplicate, non-monotonic, and long gap (`AirspeedQualityMath.cpp:12-30`). Spectral update versus held result is exposed by a boolean and counter.

Temporal q uses the same 50 Hz IAS samples. Raw `abs(delta IAS)/dt`, filtered value, normalized value, q raw, and q smoothed are logged. The filters use `alpha=dt/(tau+dt)` (`EKF2.cpp:176-186` and `456-464`), not fixed alpha 0.2. Reset initializes q to 1, avoiding startup false-low q. `temporal_raw` is a magnitude rate indicator, not a signed/pure aerodynamic derivative.

## Findings

| ID / severity | File / function / lines | Observed behavior | Risk | Required correction | Verification |
|---|---|---|---|---|---|
| P1-02 / P1 | `src/modules/ekf2/EKF2.cpp`, `AirspeedQualityEstimator::update`, 330-362; `params_airspeed.yaml`, `EKF2_ASP_SWIN`, 84-93 | A window is declared ready at 95% of samples and 95% of duration: 190 samples and 3.8 s for nominal 4 s/200 samples, while parameter text promises the full window. | Runtime spectral values do not implement the frozen 4 s experiment definition; startup/transient results are not comparable to the 200-sample tests. | Require the specified full sample count/duration with an explicit tolerance defined in the protocol, and make message/document semantics match. | Full-estimator test must prove first evaluation timestamp/count, ring wrap, drops, and reset refill. |

Existing spectral tests exercise tones 2/3/4/5/5.5/6.8 Hz, 8 Hz with valid bounds, invalid edge, slow trend, deterministic broadband noise, zero energy, Nyquist, and timestamp classification. They call the standalone math helper with uniformly indexed samples; they do not exercise the producer/resampler, estimator fill policy, jittered sample values, repeated resets, or integrated q/gate timing. Those gaps are tracked in `P1-06`.
