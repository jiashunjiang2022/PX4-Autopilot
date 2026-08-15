# EKF variance-path review

## Connected path

```text
q
 -> observation_variance(R0, q, RMAX/RCST, mode)
 -> airspeedSample.noise_var
 -> EstimatorInterface::setAirspeedData() ring buffer
 -> Ekf::updateAirspeed()
 -> aid_src.observation_variance
 -> generated innovation variance
 -> existing measurement fusion
```

`R0 = (EKF2_EAS_NOISE * EAS2TAS)^2`, in `(m/s)^2` (`EKF2.cpp:2883-2900`). `RMAX` and `RCST` are variance multipliers (`AirspeedQualityMode.hpp:46-61`). The buffer copies the complete sample, changing only its time for configured delay (`estimator_interface.cpp:250-278`). `Ekf::updateAirspeed()` accepts finite positive `noise_var`, otherwise falls back to nominal, and passes R to the generated innovation calculation and aid-source status (`airspeed_fusion.cpp:169-189`). No later override was found. FULL's quality gate is separate from the normal innovation rejection (`airspeed_fusion.cpp:90-125,169-195`); other modes force the quality gate open.

The logged `estimator_aid_src_airspeed.observation_variance` is the strongest available evidence of the variance actually used after buffer delay. It does not expose a direct Kalman gain; influence must be inferred from observation/innovation variance, rejection, and fused status.

## Finding

| ID / severity | File / function / lines | Observed behavior | Risk | Required correction | Verification |
|---|---|---|---|---|---|
| P0-02 / P0 / NEEDS_ULOG_VERIFICATION | `src/modules/ekf2/EKF2.cpp`, `EKF2::UpdateAirspeedSample`/`publish_quality`, 2739-2833 and 2865-2938; `src/modules/ekf2/EKF/aid_sources/airspeed/airspeed_fusion.cpp`, `Ekf::updateAirspeed`, 169-189 | The quality message is published while R uses the previous `_airspeed_quality_eas2tas`. A later raw/validated sample recomputes R using current CAS/TAS and queues it, but no quality message is published for that sample and quality `timestamp_sample` remains the quality-input time. Thus recorded `nominal_r_as`/`r_as_used` may differ from the actual queued/fused sample. | The ULog cannot establish exact per-observation R, invalidating a core experiment claim; the task mandates P0 for any possible recorded/actual mismatch. | Publish the exact used R with the airspeed sample timestamp/device/source, or extend sample-aligned aid-source evidence so the mapping is unambiguous. | Integration test with changing EAS2TAS/q must assert exact equality between queued `noise_var`, aid-source observation variance, and the sample-aligned diagnostic record. |

Conclusion: the adaptive variance is genuinely connected to the EKF computation, but the dedicated diagnostic R fields are not authoritative per fusion sample.
