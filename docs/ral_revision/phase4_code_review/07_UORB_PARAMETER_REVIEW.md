# uORB and parameter review

## uORB

The three review topics are registered and build-generated. Structs are zero-initialized at publication sites. `airspeed_quality_input` distinguishes publication `timestamp` from uniform-grid `timestamp_sample`, carries pressure units, IAS units, device ID, source enum, validity, measured/configured rates, gap count, and reset reason. `ekf2_airspeed_quality` carries estimator sample time, mode/effective switches, spectral update/hold evidence, q internals, and R fields. Selector status carries source/result fields but has the traceability defect in `P1-04`.

No explicit queue-length directive is present, so normal uORB latest-sample behavior applies. This is acceptable only if EKF2 always services the 50 Hz topic before overwrite. That has not been shown under the new spectral load; missed grid samples would leave a nonuniform sample array for a Goertzel implementation that assumes uniform indexing. This is part of `P1-05`/`P1-06` and requires target evidence.

`input_rate_hz` in `ekf2_airspeed_quality` is assigned `quality_input.output_rate_hz` (`EKF2.cpp:2815-2817`), i.e. configured grid rate, not measured producer or observed consumer rate. Logger intervals are caps, not producer rates.

## Parameters

- `EKF2_ASP_MODE` defaults to BASELINE, range 0-3, reboot-required. Unknown runtime values fail to BASELINE.
- `EKF2_ASP_RCST` is reboot-required and documented as a variance multiplier.
- Default `EKF2_ASP_RU=8.0`; no online 7.5 value was found.
- `FLAP_RATIO` defaults/fallbacks to 8.0 in AS5600, wing phase, and EKF2 effective-ratio handling.
- `SFS=50`, `SWIN<=5` imply at most 250 nominal samples, below the 256 ring capacity.
- Invalid spectral reference/Nyquist/flap-band combinations return an explicit invalid reason; no silent spectral-band clamp was found.
- Mode/RCST are frozen by construction; most threshold parameters remain updateable through normal ModuleParams behavior. Formal experiments must freeze and record the whole parameter set.

## Finding

| ID / severity | File / function / lines | Observed behavior | Risk | Required correction | Verification |
|---|---|---|---|---|---|
| P2-01 / P2 | `src/modules/ekf2/params_airspeed.yaml`, `EKF2_ASP_SFS`, 73-83; `src/modules/ekf2/EKF2.cpp`, `UpdateAirspeedSample`, 2803-2808 | `EKF2_ASP_SFS` is declared as expected spectral rate but the online estimator receives `quality_input.output_rate_hz`; the parameter is unused. Its text also says flap center is clamped below Nyquist, while math explicitly invalidates out-of-range bands. | Frozen configurations can imply a control that does not exist and reviewers can misinterpret boundary behavior. | Remove the unused parameter or use it for an explicit rate-consistency check; correct description to the actual invalidation policy. | Parameter/schema test must show each exposed parameter changes or validates the stated behavior. |

No ABI/padding failure was observed in the successful target build, but schema compatibility and logger decoding still require a real ULog (`NEEDS_ULOG_VERIFICATION`).
