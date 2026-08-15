# Phase 4 parameter freeze template

Copy `frozen_config.template.json` to a run-specific file and replace every
`null` with the bench-approved selector value. Do not treat the current quality
threshold defaults as experimentally frozen values.

Before each formal run, explicitly set and record:

- `FLAP_RATIO=8.0`; a persisted old value overrides the new firmware default.
- `EKF2_ARSP_THR>0`; zero disables the airspeed fusion path under test.
- `EKF2_LOG_VERBOSE=1`; otherwise `estimator_aid_src_airspeed` evidence is absent.
- `EKF2_ASP_MODE` to exactly one of 0, 1, 2, or 3.
- `EKF2_ASP_RCST` to the frozen variance multiplier, including modes that do not use it.
- `SDLOG_PROFILE` with bit 12 set. Prefer `4097` to combine default and RA-L profiles.

Example after values are frozen:

```sh
param reset FLAP_RATIO
param set FLAP_RATIO 8.0
param set EKF2_ARSP_THR <FROZEN_POSITIVE_THRESHOLD>
param set EKF2_LOG_VERBOSE 1
param set EKF2_ASP_MODE <0|1|2|3>
param set EKF2_ASP_RCST <FROZEN_VARIANCE_MULTIPLIER>
param set SDLOG_PROFILE 4097
param save
reboot
```

Run the offline preflight checker on the resulting bench ULog. A missing device,
topic, rate, frozen value, aid-source publication, or logger dropout is a FAIL.
