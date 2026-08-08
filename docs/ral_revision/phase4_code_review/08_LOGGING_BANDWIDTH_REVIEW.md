# Logging and bandwidth review

The dedicated bit-12 profile records the pressure-to-q-to-EKF-to-selector chain, flap/encoder/phase evidence, wind, TECS/NPFG status, setpoints, attitude/rates, and actuator outputs. It does not enable a generic all-topic high-rate profile. Multi-instance limits are explicit. `differential_pressure` and core quality/kinematic topics have zero logger intervals where all producer publications are requested; all intervals remain logger upper bounds only.

## Static profile-only estimate

| Topic | Bytes | Hz | Instances | Payload B/s |
|---|---:|---:|---:|---:|
| differential_pressure | 32 | 83 | 2 | 5,312 |
| airspeed_quality_input | 44 | 50 | 1 | 2,200 |
| airspeed | 28 | 20 | 1 | 560 |
| airspeed_validated | 41 | 10 | 1 | 410 |
| encoder_count | 24 | 100 | 1 | 2,400 |
| rpm | 16 | 100 | 2 | 3,200 |
| flap_frequency | 12 | 100 | 1 | 1,200 |
| wing_phase | 41 | 100 | 1 | 4,100 |
| ekf2_airspeed_quality | 138 | 50 | 1 | 6,900 |
| estimator_aid_src_airspeed | 59 | 20 | 2 | 2,360 |
| wind | 48 | 20 | 1 | 960 |
| estimator_wind | 48 | 20 | 2 | 1,920 |
| estimator_status_flags | 83 | 20 | 2 | 3,320 |
| airspeed_selector_quality_status | 19 | 10 | 1 | 190 |
| tecs_status | 104 | 20 | 1 | 2,080 |
| fixed_wing_lateral_guidance_status | 37 | 20 | 1 | 740 |
| fixed_wing_lateral_status | 16 | 20 | 1 | 320 |
| fixed_wing_lateral_setpoint | 20 | 50 | 1 | 1,000 |
| fixed_wing_longitudinal_setpoint | 28 | 50 | 1 | 1,400 |
| vehicle_attitude_setpoint | 40 | 50 | 1 | 2,000 |
| vehicle_rates_setpoint | 33 | 50 | 1 | 1,650 |
| vehicle_attitude | 49 | 50 | 1 | 2,450 |
| vehicle_angular_velocity | 40 | 50 | 1 | 2,000 |
| vehicle_torque_setpoint | 28 | 100 | 2 | 5,600 |
| vehicle_thrust_setpoint | 28 | 100 | 2 | 5,600 |
| actuator_motors | 66 | 100 | 1 | 6,600 |
| actuator_servos | 48 | 100 | 1 | 4,800 |

Exact CSV sum: **71,272 B/s payload** = 69.602 KiB/s = 4.276 MB/min = 85.526 MB/20 min. With the artifact's 16 B/sample allowance: **101,768 B/s conservative** = 99.383 KiB/s = 6.106 MB/min = 122.122 MB/20 min.

These totals exclude default-profile traffic, metadata bursts, filesystem overhead, compression behavior, and unexpected extra instances. They are not an SD-card qualification.

## Finding

| ID / severity | File / function / lines | Observed behavior | Risk | Required correction | Verification |
|---|---|---|---|---|---|
| P1-07 / P1 / NEEDS_HARDWARE_VERIFICATION / NEEDS_ULOG_VERIFICATION | `src/modules/logger/logged_topics.cpp`, `LoggedTopics::add_flapping_dataset_topics`, 383-423; `artifacts/ral_revision_phase4/ulog_rate_check/static_bandwidth.csv`, rows 1-28 | Profile is comprehensive, but the estimate excludes default traffic/overhead and no target SD soak or ULog rate audit exists. | Logger dropouts could remove the exact samples needed for q/R/source claims; estimated rates could be reported as measured producer rates. | Qualify the selected SD card/profile under worst target load and preserve observed producer/consumer rates and dropout counters. | >=30 min target soak: zero logger/buffer dropout, continuous timestamps/counters, measured per-topic rates/instances, and file-size/bandwidth within storage margin. |
