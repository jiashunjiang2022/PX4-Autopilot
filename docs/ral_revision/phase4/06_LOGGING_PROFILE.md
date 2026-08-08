# Logging profile

Set bit 12 in `SDLOG_PROFILE`; `4097` combines the default profile and RA-L profile. Logger intervals are upper limits only and do not change producer rates.

The profile records the complete evidence chain: differential pressure, uniform quality input, normal airspeed topics, quality state, airspeed aid source, selected and multi-instance EKF wind, selector outcome, encoder/RPM/flap/phase, TECS and lateral guidance/status/setpoints, attitude/rate setpoints and states, torque/thrust setpoints, and motor/servo outputs.

Configured maxima include 2 differential-pressure instances, 2 airspeed aid-source instances, 2 estimator-wind instances, 4 airspeed-wind instances, 2 RPM instances, and 2 torque/thrust setpoint instances. `device_id` and message `timestamp_sample` fields are preserved by logging the original messages.

The generated uORB metadata gives a profile-only payload estimate of 69.6 KiB/s. Adding a conservative 16 B/sample framing allowance gives 99.4 KiB/s. See `artifacts/ral_revision_phase4/ulog_rate_check/static_bandwidth.csv`.

This estimate excludes default-profile traffic, metadata bursts, filesystem overhead, and unusual multi-instance activation. A target SD soak with zero ULog dropout messages, zero logger buffer dropouts, and zero logger message gaps is mandatory.
