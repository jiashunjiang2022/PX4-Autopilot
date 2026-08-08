# Sample-aligned variance

`EstimatorInterface::setAirspeedData()` returns whether the observation was
actually pushed to the EKF buffer. `ekf2_airspeed_quality` is published only on
that successful return, so rate limiting or an uninitialized buffer cannot
produce an authoritative record for an unqueued sample.

Both validated and raw input paths call the same queue/publish function. The
validated path joins `airspeed_validated` with
`airspeed_selector_quality_status` by publication timestamp to recover the final
physical device and sample timestamp without changing the versioned validated
message. The raw path records its own `timestamp_sample` and `device_id`.

The diagnostic's `r_as_used` is copied from `airspeedSample.noise_var`; its
`fuse_enabled` is copied from `airspeedSample.fuse_enabled`. EKF buffer delay
changes the internal fusion time only. The diagnostic retains the original
observation timestamp, quality timestamp, quality age, source/device identities,
EAS2TAS, q snapshot, nominal variance, mode, and enabled-path flags.

Parameterized tests cover changing q, EAS2TAS, nominal noise, and all four modes.
The EKF integration test verifies multiple queued variances reach
`aid_src_airspeed().observation_variance` exactly.
