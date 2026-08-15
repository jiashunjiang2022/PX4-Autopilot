# Phase 4.1 ULog schema

The authoritative variance record is `ekf2_airspeed_quality`, published only
after the corresponding `airspeedSample` is accepted into the EKF observation
buffer. Its expected rate therefore follows accepted airspeed observations,
not the 50 Hz quality producer. `airspeed_quality_input` remains the uniform
50 Hz producer record.

Identity joins:

- `airspeed_quality_input`: `timestamp_sample`, `source_instance`, `device_id`.
- `ekf2_airspeed_quality`: airspeed and quality timestamps, sources, devices,
  `eas2tas`, `nominal_r_as`, exact `r_as_used`, and exact `fuse_enabled`.
- `airspeed_selector_quality_status`: pre-quality, quality, and final identities,
  trigger reason, concurrent conditions, and fallback outcome.

The JSON files in this directory are copied from generated uORB metadata after
the final firmware build. Bench ULog validation remains mandatory.
