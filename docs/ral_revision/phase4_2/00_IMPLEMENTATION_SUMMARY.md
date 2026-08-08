# Phase 4.2 implementation summary

Phase 4.2 is limited to bench-evidence corrections. It does not change the
quality formula, spectral estimator, four experiment modes, TECS, NPFG, or any
controller.

## Startup identity bootstrap

The Phase 4.1 preflight checker now classifies selector records before the first
complete device/instance identity match as `BOOTSTRAP`. It reports sample count,
duration, first matched timestamp, and post-bootstrap mismatch count. Bootstrap
must finish inside the frozen timeout, and it may not overlap an explicitly
declared formal interval or the first armed timestamp found in the ULog.

After the first match, every physical-Pitot selector record must retain the same
nonzero device, quality device, source instance, and identity-match flag.

## Original PX4 validity semantics

`pre_quality_output_finite` records only whether IAS/CAS/TAS outputs are finite.
For a physical pre-quality source, `original_sensor_valid` records the actual
`AirspeedValidator::get_airspeed_valid()` result. Ground-minus-wind and synthetic
pre-selections are recorded through `original_selection_was_fallback` and
`original_selected_source`; they are not mislabeled as a selected physical
validator failure.

Trigger codes distinguish original sensor invalidity, an original PX4 fallback,
quality rejection, blockage, and source identity mismatch. Original PX4 source
selection and validator behavior are unchanged.

## Fallback outcome

Both quality and blockage paths finalize fallback evidence from the actual final
`airspeed_source`:

| Final source | Outcome |
| --- | --- |
| SENSOR_1/2/3 | ALTERNATE_PHYSICAL |
| GROUND_MINUS_WIND | GROUND_MINUS_WIND |
| SYNTHETIC | SYNTHETIC |
| DISABLED | UNAVAILABLE |

The same helper is used by production code and unit tests, including a
quality-plus-blockage concurrency case.

## Authoritative ULog evidence

`check_airspeed_evidence.py` validates the quality-input grid, bootstrap and
identity, selector/validated timestamp joins, EKF quality/source joins, exact R
against `estimator_aid_src_airspeed`, the four-mode truth table, selector
traceability, and logger continuity. It always writes a JSON result and writes
CSV evidence tables for every analysis reached before a failure.

R is joined by uORB instance and observation `timestamp_sample`; publication
timestamps are retained only as diagnostics.
