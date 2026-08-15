# Phase 4.1 fix summary

Repository boundary: branch `air`, HEAD
`650836c48937f654a0a81fa894b381ff5b130cc2`. Existing uncommitted Phase 4
work was preserved. No checkout, reset, clean, merge, rebase, network access,
dependency installation, hardware connection, or flight was performed.

P0-01 is closed at code-review level by binding the quality stream to DP uORB
instance, nonzero device ID, sample timestamp, producer validity, EKF
observation identity, and the selector's strict single-Pitot proof. Any identity
mismatch disables quality effects and preserves the original PX4 selection.

P0-02 is closed at code-review level by publishing the authoritative diagnostic
only after `setAirspeedData()` accepts the exact `airspeedSample`. The diagnostic
copies `noise_var`, `fuse_enabled`, EAS2TAS, source/device, observation timestamp,
and quality timestamp directly from that accepted observation and its snapshot.

Finite signed negative differential pressure is retained, the default spectral
window requires exactly 200 samples at 50 Hz, and the per-update spectral stack
copies were removed. Final status is `READY_FOR_BENCH_REVIEW`, never flight ready.
