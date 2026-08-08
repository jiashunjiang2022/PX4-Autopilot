# Selector fallback

The selector first records the original selected source and whether the original IAS/CAS/TAS tuple is finite. Quality handling is bypassed in BASELINE, CONSTANT_R, and VARIANCE_ONLY.

In FULL_PROPOSED, stale/invalid quality, a closed quality fusion gate, or persistent low spectral quality drives a latched rejection. The latch keeps the existing hold and re-enable dwell behavior. Rejection does not alter the underlying `AirspeedValidator` validity.

Fallback order is:

1. Another valid physical Pitot source.
2. Configured ground-minus-wind fallback when available.
3. Configured synthetic fallback for fixed-wing operation.
4. Original PX4 disabled/no-airspeed behavior when no fallback exists.

`airspeed_selector_quality_status` records original validity, pre-quality source, rejection reason, fallback attempt/availability/source, and final source/validity. Reasons distinguish original PX4 invalid, quality safeguard, sensor timeout, source unavailable, and fallback selected.

Unit tests cover original invalidity, quality rejection, stale/timeout quality, alternate physical/ground-wind/synthetic fallback, unavailable fallback, hold/recovery/re-enable, and explicit physical -> ground-wind -> disabled source transitions.

If the independent blockage detector and quality safeguard are active together, the blockage fallback is applied last and the status topic reports the final outcome. Whether real source switching is free of unacceptable TECS transients must be demonstrated on bench; no TECS or NPFG control law was changed.
