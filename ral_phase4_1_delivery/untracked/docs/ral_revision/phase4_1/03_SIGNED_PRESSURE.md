# Signed differential pressure

Finite negative calibrated differential pressure no longer causes a reset.
`SENS_DP_QMAX` defaults to 7000 Pa and applies an absolute magnitude limit, so
finite values in `[-limit, +limit]` remain valid while NaN, Inf, and over-range
values reset the producer.

The producer uses PX4's existing `calc_IAS_corrected()` convention. The current
implementation returns a signed IAS for signed differential pressure; no second
density, tube, or sensor compensation model was introduced. The logged quality
input contains both signed filtered differential pressure and the signed
Pitot-derived IAS.

The temporal indicator is the filtered magnitude of the rate of change of this
signed Pitot-derived signal. It is not described as pure `d_tr`. Crossing zero,
short negative excursions, and continuous in-range negative input remain
continuous at the quality-input boundary.

Host tests cover positive/negative symmetry, near-zero negative pressure,
cross-zero interpolation, in-range negative pressure, over-range, NaN, and Inf.
Zero-flow noise and low-speed swept input remain bench-verification items.
