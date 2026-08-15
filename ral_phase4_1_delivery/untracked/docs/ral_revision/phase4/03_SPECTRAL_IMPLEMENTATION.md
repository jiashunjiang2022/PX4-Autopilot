# Spectral implementation

The estimator retains the multi-bin Goertzel design. Frozen defaults are 50 Hz, 4 s, 200 samples, 0.25 Hz bin spacing, 0.5--8.0 Hz reference band, and 0.5 Hz flap half-band. The ring holds 256 samples; the window parameter is limited to 5 s at 50 Hz.

At the platform maximum, the 6.8 +/- 0.5 Hz flap band is fully inside the default reference band. An 8 Hz center with a 0.5 Hz half-band is intentionally invalid under the 8 Hz default upper bound; the 8 Hz tone test uses a 9 Hz reference upper bound. This is explicit band policy, not clamping.

Full spectral evaluation occurs no faster than `EKF2_ASP_SEVL` (default 0.5 s), not for every 50 Hz sample. The result includes an update flag and counter so held diagnostics cannot be confused with a new calculation.

Explicit invalid reasons cover insufficient samples, input rate, Nyquist, reference band, flap band, zero energy, nonfinite values, input gap, and missing recent flap frequency. Early-boot window subtraction saturates at zero.

Deterministic tests cover 2, 3, 4, 5, 5.5, 6.8 and 8 Hz tones; flap/reference edge crossing; broadband noise; slow trend plus flap tone; jitter; dropped, duplicate, non-monotonic and long-gap timestamps; Nyquist overlap; and zero energy. Runtime threshold calibration and target-board WCET remain bench tasks.
