# Remaining risks

The code is ready for review, not bench or flight release.

Blocking evidence still absent:

1. MS4525DO and AS5600 device IDs on the actual aircraft.
2. Measured differential-pressure producer rate and continuous 50 Hz quality output under load.
3. Anti-alias cutoff and all q/R/gate/selector thresholds frozen from representative bench data.
4. EKF INS work-queue WCET, missed deadlines, and stack high-water at worst spectral evaluation load.
5. Source-switch transition bounds at the selector -> TECS interface, including simultaneous blockage and quality rejection.
6. At least 30 minutes of target SD logging with no ULog dropout, logger buffer overflow, or message gap.
7. Confirmation that persisted parameters do not retain the old 7.5 ratio or disable airspeed fusion.
8. Hall polarity, pin mapping, producer start, and edge accounting on the target aircraft.

The static logger estimate is not an SD qualification. Uniform interpolation does not increase sensor information bandwidth. The default 8 Hz reference upper limit explicitly rejects an 8 +/- 0.5 Hz flap band; this does not affect the 6.8 Hz platform maximum.

The unrelated MS4525DO double-read consistency issues identified in the Phase 3 audit were not changed because Phase 4 constrained work to the requested core patch and prohibited sensor replacement. They remain a separate P1 driver/bench item.
