# FAST history reset policy

Generic `resetIntegralAndTransfer()` and `resetRollIntegralAndTransfer()` remain
control-only and retain their original cadence. FAST history is cleared by a
single transition detector in `Run()` when entering or leaving the sustained
reset regime (`landed || !fixed-wing-rate regime || !rate control enabled`).
Thus repeated landed housekeeping resets do not erase the diagnostic ring each
cycle, while ground-to-flight and flight-to-ground boundaries clear once.
