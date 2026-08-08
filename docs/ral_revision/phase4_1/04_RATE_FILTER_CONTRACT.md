# Source-rate and filter contract

The MS4525DO driver schedules a 2 ms conversion wait followed by 10 ms before
the next measurement, giving a nominal period near 12 ms or 83.33 Hz. The
accepted producer contract is therefore 70-100 Hz, not an arbitrary 50 Hz source.

At least ten real source intervals are required before rate validity. The source
rate is smoothed; the low-pass filter is not reconfigured on every sample. A
stable measured-rate shift greater than 5 percent resets filter/interpolation
state and configures the filter with the new measured rate.

The second-order anti-alias filter uses `SENS_DP_QCUT` (default 10 Hz), then
publishes on a fixed 20 ms grid only when two real filtered source samples
bracket the requested output time. It does not publish held values. A source
outside 70-100 Hz publishes invalid quality and does not use interpolation to
manufacture a valid 50 Hz signal.

Generated records expose measured source rate, rate validity, configured filter
rate, cutoff, output rate, reset counter, and reset reason. Swept-tone attenuation
and phase response remain mandatory bench tests.
