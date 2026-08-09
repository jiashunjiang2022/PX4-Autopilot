# Source-rate and filter contract

Bench data from the physical MS4525DO producer shows a mean source rate near
59 Hz, a typical operating range of 53-65 Hz, and no source gaps above 40 ms.
The accepted producer contract is therefore 52-100 Hz, not an arbitrary 50 Hz source.

At least ten real source intervals are required before rate validity. The source
rate is smoothed; the low-pass filter is not reconfigured on every sample. A
stable measured-rate shift greater than 5 percent resets filter/interpolation
state and configures the filter with the new measured rate.

The second-order anti-alias filter uses `SENS_DP_QCUT` (default 10 Hz), then
publishes on a fixed 20 ms grid only when two real filtered source samples
bracket the requested output time. It does not publish held values. A source
outside 52-100 Hz publishes invalid quality and does not use interpolation to
manufacture a valid 50 Hz signal.

Generated records expose measured source rate, rate validity, configured filter
rate, cutoff, output rate, reset counter, and reset reason. Swept-tone attenuation
and phase response remain mandatory bench tests.
