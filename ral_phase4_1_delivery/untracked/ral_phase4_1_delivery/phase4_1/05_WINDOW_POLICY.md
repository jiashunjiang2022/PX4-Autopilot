# Spectral window policy

The required sample count is `round(Fs * window_seconds)`. With the default
50 Hz producer and 4.0 s window, evaluation requires exactly 200 valid uniformly
spaced samples. Sample 199 is not ready; sample 200 is the first eligible point.

Producer reset reasons are propagated as invalid samples. EKF2 resets the quality
estimator on invalid input and also resets when successive valid quality sample
timestamps are not exactly 20 ms apart. A reset, input drop, device change,
instance change, or window-parameter change therefore requires a complete refill.

The ring capacity remains 256 samples. The tested ring-start helper handles both
normal and wrapped 200/250-sample windows. Spectral evaluation is limited to no
more than once per configured interval, with a minimum default interval of 0.5 s.

Tests cover 199/200, reset-and-refill behavior, input-gap classification, ring
wrap, capacity rejection, and the 4 s to 5 s parameter change.
