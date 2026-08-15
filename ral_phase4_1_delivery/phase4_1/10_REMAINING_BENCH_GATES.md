# Remaining bench gates

Current status is `READY_FOR_BENCH_REVIEW`. It is not `READY_FOR_FLIGHT`.

The following gates remain mandatory:

1. Confirm exactly one physical Pitot, one active DP uORB instance, and nonzero
   stable device IDs after reboot.
2. Run the Phase 4.1 offline preflight checker on every bench/formal ULog and
   require identity equality and explained source transitions.
3. Perform zero-flow, low-speed cross-zero, negative-pressure, range-limit, and
   reconnect tests with the installed tubing and MS4525DO.
4. Perform swept-tone anti-alias acceptance around the flapping band and near
   the 25 Hz output Nyquist frequency.
5. Measure EKF2 stack high-water, estimator/spectral WCET distributions, CPU
   load, deadlines, and missed-quality counters on target hardware.
6. Verify all four modes with runtime topic joins: diagnostic R equals the
   accepted observation and the aid-source variance for each sample.
7. Exercise low/stale/invalid quality, blockage concurrency, fallback outcomes,
   recovery, and unexplained-switch rejection using actual module publications.
8. Complete a 30-minute SD-card soak with the RA-L profile and zero ULog/logger
   dropouts or message gaps.
9. Complete normal hardware, HIL, ground-run, and institutional flight-safety
   review gates before any flight consideration.
