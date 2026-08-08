# Remaining bench gates

The code and offline tooling are ready to collect bench evidence, but no real
ULog was available in this software-only phase. Each frozen experiment mode
still requires a real post-reboot bench ULog and `RESULT=PASS` from the Phase 4.2
checker.

Bench work must also retain the existing Phase 4.1 gates that software tests do
not establish: swept-tone anti-alias response, real sensor rate/jitter, stack
high-water and runtime timing, SD throughput/soak with zero dropouts, and source
transition tests for each configured `ASPD_FALLBACK` value.

Passing these bench gates is not flight authorization. Hardware-in-loop and
flight-readiness review remain separate later stages.
