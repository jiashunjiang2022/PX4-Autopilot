# Disarmed bench plan (not executed)

1. Independently review diff and firmware SHA; keep propulsion disconnected and vehicle DISARMED.
   Do not flash automatically. Check firmware identity, existing aircraft configuration and logger.
2. Phase A: param set FLAP_FAST_EN 0. Observe listener flap_fast_control and
   listener flap_fast_shadow; inspect fw_rate_control status and logger status.
   Capture a ULog and verify FAST final is exactly zero, existing baseline outputs and
   I/S behavior unchanged for equivalent inputs, and no log dropouts.
3. Phase B, still DISARMED: param set FLAP_FAST_EN 1.
   Verify enabled=true, disabled_by_state=true, final=0. Observe predictor frame sequence,
   nominal 20 Hz cadence, finite outputs after warmup and no missed frames.
   Do not expect control-valid/nonzero gates while disarmed: safety bypasses that chain.
4. Exercise nonzero gate/slew/stale/reset through supplied host test, not by bypassing arming
   guards. Replay decision_timestamp/decision_dt/model_timestamp and all intermediates in
   future runtime captures. No armed propulsive test is authorized here.
5. param set FLAP_FAST_EN 0; verify immediate zero. Leave disabled.
6. Before any flight proposal resolve instantaneous shared-tail margin, verify work-queue
   stack/latency and ULog completeness on hardware, then obtain separate review/authorization.

READY_FOR_FAST_ON_FLIGHT=NO
