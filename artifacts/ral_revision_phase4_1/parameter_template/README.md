# Phase 4.1 frozen configuration

Create a run-specific copy of `frozen_config.template.json` and replace every
`null` with the bench-approved selector value. The template intentionally omits
the removed `EKF2_ASP_SFS`; the quality producer owns the fixed 50 Hz output
contract. `SENS_DP_QMAX` is an absolute signed differential-pressure limit in Pa.

Before every bench or formal data run, record and verify the exact mode,
`EKF2_ASP_RCST`, the positive `EKF2_ARSP_THR`, logger profile, Pitot device ID,
and differential-pressure uORB instance. Run `preflight_check.py` on the
post-reboot ULog. Any unknown/zero device, multiple physical Pitots, identity
mismatch, unexplained selected-source switch, missing topic, or logger dropout
is a hard FAIL.

Startup identity is allowed a bounded warm-up (`identity_bootstrap_timeout_s`,
or `--bootstrap-timeout-s`). Samples before the first complete device/instance
match are reported as bootstrap and excluded from the formal identity rate.
After that first match every physical-Pitot selector sample must match. If the
log arms, the first armed timestamp is automatically treated as the formal
valid-interval start; `--valid-start-us` can set an earlier run-specific start.
Bootstrap may not overlap that interval.

This checker is an offline evidence gate. It does not replace the swept-tone,
stack high-water, WCET, SD-soak, hardware-in-loop, or flight-readiness gates.
