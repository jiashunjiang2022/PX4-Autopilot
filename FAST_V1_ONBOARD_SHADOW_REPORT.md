# FAST-V1 onboard shadow inference

This branch contains diagnostics-only onboard execution of the frozen ABS4 and
DELTA4 full-development exports. The export package is
`fast_model_export_full_dev_appliedt`; `verify_export.py` passed for both models.

The runtime uses float32 StandardScaler plus OLS coefficients, a causal 20 Hz
frame gate, and a fixed nine-frame total-equivalent-I history. During warm-up
or non-finite input, predictions remain invalid and are held at zero. The
predictions are copied only to `flap_fast_shadow`; no model output is read by
RateControl, BumplessRollITransfer, torque setpoint, allocator, or actuator
code. No injection parameter or actuator publication was added.

Model identities:

- `FAST_V1_ABS4_H03_FULLDEV_APPLIEDT_v1`, SHA256
  `88d243911ac8d048099ba5bed058cbb89c43135aebc91b8c3725fc02be342bec`
- `FAST_V1_DELTA4_H03_FULLDEV_APPLIEDT_v1`, SHA256
  `134eb1a349925d9441c7d839f5880f70e8ffb67f3f07339cae62460d73d61181`

Historical grid phase is not claimed to be exactly reproduced; the onboard
implementation is an explicitly causal fixed 50 ms frame grid using latest
held controller values.
