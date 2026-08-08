# RA-L Phase 4.3 artifacts

This directory records software-only evidence for observation-time alignment
and formal-baseline correction. No hardware or flight test was performed.

The authoritative fail-closed ULog checker remains at:

`artifacts/ral_revision_phase4_2/ulog_evidence/check_airspeed_evidence.py`

Use `ulog_evidence/frozen_config.template.json` to create a run-specific frozen
configuration before collecting each formal bench ULog. Every `null` value must
be resolved before use. `ASPD_QBLK_EN` is fixed at zero for all formal modes.

Logs:

- `logs/ctest_phase4_3.log`: final 8-test CTest run.
- `logs/python_evidence_tests.log`: final 25-test Python evidence run.
- `logs/px4_fmu-v6c_default_build.log`: first build attempt; records the NuttX
  `<array>` incompatibility.
- `logs/px4_fmu-v6c_default_build_retry.log`: second build attempt; records the
  uORB format-buffer limit.
- `logs/px4_fmu-v6c_default_build_final.log`: successful final firmware build.

`build_metrics.json` and `test_results.json` summarize the final accepted run.
The failed intermediate logs are intentionally retained as complete audit
history.
