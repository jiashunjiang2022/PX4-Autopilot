# Phase 4.2 bench evidence protocol

1. Create and freeze a run-specific copy of
   `artifacts/ral_revision_phase4_2/ulog_evidence/frozen_config.template.json`.
2. Reboot before the run. Do not arm or enter the formal valid interval until
   selector identity bootstrap has completed.
3. Collect the ULog with the required quality, selector, EKF aid-source, and
   logger topics.
4. Run `check_airspeed_evidence.py` with the expected experiment mode and frozen
   JSON.
5. Retain the generated summary JSON and all CSV tables with the experiment.

Required acceptance includes:

- one nonzero physical Pitot device and one differential-pressure instance;
- bootstrap completed within the predeclared timeout;
- zero post-bootstrap physical identity mismatches;
- valid 50 Hz input on an exact 20 ms sample grid with no held timestamps;
- no post-bootstrap reset/gap-counter changes;
- selector/validated and R join rates at or above frozen thresholds;
- zero joined R value mismatches within frozen absolute/relative tolerances;
- expected mode truth-table behavior;
- no unexplained selector transition;
- zero ULog, logger-buffer, and logger message-gap dropouts.

For `VARIANCE_ONLY` and `FULL_PROPOSED`, the log must contain enough quality
variation to demonstrate that the R/nominal-R ratio changes. A static-q log
fails closed because it cannot prove the requested behavior.
