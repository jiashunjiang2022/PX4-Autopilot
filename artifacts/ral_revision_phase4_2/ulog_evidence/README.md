# Phase 4.3 authoritative ULog evidence

Use a run-specific copy of `frozen_config.template.json`. Replace every `null`
before collecting data and do not change acceptance thresholds after inspecting
the ULog.

```sh
python3 artifacts/ral_revision_phase4_2/ulog_evidence/check_airspeed_evidence.py \
  path/to/log.ulg \
  --expected-mode FULL_PROPOSED \
  --config path/to/frozen_config.json \
  --output-dir artifacts/ral_revision_phase4_2/ulog_evidence/run_name
```

Accepted mode names are `BASELINE`, `CONSTANT_R`, `VARIANCE_ONLY`, and
`FULL_PROPOSED`; numeric values 0 through 3 are also accepted.

The checker fails closed on missing topics or fields, unresolved parameters,
identity bootstrap failure, post-bootstrap identity mismatch, a nonuniform
quality-input grid, resets/gaps, selector/validated join loss, EKF diagnostic
identity errors, future or unmatched quality, stale adaptive R, R mismatch,
mode truth-table failure, an enabled formal blockage heuristic, unexplained
selector transitions, or logger dropouts.

Sensor and selector evidence is joined with the original airspeed observation
`timestamp_sample`. R is joined using uORB instance plus
`ekf_buffer_timestamp_sample` against the aid-source `timestamp_sample`. Topic
publication timestamps are recorded in the CSV but are never used as either
join key. The run-specific frozen configuration must contain
`ASPD_QBLK_EN=0` for all four formal modes.

Outputs always include `airspeed_evidence_summary.json`. Available evidence
tables include:

- `checks.csv`
- `failures.csv`
- `quality_input.csv`
- `selector_validated_join.csv`
- `selector_unmatched.csv`
- `ekf_diagnostic_join.csv`
- `r_join.csv`
- `selector_traceability.csv`
- `selector_transitions.csv`
- `topic_rates.csv`

An exit status of zero and `RESULT=PASS` are required. This is a bench-evidence
gate and does not establish flight readiness.
