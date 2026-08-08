# ULog checks

Run after a disarmed producer check and again after the full SD soak:

```sh
python3 artifacts/ral_revision_phase4/parameter_template/preflight_check.py bench.ulg \
  --config frozen_run.json --expected-mode 3 --expected-rcst 2.0
python3 artifacts/ral_revision_phase4/ulog_rate_check/check_ulog_rates.py bench.ulg \
  --csv measured_rates.csv
```

Both commands are fail-closed and return nonzero on missing evidence. The rate
checker uses `timestamp_sample` when available, otherwise `timestamp`; logger
intervals are treated only as upper limits and are never reported as producer rates.
