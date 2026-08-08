# RA-L Phase 4.3: observation-time alignment and formal baseline

## Decision

`READY_FOR_FORMAL_BENCH`

This decision means the code, frozen-parameter gate, and evidence tooling are
ready to collect formal bench ULogs. It does not mean that a real bench ULog has
passed, and it does not establish flight readiness.

## Repository state

- Branch: `air`
- Base HEAD: `650836c48937f654a0a81fa894b381ff5b130cc2`
- All pre-existing Phase 4, 4.1, and 4.2 uncommitted work was preserved.
- No TECS, NPFG, attitude-control, or angular-rate-control law was changed by
  Phase 4.3.
- No hardware was connected and no flight test was performed.

## Implemented corrections

### Dual timestamps

`Ekf2AirspeedQuality` now records:

- `timestamp_sample`: original airspeed observation time.
- `ekf_buffer_timestamp_sample`: exact delayed timestamp accepted by
  `EstimatorInterface::setAirspeedData` and written to its ring buffer.
- `timestamp`: diagnostic publication time.
- `quality_timestamp_sample`: causal quality snapshot time.

`EstimatorInterface::setAirspeedData` returns the timestamp from the actual
queued sample after a successful push. EKF2 does not duplicate the delay
formula.

### Causal quality snapshot

EKF2 owns a fixed-capacity 16-entry snapshot ring with compile-time storage and
no dynamic allocation. Each airspeed observation selects the newest snapshot
whose timestamp is not later than the original observation. The default maximum
age is 200 ms. Startup without history, future-only history, stale history, and
invalid quality are logged with distinct reasons.

The ring is reset on producer reset, invalid input, non-20-ms gaps, and
non-monotonic timestamps. Wrap, startup, reset, gap, earlier/equal selection,
future rejection, and stale selection have unit coverage.

### Sample-level R policy

Adaptive R is applied only when all of the following hold:

- the formal mode requests adaptive R;
- the selected source is SENSOR_1;
- source instance is zero;
- nonzero device IDs match;
- the quality snapshot is causal, fresh, valid, and finite.

Otherwise the sample uses nominal R and records both the requested/applied flags
and a nonzero fallback reason. Stale quality cannot keep adaptive R active in
VARIANCE_ONLY or FULL_PROPOSED. FULL_PROPOSED retains the source-bound fusion
gate path, but stale quality cannot enable fusion or adaptive R.

Constant-R is similarly restricted to identity-matched SENSOR_1, instance zero.
Synthetic, ground-minus-wind, alternate physical, unknown, and mismatched
sources use nominal R.

## Formal blockage baseline

The reboot-required `ASPD_QBLK_EN` parameter controls the inherited custom
blockage heuristic. Its default is zero. When zero, the custom detector and its
fallback path do not run, state is cleared, and `concurrent_blockage` is false.
Original PX4 AirspeedValidator and selector behavior remains active.

All formal mode templates freeze `ASPD_QBLK_EN=0`. The evidence checker fails if
the parameter is missing/nonzero or if a formal log reports a blockage trigger
or concurrent blockage flag. When explicitly enabled outside the formal
baseline, blockage fallback uses the common `choose_fallback` path and honors
`ASPD_FALLBACK=0/1/2`.

## ULog evidence contract

The fail-closed checker uses:

- original observation time for quality and selector joins;
- EKF buffer time for exact `estimator_aid_src_airspeed` R joins;
- publication time only as recorded context.

It rejects unmatched aid samples, future quality, incorrect observation-based
quality age, stale adaptive samples, Constant-R on an ineligible source,
nonzero formal blockage configuration, unexplained selector transitions, and
logger dropouts. Outputs remain JSON plus CSV evidence tables.

## Verification

- CTest: 8/8 passed, including the seven Phase 4 targets and
  `functional-uORBMessageFields`.
- Python evidence tests: 25/25 passed.
- `git diff --check`: passed.
- `px4_fmu-v6c_default`: passed.
- FLASH: 1,832,016 bytes of 1,966,080 bytes, 93.18%.
- AXI SRAM: 61,892 bytes of 524,288 bytes, 11.80%.
- Maximum untokenized uORB format: 1,638 bytes.

The build exposed two target-only constraints that were corrected and retained
in the audit logs: NuttX does not provide `<array>`, so the fixed snapshot ring
uses an equivalent compile-time C array; the logger format buffer was increased
from 1,600 to 1,800 bytes to carry the required diagnostic schema with PX4's
150-byte expansion margin. The uORB message-format functional test passes.

## Remaining formal bench gates

Before any later flight-readiness decision, collect real bench ULogs for all
four modes with run-specific frozen parameters, then require the authoritative
checker to pass every ULog. Confirm zero logger/ULog dropouts, exact buffer-time
R joins, no future or stale adaptive samples, stable single-Pitot identity, and
`ASPD_QBLK_EN=0` throughout the formal dataset.
