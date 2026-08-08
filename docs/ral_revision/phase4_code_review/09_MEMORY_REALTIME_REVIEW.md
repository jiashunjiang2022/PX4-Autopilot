# Memory and real-time review

## Build evidence

- Target build: PASS (`px4_fmu-v6c_default`).
- FLASH: `1,828,392 B / 1,920 KiB = 93.00%`.
- AXI SRAM: `61,892 B / 512 KiB = 11.80%`.
- Before-patch FLASH/RAM and per-symbol growth were not captured, so incremental bytes and largest contributors are unknown.
- Host-only gtests are not linked into flight firmware. New messages, strings, parameter metadata, estimator code, and logger topic names do contribute to target size.

FLASH classification: **TOO_CLOSE_FOR_SAFE_ITERATION**. It is not automatically a build blocker, but the roughly 7% headroom leaves limited room for review fixes and target/configuration variance.

## Runtime footprint

`AirspeedQualityEstimator` owns 256 floats plus 256 `uint64_t` timestamps: approximately 3,072 B persistent, excluding scalar state (`EKF2.hpp:234-285`). Every accepted update declares another 256-float and 256-timestamp pair: approximately 3,072 B in the EKF2 call stack (`EKF2.cpp:338-350`), even when spectral evaluation is not due. At the default band/window, one evaluation performs about 31 Goertzel bins x 200 samples, plus window copy/mean removal, every 0.5 s while eligible. The arrays are fixed; no new estimator heap allocation was found. Existing EKF observation ring allocation is unchanged in mechanism.

## Finding

| ID / severity | File / function / lines | Observed behavior | Risk | Required correction | Verification |
|---|---|---|---|---|---|
| P1-05 / P1 / NEEDS_BENCH_VERIFICATION | `src/modules/ekf2/EKF2.hpp`, `AirspeedQualityEstimator`, 222-285; `src/modules/ekf2/EKF2.cpp`, `AirspeedQualityEstimator::update`, 330-396 | Roughly 3 KiB persistent buffers are duplicated by roughly 3 KiB stack temporaries; window copying occurs at 50 Hz and Goertzel runs on the EKF2 scheduled work item. No WCET, deadline, stack high-water, or consumer-overwrite evidence is supplied. | Stack exhaustion or EKF deadline delay can be safety-critical; missed unqueued quality samples can also corrupt the assumed uniform spectral input. | Remove avoidable per-call copies/stack pressure or justify them with target margins; instrument runtime and detect consumer gaps. | Target worst-case flap/load run with EKF perf latency/WCET, missed schedule count, stack high-water, CPU load, and quality timestamp continuity. |

No blocking call or estimator-side dynamic allocation was introduced in the quality algorithm. Static RAM percentage alone is insufficient to clear stack and deadline risk.
