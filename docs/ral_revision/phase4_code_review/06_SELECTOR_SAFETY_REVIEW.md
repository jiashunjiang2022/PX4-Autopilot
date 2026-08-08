# Selector and fallback safety review

## Runtime path

```text
normal PX4 selection and value copy
 -> pre_quality_source + finite-value original_valid
 -> FULL only: q freshness/validity/fuse decision
 -> latch (hold + recovery dwell)
 -> alternate valid physical sensor
 -> configured ground-minus-wind or synthetic fallback
 -> disabled/NaN only when no fallback exists
 -> blockage fallback check
 -> final source/validity status
 -> airspeed_validated publication to TECS/consumers
```

BASELINE, CONSTANT_R, and VARIANCE_ONLY clear/bypass the quality latch at `airspeed_selector_main.cpp:854-858`; only FULL evaluates q. Original selected values and their finite validity are recorded before quality processing (`830-843`). Quality rejection does not change each `AirspeedValidator`'s original PX4-valid flag. A valid alternate physical source is preferred, followed by configured ground-wind/synthetic fallback (`887-932`). NaNs are assigned only when fallback is unavailable, not unconditionally. The final status and `airspeed_validated` are published in the same run (`951-957`).

The latch provides a 1 s q-message freshness check, hold, q hysteresis, and recovery dwell (`845-881` and `AirspeedSelectorQuality.hpp`). This is separate from EKF2's 200 ms quality-input freshness gate, so exact transition behavior still needs integrated timing evidence.

## Findings

| ID / severity | File / function / lines | Observed behavior | Risk | Required correction | Verification |
|---|---|---|---|---|---|
| P0-01 / P0 | `src/modules/sensors/sensors.hpp`, `Sensors`, 183; `src/modules/airspeed_selector/airspeed_selector_main.cpp`, `AirspeedModule::Run`, 845-938 | Selector uses one global q without sensor instance/device identity, including after normal source changes and when selecting an alternate physical source. | Rejection/fallback decisions may refer to another Pitot. | Bind the quality decision to `pre_quality_source` and qualify alternate sensors independently, or enforce a logged single-Pitot invariant. | Multi-Pitot source-switch and device-switch module test plus ULog identity assertion. |
| P1-04 / P1 / NEEDS_ULOG_VERIFICATION | `msg/AirspeedSelectorQualityStatus.msg`, message schema, 1-28; `airspeed_selector_main.cpp`, status publication, 830-955 | Status has publication time only. When fallback succeeds, `FALLBACK_SELECTED` overwrites the initiating low-q/stale/blockage cause; blockage handling may overwrite prior quality cause. | An experiment log cannot reconstruct why or exactly when a source transition occurred, especially for concurrent blockage and quality rejection. | Add sample/decision timestamp and separate trigger/cause from fallback outcome; retain q/device/source identity and concurrent causes. | Replay/module test must assert exact status fields and ordering for low q, stale q, blockage, simultaneous causes, no fallback, and recovery. |

Helper tests cover pure decision/latch behavior but do not instantiate `AirspeedModule`, check numerical IAS/CAS/TAS copies, or observe the TECS-facing publication. This is included in `P1-06`.

Static conclusion: fallback logic is fail-explicit and avoids unconditional NaN output, but **selector_safe is conditional/false for release** until q-to-source identity and transition evidence are fixed.
