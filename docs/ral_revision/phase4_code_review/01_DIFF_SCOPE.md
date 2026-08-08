# Diff scope review

## Boundary

- Branch: `air`
- HEAD: `650836c48937f654a0a81fa894b381ff5b130cc2`
- Index diff: empty at review start
- Core patch: 25 tracked modifications plus 11 new files, 36 files total
- Review output is confined to `docs/ral_revision/phase4_code_review/` and `artifacts/ral_revision_phase4_review/`.

## Core files

| Area | Files | Scope result |
|---|---|---|
| uORB | `msg/CMakeLists.txt`, `msg/AirspeedQualityInput.msg`, `msg/AirspeedSelectorQualityStatus.msg`, `msg/Ekf2AirspeedQuality.msg` | Expected |
| Encoder | `src/drivers/encoder/as5600/AS5600.cpp`, `AS5600.hpp`, `AS5600Math.hpp`, `AS5600MathTest.cpp`, `CMakeLists.txt`, `module.yaml` | Expected mechanical ratio/evidence path |
| Wing phase | `src/modules/wing_phase/CMakeLists.txt`, `WingPhase.cpp`, `WingPhaseMath.cpp`, `WingPhaseMath.hpp`, `WingPhaseMathTest.cpp` | Expected |
| Mode helper | `src/lib/airspeed/CMakeLists.txt`, `AirspeedQualityMode.hpp`, `AirspeedQualityModeTest.cpp` | Expected |
| EKF2 quality | `src/modules/ekf2/CMakeLists.txt`, `EKF/common.h`, `EKF2.cpp`, `EKF2.hpp`, `AirspeedQualityMath.cpp`, `AirspeedQualityMath.hpp`, `params_airspeed.yaml`, `test/CMakeLists.txt`, `test/test_AirspeedQuality.cpp` | Expected; does not alter generated Kalman update |
| Producer | `src/modules/sensors/sensor_params.c`, `sensors.cpp`, `sensors.hpp` | Expected; producer is in generic sensors module, not MS4525DO driver |
| Selector | `src/modules/airspeed_selector/CMakeLists.txt`, `airspeed_selector_main.cpp`, `AirspeedSelectorQuality.hpp`, `AirspeedSelectorQualityTest.cpp` | Expected |
| Logger/tests | `src/modules/logger/logged_topics.cpp`, `test/CMakeLists.txt` | Expected |

## Requested 15-category mapping

| Category | Actual changed surface |
|---|---|
| 1. MS4525DO / differential pressure | `sensors.cpp/.hpp`, `sensor_params.c`; MS4525DO driver itself unchanged |
| 2. Airspeed quality input | `sensors.cpp/.hpp`, `AirspeedQualityInput.msg` |
| 3. Resampling / anti-alias filter | `sensors.cpp/.hpp` |
| 4. Spectral estimator | `EKF2.cpp/.hpp`, `AirspeedQualityMath.cpp/.hpp` |
| 5. Temporal indicator | `EKF2.cpp/.hpp` |
| 6. EKF2 variance path | `EKF/common.h`, `EKF2.cpp`, mode helper; existing Kalman update unchanged |
| 7. Experiment mode | `AirspeedQualityMode.hpp`, EKF2/selector params and call sites |
| 8. Airspeed selector | `airspeed_selector_main.cpp`, helper and tests |
| 9. Fallback | selector runtime/helper/tests |
| 10. uORB messages | four `msg/` files listed above |
| 11. Parameters | `params_airspeed.yaml`, `sensor_params.c`, AS5600 `module.yaml` |
| 12. Logger | `logged_topics.cpp` |
| 13. Tests | five unit-test sources plus two test CMake registrations |
| 14. Build configuration | changed module/library/message/test `CMakeLists.txt` files |
| 15. Documentation or scripts | existing Phase 4 delivery docs/artifacts are outside the 36-core-file count and were not modified by this review |

## Control-law boundary

No modified file belongs to TECS control laws, NPFG control laws, attitude control, rate control, mixer/ControlAllocator, or the EKF airspeed Kalman equations. `src/modules/ekf2/EKF/aid_sources/airspeed/airspeed_fusion.cpp` is unchanged; the new `noise_var` field reaches that existing update. The MS4525DO driver is also unchanged. Therefore no out-of-scope control-law modification and no scope-based `BLOCKER` were found.

The patch does affect generic airspeed publication, EKF2 scheduling load, selector output choice in FULL mode, uORB schema, logger profile, encoder ratio, and wing phase. These are planned but flight-relevant surfaces and are covered by the remaining reports.
