# Changed files

There are 36 core implementation/message/parameter/test files in the Phase 4 patch.

## Messages (4)

- `msg/CMakeLists.txt`
- `msg/AirspeedQualityInput.msg`
- `msg/AirspeedSelectorQualityStatus.msg`
- `msg/Ekf2AirspeedQuality.msg`

## Mechanical ratio and phase (11)

- `src/drivers/encoder/as5600/AS5600.cpp`
- `src/drivers/encoder/as5600/AS5600.hpp`
- `src/drivers/encoder/as5600/AS5600Math.hpp`
- `src/drivers/encoder/as5600/AS5600MathTest.cpp`
- `src/drivers/encoder/as5600/CMakeLists.txt`
- `src/drivers/encoder/as5600/module.yaml`
- `src/modules/wing_phase/CMakeLists.txt`
- `src/modules/wing_phase/WingPhase.cpp`
- `src/modules/wing_phase/WingPhaseMath.cpp`
- `src/modules/wing_phase/WingPhaseMath.hpp`
- `src/modules/wing_phase/WingPhaseMathTest.cpp`

## Quality, modes, and EKF (12)

- `src/lib/airspeed/CMakeLists.txt`
- `src/lib/airspeed/AirspeedQualityMode.hpp`
- `src/lib/airspeed/AirspeedQualityModeTest.cpp`
- `src/modules/ekf2/CMakeLists.txt`
- `src/modules/ekf2/EKF/common.h`
- `src/modules/ekf2/EKF2.cpp`
- `src/modules/ekf2/EKF2.hpp`
- `src/modules/ekf2/AirspeedQualityMath.cpp`
- `src/modules/ekf2/AirspeedQualityMath.hpp`
- `src/modules/ekf2/params_airspeed.yaml`
- `src/modules/ekf2/test/CMakeLists.txt`
- `src/modules/ekf2/test/test_AirspeedQuality.cpp`

## Producer, selector, logger, and test registration (9)

- `src/modules/sensors/sensor_params.c`
- `src/modules/sensors/sensors.cpp`
- `src/modules/sensors/sensors.hpp`
- `src/modules/airspeed_selector/CMakeLists.txt`
- `src/modules/airspeed_selector/airspeed_selector_main.cpp`
- `src/modules/airspeed_selector/AirspeedSelectorQuality.hpp`
- `src/modules/airspeed_selector/AirspeedSelectorQualityTest.cpp`
- `src/modules/logger/logged_topics.cpp`
- `test/CMakeLists.txt`

This patch intentionally does not modify TECS, NPFG, attitude control, rate control, ControlAllocator control laws, the MS4525DO driver, or the EKF Kalman measurement update implementation.

The 11 files in this directory and generated files under `artifacts/ral_revision_phase4` are delivery evidence and are not included in the 36-file core count.
