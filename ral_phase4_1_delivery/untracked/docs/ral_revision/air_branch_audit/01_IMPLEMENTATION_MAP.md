# 实现边界与调用链

## 仓库边界

- branch：`air`
- commit：`650836c48937f654a0a81fa894b381ff5b130cc2`
- 审计开始时工作树：clean
- 本文实现状态：`PARTIAL`。存在可执行算法、EKF 方差接入和 selector safeguard，但缺四模式、完整频谱覆盖、可靠 Hall/ULog 合同和测试。

## 模块地图

| 分类 | 主要文件与函数 | 输入 | 输出 | 状态 |
|---|---|---|---|---|
| MS4525DO | `src/drivers/differential_pressure/ms4525do/MS4525DO.cpp:115-229`, `MS4525DO::RunImpl()` | I2C conversion/read | `differential_pressure`：`timestamp_sample`, Pa, degC, `error_count` | `PARTIAL` |
| airspeed processing | `src/modules/sensors/sensors.cpp:261-346`, `Sensors::diff_pres_poll()` | `differential_pressure` | `airspeed`：IAS/TAS，约 20 Hz | `PASS` 最低频率 |
| AS5600 | `src/drivers/encoder/as5600/AS5600.cpp:120-239`, `AS5600::RunImpl()` | angle raw | `encoder_count`, `flap_frequency`, `rpm`, `debug_vect` | `FAIL` ratio default |
| Hall capture | `src/drivers/rpm_capture/RPMCapture.cpp:108-172`, `RPMCapture::Run()`/GPIO ISR | RPM input edge | `hall_event.timestamp`, `pulse_count` | `PARTIAL` |
| wing phase | `src/modules/wing_phase/WingPhase.cpp:114-232`, `WingPhase::Run()` | `encoder_count`, `flap_frequency`, `hall_event` | `wing_phase` | `PARTIAL` |
| quality estimator | `src/modules/ekf2/EKF2.cpp:46-630`, `AirspeedQualityEstimator::update()` | TAS + flap frequency + parameters | q, `R_as_used`, fusion gate | `FAIL` spectral coverage |
| EKF2 input bridge | `src/modules/ekf2/EKF2.cpp:2673-3036`, `EKF2::UpdateAirspeedSample()` | raw/validated airspeed | `airspeedSample.noise_var`, quality topic | `PARTIAL` mixed source |
| EKF fusion | `src/modules/ekf2/EKF/aid_sources/airspeed/airspeed_fusion.cpp:94-243`, `Ekf::controlAirDataFusion()`/`updateAirspeed()` | `airspeedSample` | wind state, aid status | `PASS` covariance wiring |
| selector | `src/modules/airspeed_selector/airspeed_selector_main.cpp:673-889`, `AirspeedSelector::Run()` | validators + q | `airspeed_validated` | `FAIL` path isolation/logging |
| TECS/control | `src/modules/fw_lateral_longitudinal_control/FwLateralLongitudinalControl.cpp:289-477,616-641` | selected CAS/TAS ratio, setpoints | TECS, lateral/longitudinal, attitude setpoint | `PARTIAL` logging |
| NPFG/mode | `src/modules/fw_mode_manager/FixedWingModeManager.cpp:160-180,2720-2734` | selected CAS, local position/wind | fixed-wing guidance/setpoints | `PARTIAL` logging |
| controllers | `src/modules/fw_att_control/FixedwingAttitudeControl.cpp:144-167`; `src/modules/fw_rate_control/FixedwingRateControl.cpp:153-180` | attitude/rate SP + selected CAS | rates, torque/thrust | `PASS` chain exists |
| allocator | `src/modules/control_allocator/ControlAllocator.cpp:692-757` | torque/thrust setpoint | actuator motors/servos | `PASS` |
| logger | `src/modules/logger/logged_topics.cpp:46-220,383-410,580-634` | uORB topics | ULog | `FAIL` dataset completeness |
| uORB | `msg/Ekf2AirspeedQuality.msg`, `msg/EncoderCount.msg`, `msg/FlapFrequency.msg`, `msg/HallEvent.msg`, `msg/WingPhase.msg`, `msg/CMakeLists.txt` | publishers | subscribers/logger | `PARTIAL` schema gaps |
| parameters | `src/modules/ekf2/params_airspeed.yaml:46-224`; `src/drivers/encoder/as5600/module.yaml:6-18` | parameter store | estimator/encoder/phase | `FAIL` mode/ratio |
| build/start | board `default.px4board`, `ROMFS/px4fmu_common/init.d/rcS:449-453` | board config/params | module startup | `PARTIAL` Hall disabled |
| analysis scripts | relevant RA-L spectral/flight analysis helper | `NOT FOUND` in audited implementation map | none | `UNKNOWN` |

## 实际调用链

| Segment | Caller -> callee / topic | field contract | 状态与证据 |
|---|---|---|---|
| sensor conversion | scheduler -> `MS4525DO::RunImpl()` | conversion starts at sample timestamp; read/publish after 2 ms then next cycle 10 ms | `PASS`, `MS4525DO.cpp:115-125,199-227` |
| pressure publication | `MS4525DO::RunImpl()` -> `PublicationMulti<differential_pressure>` | `timestamp_sample`, `timestamp`, `differential_pressure_pa`, `temperature`, `error_count` | `PASS`, `MS4525DO.cpp:199-215` |
| pressure processing | `differential_pressure` -> `Sensors::diff_pres_poll()` -> `airspeed` | 50 ms averaging; `timestamp_sample` is averaged source sample time | `PARTIAL`, `sensors.cpp:261-346` |
| spectral input | `EKF2::UpdateAirspeedSample()` -> `AirspeedQualityEstimator::update()` | validated TAS/publication timestamp or raw TAS/sample timestamp | `FAIL`, `EKF2.cpp:2755-2777,2862-2884,2926-2951` |
| q to covariance | estimator -> `airspeedSample.noise_var` -> `Ekf::setAirspeedData()` | variance `(m/s)^2` | `PASS`, `EKF2.cpp:581-601,2814-2822,2987-2995` |
| covariance fusion | `Ekf::controlAirDataFusion()` -> `Ekf::updateAirspeed()` | uses sample `noise_var` as observation R and updates aid status | `PASS`, `airspeed_fusion.cpp:94-126,169-243` |
| quality publication | `EKF2::UpdateAirspeedSample()` -> `ekf2_airspeed_quality` | q, q_raw, q_smoothed, R, spectral/gate diagnostics | `PARTIAL`, `EKF2.cpp:2886-2918,2997-3030`; missing mode/R0/sample time |
| selector safeguard | `AirspeedSelector::Run()` consumes q -> changes outgoing selected source | low/invalid/stale q may publish NaN and `SOURCE_DISABLED` | `FAIL`, `airspeed_selector_main.cpp:823-889`; no independent enable/reason/fallback |
| selected airspeed to TECS | `airspeed_validated` -> `FwLateralLongitudinalControl::updateAirspeed()` -> TECS update | CAS/EAS and TAS/CAS ratio | `PASS`, `FwLateralLongitudinalControl.cpp:397-412,616-641` |
| selected airspeed to guidance | `airspeed_validated` -> `FixedWingModeManager::airspeed_poll()` -> guidance | CAS used by mode/guidance | `PASS`, `FixedWingModeManager.cpp:160-180` |
| control outputs | mode manager -> lateral/longitudinal SP -> attitude SP -> rates SP -> torque/thrust -> allocator | source/setpoints/status topics | `PARTIAL`; execution chain exists, dataset logging rates incomplete |

## 数据流结论

`MS4525DO -> differential_pressure -> airspeed -> quality -> q/R -> EKF` 的计算路径存在。论文所需的顺序表述 `EKF wind -> selector` 需要谨慎：selector 主要选择/验证 airspeed source，并可用 wind 做 fallback；EKF wind 与 selector 之间不是单一同步调用。当前 quality 同时影响 EKF `noise_var/fuse_enabled` 和 selector 输出，不能在 VARIANCE_ONLY 与 FULL_PROPOSED 之间隔离。

## 启动边界

AS5600 和 wing_phase 已被目标 board 构建并启动。`RPM_CAPTURE` 虽被构建，但 `ROMFS/px4fmu_common/init.d/rcS:449-453` 仅在 `RPM_CAP_ENABLE>0` 时启动；参数默认 0，且静态未确认 board 的 RPM Input function 2070。因此 Hall-indexed phase 的正式可用性是 `NEEDS RUNTIME VERIFICATION`，默认状态判 `PARTIAL`。
