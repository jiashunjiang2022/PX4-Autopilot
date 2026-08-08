# TECS、NPFG 与控制证据链审计

总体状态：执行链 `PASS/PARTIAL`，返修 ULog 证据链 `FAIL`。

## Selected airspeed 的使用者

| Consumer | caller/callee | 使用字段 | 证据 | 状态 |
|---|---|---|---|---|
| FW longitudinal/TECS | `FwLateralLongitudinalControl::updateAirspeed()` -> TECS update | selected calibrated/equivalent airspeed；TAS/CAS ratio | `src/modules/fw_lateral_longitudinal_control/FwLateralLongitudinalControl.cpp:397-412,616-641` | `PASS` |
| mode manager/guidance | `FixedWingModeManager::airspeed_poll()` -> mode/guidance | selected CAS | `src/modules/fw_mode_manager/FixedWingModeManager.cpp:160-180` | `PASS` |
| attitude scaling | `FixedwingAttitudeControl::Run()` | selected CAS for scaling | `src/modules/fw_att_control/FixedwingAttitudeControl.cpp:144-167` | `PASS` |
| rate scaling | `FixedwingRateControl::Run()` | selected CAS for gain/scaling | `src/modules/fw_rate_control/FixedwingRateControl.cpp:153-180` | `PASS` |

NPFG/DirectionalGuidance 不是简单直接订阅一个 airspeed topic；`FixedWingModeManager` 将 selected airspeed、wind 和 local-position/navigation 状态带入 guidance 计算。quality selector 切换或输出 NaN 会先改变 mode manager/controller 可用的 airspeed，再影响 TECS、guidance 约束和 rate scaling。

## TECS 证据

`FwLateralLongitudinalControl` 在 `FwLateralLongitudinalControl.cpp:443-477` 发布 `tecs_status`。`msg/TecsStatus.msg` 提供 airspeed/altitude setpoint 与 estimate、能量误差/速率、pitch/throttle setpoint 以及 mode/limit 状态的主要字段，静态上可回答 TECS 响应。

缺口是 logger：default 仅 200 ms，即 5 Hz（`src/modules/logger/logged_topics.cpp:132`），flapping dataset 没有加入该 topic。要求最低 20 Hz、推荐 50 Hz，因此飞行证据为 `FAIL`。

## NPFG/横向制导证据

- `FixedWingModeManager` local-position 更新间隔为 20 ms，典型上限约 50 Hz。
- `FixedWingModeManager.cpp:2720-2734` 发布 guidance status，包括 course setpoint、lateral acceleration feed-forward、bearing feasibility、track error、adapted period 和 wind validity。
- `FwLateralLongitudinalControl.cpp:289-294` 发布 lateral status，包括 resultant lateral acceleration/can-run 状态。
- default logger 对 `fixed_wing_lateral_guidance_status` 和 `fixed_wing_lateral_status` 仅 100 ms/10 Hz（`logged_topics.cpp:161-162`），flapping dataset 均遗漏。

path tangent/path setpoint 的完整几何字段需结合 `fixed_wing_lateral_setpoint` 与 guidance status；不是所有论文希望字段都在一个 topic。当前 topic 组合静态可重建大部分链，但不能在现有 dataset profile 下以 20--50 Hz 记录，状态 `PARTIAL/FAIL`。

## 姿态、角速率与执行器

调用链为：

`FixedWingModeManager` -> `fixed_wing_lateral_setpoint`/`fixed_wing_longitudinal_setpoint` -> `FwLateralLongitudinalControl` -> `vehicle_attitude_setpoint` -> `FixedwingAttitudeControl` -> `vehicle_rates_setpoint` -> `FixedwingRateControl` -> `vehicle_torque_setpoint`/`vehicle_thrust_setpoint` -> `ControlAllocator::Run()` -> `actuator_motors`/`actuator_servos`。

`FwLateralLongitudinalControl.cpp:312-318` 发布 attitude setpoint；`ControlAllocator.cpp:692-757` 发布 actuator outputs。现有 vehicle attitude/angular velocity producer 已为 100--200 Hz 量级，无需提高 producer。flapping dataset 对 attitude、angular velocity、torque/thrust、motors/servos 是 full producer；但 attitude/rate setpoint 依赖 DEFAULT profile 的 20/50 Hz 配置，应明确强制 profile 组合或显式加入。

## Reviewer 时间链可回答性

| Link | 当前计算/消息 | 当前 ULog | 判定 |
|---|---|---|---|
| q -> R_as | `ekf2_airspeed_quality.airspeed_q/r_as_used` | flapping full rate | `PARTIAL`，缺 nominal R0/mode/sample timestamp |
| R_as -> innovation/fused | `estimator_aid_src_airspeed` | default 10 Hz；profile 遗漏；参数条件 advertise | `FAIL` |
| fusion -> wind | `wind`, estimator status | wind full in flapping；fusion status不完整 | `PARTIAL` |
| quality -> selected source | only final `airspeed_validated` | 无 selector outcome/reason topic | `FAIL` |
| selected airspeed -> TECS | `airspeed_validated`, `tecs_status` | TECS 5 Hz/profile 遗漏 | `FAIL` |
| selected airspeed/wind -> NPFG | guidance/lateral status/setpoint | status 10 Hz/profile 遗漏 | `FAIL` |
| NPFG/TECS -> attitude SP | setpoint topics | profile组合依赖 | `PARTIAL` |
| attitude/rates -> actuator | vehicle state/SP + allocator outputs | 多数可高频记录 | `PASS/PARTIAL` |
| actuator -> tracking error | actuator + attitude/rate/guidance errors | topic存在但 rate/profile需统一 | `PARTIAL` |

所以现有 topic 集合能形成大部分物理链，但不能形成审稿所需的无歧义实验链。必须补 selector outcome/mode 字段，并将 aid source、TECS、NPFG、source/status 与 raw sensor 明确加入同一个经带宽验证的 flight profile。
