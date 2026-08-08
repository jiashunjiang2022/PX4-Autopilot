# RA-L 返修飞行代码就绪性审计：执行摘要

审计对象：`air` 分支，commit `650836c48937f654a0a81fa894b381ff5b130cc2`。审计开始时 `git status --short` 无输出，工作树干净。本文实现已进入传感器、EKF2、selector、logger 和自定义 uORB 消息，不是仅有参数或日志框架；但实验模式、频谱覆盖范围、Hall 启动和完整证据链仍不满足返修飞行要求，当前属于可开始返修、不可直接构建/台架/飞行的半成品。

## 就绪性结论

| Gate | 结论 | 原因 |
|---|---|---|
| `READY_TO_PATCH` | `true` | 实现边界和阻断项已可静态定位，可直接进入受控修补。 |
| `READY_TO_BUILD` | `false` | 本审计未获准编译，且 `FLAP_RATIO=7.5`、频谱 4 Hz 上限、模式隔离等 P0 尚未修复。 |
| `READY_FOR_BENCH` | `false` | 应先完成 P0、编译和静态/单元测试，再进行 producer rate、CPU、stack、SD 带宽验证。 |
| `READY_FOR_FLIGHT` | `false` | 正式 4--5.5 Hz 和平台 6.8 Hz 不受当前频谱算法可靠支持，实验模式和日志证据不完整。 |

## 最大的五个问题

1. **FAIL/P0：传动比错误。** 已确认真实值为 8.0，但 `src/drivers/encoder/as5600/module.yaml:18` 默认值、`AS5600.hpp:81` fallback、`src/modules/wing_phase/WingPhase.cpp:93` 初始化和 `WingPhaseMathTest.cpp:17,24` 测试仍为 7.5。参数运行时可能覆盖默认值，但正式配置值是 `NEEDS RUNTIME VERIFICATION`。
2. **FAIL/P0：频谱上限为 4 Hz。** `AirspeedQualityEstimator::update()` 在 `src/modules/ekf2/EKF2.cpp:376-391` 将搜索上限设为 `min(4.0, Nyquist-0.1)`，并把 flap center 钳位到该上限。4--5.5 Hz 正式区间和 6.8 Hz 平台最大频率会被映射到错误频点，不能可靠解释。
3. **FAIL/P0：频谱输入没有确定、均匀的采样合同。** `EKF2::UpdateAirspeedSample()` 可分别以约 10 Hz 的 `airspeed_validated.true_airspeed_m_s` publication timestamp 和约 20 Hz 的 `airspeed.true_airspeed_m_s` sample timestamp 调用同一 estimator（`EKF2.cpp:2755-2777,2862-2884,2926-2951`）。代码不重采样、不插值、无抗混叠滤波，却按窗口首末时间和样本数估算单一 `fs_hz`。
4. **NOT_IMPLEMENTED/P0：四种实验模式未实现。** 只有 boolean `EKF2_ASP_QLTY`（`src/modules/ekf2/params_airspeed.yaml:46-55`），无法独立得到 BASELINE、CONSTANT_R、VARIANCE_ONLY、FULL_PROPOSED，也没有模式枚举、冻结的 `R_const`、selector 独立使能或 ULog mode 字段。
5. **FAIL/P0：飞行证据链和 Hall 链未闭合。** flapping profile 未包含 raw differential pressure/airspeed、Hall event、airspeed aid source、TECS/NPFG 等完整证据；`hall_event` 在 logger 中完全不存在。`RPM_CAP_ENABLE` 默认 0（`src/drivers/rpm_capture/rpm_capture_params.c:43`），板级默认也未静态发现 RPM Input 配置。

## 频率结论

| 信号 | Producer 结论 | Logger 结论 | 判定 |
|---|---:|---:|---|
| MS4525DO `differential_pressure` | 成功周期约 12 ms，即约 83 Hz | default 1 Hz；flapping profile 未加入 | Producer 满足，ULog 不满足，`PARTIAL` |
| `airspeed` 中 IAS/TAS | `Sensors::diff_pres_poll()` 50 ms 平均窗，约 20 Hz | default 1 Hz；flapping profile 未加入 | Producer 仅满足最低值，ULog 不满足，`PARTIAL` |
| `airspeed_validated` | selector 固定 10 Hz | default 5 Hz，flapping full producer | Producer 低于 20 Hz 目标，`FAIL` |
| AS5600 count/RPM/flap frequency | 10 ms interval，约 100 Hz | flapping profile full producer | 满足目标，`PASS`，仍须实测掉样 |
| `wing_phase` | module 200 Hz 调度，仅新 encoder 时发布，约 100 Hz | flapping profile full producer | 频率满足，Hall 未启动时 phase 无效，`PARTIAL` |
| `ekf2_airspeed_quality` | 可能约 20--30 Hz 的混合事件流 | default 10 Hz，flapping full producer | 输入/producer 合同不确定，`PARTIAL` |

仅提高 `src/modules/logger/logged_topics.cpp` 中 interval 不会提高任何 producer 频率，也不会修复频谱输入、Nyquist、重采样或 4 Hz 上限。

## 在线频谱明确答复

- “仍以 10 Hz 运行”不能作为完整描述。`EKF2_ASP_SFS=10 Hz` 只参与最小样本数计算；实际 estimator 可能接收 10 Hz validated 和 20 Hz raw 事件，静态估计约 20--30 Hz，准确分布为 `NEEDS RUNTIME VERIFICATION`。
- 若实际只有 10 Hz，Nyquist 为 5 Hz；但即使实际输入更快，代码仍将上限硬限制到 4 Hz。
- 4--5.5 Hz 与 6.8 Hz flap center 会被 `constrain()` 到 4 Hz，不是空频带，而是继续测错误中心附近的能量，并同时承受混叠风险。
- 当前可宣称的理论可靠上限最多为 **4.0 Hz**；考虑非均匀混合采样和无抗混叠，飞行中的实际可靠范围还需台架验证。

## P0 清单

共 **7 项**：传动比 8.0 统一；频谱覆盖 6.8 Hz；建立单一 50 Hz、带抗混叠/重采样的输入合同；四模式隔离；selector 路径独立使能和原因/源/fallback 日志；补齐必需 ULog producer/topic 频率；启用并可靠记录 Hall 事件链。详见 `10_REQUIRED_PATCH_PLAN.md`。

## 审计边界

本次没有修改源码、参数默认值或 logger 配置，没有编译、仿真、replay、联网或安装依赖。结论为静态审计；producer 实测速率、EKF work queue WCET/stack、SD 写入带宽、正式参数快照和 Hall 引脚配置均标记为 `NEEDS RUNTIME VERIFICATION`。
