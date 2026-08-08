# EKF Covariance 与 Selector 双路径审计

## Path A：q -> R_as -> EKF fusion

状态：covariance wiring `PASS`，实验隔离 `FAIL`。

1. `AirspeedQualityEstimator::update()` 计算 q，并在 `src/modules/ekf2/EKF2.cpp:581-601` 计算：
   - `R_min = sq(EKF2_EAS_NOISE * eas2tas)`；
   - `R_max = R_min * EKF2_ASP_RMAX`；
   - `R_used = R_min * (1 + (1-q)*(rmax-1))`。
2. Caller `EKF2::UpdateAirspeedSample()` 将该值写到 `airspeedSample.noise_var`（validated/raw 路径分别为 `EKF2.cpp:2814-2822,2987-2995`），再调用 `_ekf.setAirspeedData()`。
3. Callee `Ekf::controlAirDataFusion()` 和 `Ekf::updateAirspeed()` 在 `src/modules/ekf2/EKF/aid_sources/airspeed/airspeed_fusion.cpp:94-126,169-243` 直接使用 sample `noise_var` 作为 observation variance R，并写入 aid source 后执行 Kalman update。
4. `r_as_used` 因此就是送入 EKF 的 variance，单位 `(m/s)^2`，不是标准差。nominal R0 来自 `EKF2_EAS_NOISE` 标准差乘 EAS-to-TAS 后平方。
5. 未发现之后再次独立缩放该 R 的 PX4 airspeed 逻辑；innovation gate 使用 aid source 的 observation variance。该结论针对当前调用链。

质量同时控制 `_airspeed_quality_state.fuse_enabled`。`Ekf::controlAirDataFusion()` 会使用该 gate，所以当前不是“只改变 covariance”。另外 `EKF2_ARSP_THR` 默认 0（`src/modules/ekf2/params_airspeed.yaml:5-15`），默认 PX4 不融合 airspeed；正式参数必须为 `NEEDS RUNTIME VERIFICATION`。

## EKF 可观测日志

`msg/EstimatorAidSource1d.msg:1-27` 已包含 `observation`, `observation_variance`, `innovation`, `innovation_variance`, `test_ratio`, `innovation_rejected`, `fused` 等字段，足以描述单次 aid test 和是否融合。`estimator_aid_src_airspeed` 仅在 `EKF2_LOG_VERBOSE=1` 且 `EKF2_ARSP_THR>0` 时 advertise（`src/modules/ekf2/EKF2.cpp:867-880`），logger default 又只请求 10 Hz（`logged_topics.cpp:191`）。

`ekf2_airspeed_quality.r_as_used` 与 aid source 可跨 topic 对齐，但 quality 消息没有 estimator sample timestamp、nominal R0 或 mode，不能无歧义证明同一 EKF sample。状态为 `PARTIAL`。

## Path B：q -> safeguard -> selector -> controller

状态：`FAIL`。

- Caller `AirspeedSelector::Run()` 在 `src/modules/airspeed_selector/airspeed_selector_main.cpp:823-889` 消费 `ekf2_airspeed_quality.airspeed_q/fuse_enabled`。
- q 低于约 0.5、gate closed、q invalid/stale 可以 latch quality disable；freshness 1 s、hold 2 s、reenable dwell 1 s。
- safeguard 直接将 outgoing IAS/CAS/TAS 置 NaN，并把 source 设为 `SOURCE_DISABLED`（`airspeed_selector_main.cpp:872-877`）。它没有修改 `AirspeedValidator` 内部的 original validity，因此内部原状态仍保留，但没有发布出来。
- quality disable 后没有自动走 ground-minus-wind fallback；`apply_airspeed_fallback()` 只在 blockage 分支被调用（`airspeed_selector_main.cpp:885-886`）。因此“quality invalidation -> fallback”并未完整实现。
- `airspeed_validated` 进入 `FwLateralLongitudinalControl::updateAirspeed()` 和 `FixedWingModeManager::airspeed_poll()`，所以 Path B 可直接影响 TECS、guidance 和 CAS rate scaling。

## 两路径是否可分开

| Capability | 状态 | 证据/缺口 |
|---|---|---|
| q 只改变 EKF R | `NOT_IMPLEMENTED` | selector 无独立 enable；`EKF2_ASP_QLTY=1` 同时打开两边。 |
| q 只用于 selector | `NOT_IMPLEMENTED` | estimator R mapping/gate 与 q 绑定。 |
| Constant R + original selector | `NOT_IMPLEMENTED` | 无 R_const/mode。 |
| adaptive R + original selector | `NOT_IMPLEMENTED` | 即论文 VARIANCE_ONLY 缺失。 |
| adaptive R + safeguard | `PARTIAL` | 代码存在但证据、fallback、频谱覆盖不满足。 |

## Validity、source 与 reason

建议的拒绝枚举 `ORIGINAL_PX4_INVALID`, `QUALITY_SAFEGUARD`, `SENSOR_TIMEOUT`, `SOURCE_UNAVAILABLE`, `FALLBACK_SELECTED`, `OTHER` 或等价信息均未实现。现有 `msg/versioned/AirspeedValidated.msg` 未提供 pre-quality source、original validity、quality rejection 和 final reason；`Ekf2AirspeedQuality.msg` 中 gate reason 是 EKF quality gate 原因，不等价于 selector outcome。

必须补充并同时记录：`pre_quality_source`, `original_valid`, `quality_rejected`, `rejection_reason`, `fallback_attempted`, `fallback_source`, `final_selected_source`, `selector_quality_path_enabled`。这使 reviewer 能区分 PX4 原生 invalid、论文 safeguard 拒绝与传感器超时。
