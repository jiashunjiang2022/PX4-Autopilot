# 参数与 uORB Schema 审计

## 参数审计

| 需求 | 当前参数 | 类型/默认/范围/单位 | 使用位置 | 状态 |
|---|---|---|---|---|
| mechanical ratio | `FLAP_RATIO` | float, 7.5, 0.01--1000 | AS5600 frequency；wing phase counts/cycle | `FAIL`，已确认应为 8.0；`src/drivers/encoder/as5600/module.yaml:6-18` |
| spectral expected rate | `EKF2_ASP_SFS` | float, 10, 2--50 Hz | minimum sample count，非 scheduler/decimator | `FAIL`，名称/文档易被误解；`params_airspeed.yaml:65-75`, `EKF2.cpp:336` |
| spectral window | `EKF2_ASP_SWIN` | float, 4, 1--10 s | ring pruning/eval | `PARTIAL`，10 s 与 256 sample cap 不兼容于 50 Hz |
| legacy window | `EKF2_ASP_TW` | float, 2, 0.5--5 s | 参数绑定但未用于算法 | `FAIL` unused；yaml `56-64`, `EKF2.cpp:712`, `EKF2.hpp:684-685` |
| reference min/max | none; constants 0.5/4.0 Hz | compile-time | estimator frequency grid | `FAIL`，max 4 Hz 阻断正式频率 |
| flap half-width | `EKF2_ASP_DF` | float, 0.5, 0.1--2 Hz | flap band | `PASS` |
| temporal lambda/D0/phi | no matching parameters | none | fixed alpha 0.2 + `EKF2_ASP_DV0` | `NOT_IMPLEMENTED` as paper notation |
| temporal normalization | `EKF2_ASP_DV0` | float, 5, 0.1--20 m/s^2 | normalized filtered `dv` | `PARTIAL` |
| quality weights | `EKF2_ASP_QA/QB` | float, 0.7/0.3, 0--1 | `q_raw` mapping | `PASS`，但未静态发现 sum constraint |
| q smoothing | `EKF2_ASP_QTAU` | float, 0.25, 0--2 s | physical-dt low-pass | `PASS` |
| R0 | `EKF2_EAS_NOISE` | float stddev, 1.4, 0.5--5 m/s | squared after EAS-to-TAS conversion | `PASS` |
| R multiplier | `EKF2_ASP_RMAX` | float, 5, 1--20 | q-to-R mapping | `PASS` |
| q hysteresis | `EKF2_ASP_QOFF/QON` | float, 0.4/0.6 | EKF fusion gate | `PASS/PARTIAL`，ordering constraint需 test |
| persistence | `EKF2_ASP_TOFF/TON/THLD` | 0.20/0.30/0.25 s | gate hysteresis/hold | `PASS` |
| flap state/timeout | `EKF2_FLAP_F_ON/OFF`, `T_ON/OFF/T_TO` | yaml `180-224` | estimator state | `PASS` |
| experiment mode | none | none | none | `NOT_IMPLEMENTED` |
| constant-R value | none | none | none | `NOT_IMPLEMENTED` |
| selector-quality enable | none | none | selector always consumes quality when present | `NOT_IMPLEMENTED` |

PX4 parameter snapshot normally进入 ULog，但“参数存在于快照”不等于 effective mode 被记录；派生的两个作用开关、实际 R mode 和 mode transition 仍需 topic/event。除明确标记 `reboot_required` 的参数外，EKF2 参数通过运行时 update；当前未发现 mode 启动冻结。

## 7.5/8.0 一致性

与本文机械比直接相关的 7.5 引用如下：

- `src/drivers/encoder/as5600/module.yaml:18`：`FLAP_RATIO` default 7.5。
- `src/drivers/encoder/as5600/AS5600.hpp:81`：driver fallback `_flap_ratio{7.5f}`。
- `src/modules/wing_phase/WingPhase.cpp:93`：`_counts_per_cycle` 初始 `4096*7.5`。
- `src/modules/wing_phase/WingPhaseMathTest.cpp:17,24`：test vectors 使用 7.5。

`AS5600::update_flap_ratio_param()` 在 `AS5600.cpp:88-99` 成功读取参数后会覆盖 fallback，`RunImpl()` 在 `AS5600.cpp:208-210` 用该值从 filtered RPM 换算 flap frequency；`WingPhase::Run()`/parameter update 在 `WingPhase.cpp:121` 计算 counts/cycle。因此默认错误会同时影响 frequency 与 phase。其他通用仓库中数值 7.5 不是机械比，不应机械替换。

## 自定义 uORB 消息

相关消息已列入 `msg/CMakeLists.txt`：`Ekf2AirspeedQuality.msg`, `EncoderCount.msg`, `FlapFrequency.msg`, `HallEvent.msg`, `WingPhase.msg`。未发现为这些 topic 设置自定义 queue length；按默认单槽语义评估，事件 burst 可能覆盖。

### `ekf2_airspeed_quality`

`msg/Ekf2AirspeedQuality.msg:1-42` 已有 publication `timestamp`、q/raw/smoothed、`r_as_used`、fuse gate、flap/spectral/gap/gate diagnostics。缺少：

- `timestamp_sample`；
- spectral update counter/明确 updated flag；
- measured spectral input rate 和 input source；
- actual flap-band center、reference lower/upper limits；
- nominal R0、experiment mode、effective R mode、selector path enable；
- producer/update sequence，用于跨 topic 与 EKF aid source 对齐。

当前 `do_eval` 近似一次更新标志，但 held result 与 publication cadence 仍需要 counter 才能无歧义分析。

### `airspeed_validated` / selector status

`msg/versioned/AirspeedValidated.msg` 有 selected source 与 IAS/CAS/TAS/validity 类字段，但没有 original pre-quality source/validity、quality rejection、fallback outcome 和 rejection reason。建议新增独立 selector status topic，避免破坏 versioned 主消息 ABI，并明确枚举：`ORIGINAL_PX4_INVALID`, `QUALITY_SAFEGUARD`, `SENSOR_TIMEOUT`, `SOURCE_UNAVAILABLE`, `FALLBACK_SELECTED`, `OTHER`。

### Hall/phase

- `msg/HallEvent.msg` 只有 `timestamp` 和 `pulse_count`，缺 raw Hall state、edge polarity、previous-edge period、Hall-derived frequency、overrun/lost count。
- `msg/WingPhase.msg` 提供 phase/unwrapped phase/zero information，但缺 Hall correction reason、timeout/status reason、update counter。
- `RPMCapture` ISR 在 `src/drivers/rpm_capture/RPMCapture.cpp:162-172` 若 `_interrupt_happened` 已为 true，只增加 error count；`Run()` 每次只把 pulse count 加一并发布（`108-115,154-159`）。因此单槽 flag 和默认 uORB queue 都可能丢边沿，不能区分编码器噪声与真实周期变化。

## 多实例与 timestamp

`differential_pressure`、EKF aid/status 和 `airspeed_wind` 是 multi-instance logger topic；flight profile 必须明确实例数量并保留每实例 device/source identity。MS4525DO 的 `timestamp_sample` 是 conversion 启动时间，publication `timestamp` 是 publish time；`airspeed.timestamp_sample` 是 50 ms window 内 source timestamp 平均值。quality message只有 publication timestamp，跨 topic 因此不够严谨。
