# 四种实验模式隔离审计

总体状态：`NOT_IMPLEMENTED`。当前只有 `EKF2_ASP_QLTY` boolean，`0` 强制 nominal quality，`1` 同时打开 adaptive R、EKF quality gate 和 selector safeguard。没有模式 enum、`R_const`、variance-only switch 或 selector-quality enable。

## 当前控制面

- 参数定义：`src/modules/ekf2/params_airspeed.yaml:46-55`，`EKF2_ASP_QLTY` 仅允许 0/1。
- EKF caller：`EKF2::UpdateAirspeedSample()` 在 `src/modules/ekf2/EKF2.cpp:2755-2822,2862-2918,2929-2995` 根据该 boolean 运行 estimator 或强制 q=1/R=nominal。
- selector caller：`AirspeedSelector::Run()` 在 `src/modules/airspeed_selector/airspeed_selector_main.cpp:823-889` 直接消费 `ekf2_airspeed_quality`；没有独立 enable 参数。
- 参数由运行时 `parameter_update` 更新，未发现启动锁定、切换拒绝或 mode-change ULog event。

## Truth table

| Mode | Quality diagnostic | Adaptive R | Fixed R | Quality changes selector | Original PX4 validity retained | Expected selector behavior | Expected EKF behavior | Implemented? | Evidence |
|---|---|---|---|---|---|---|---|---|---|
| A BASELINE | 应运行但只诊断 | No | nominal PX4 R0 | No | Yes | 原 PX4 validator/selector/fallback | 原 nominal R 与原 fusion gate | `PARTIAL` | `EKF2_ASP_QLTY=0` 给 nominal R，但 estimator 不诊断运行且无 mode 日志；`EKF2.cpp:2782-2813,2953-2985` |
| B CONSTANT_R | 应运行但只诊断 | No | frozen `R_const` | No | Yes | 与 baseline 相同 | 始终使用试验指定常量 R | `NOT_IMPLEMENTED` | 无 mode enum、无 constant-R parameter、无 constant mapping |
| C VARIANCE_ONLY | Yes | Yes | No | No | Yes | 原 PX4 validator/selector/fallback | q 只改变 R | `NOT_IMPLEMENTED` | `EKF2_ASP_QLTY=1` 同时使 selector 消费 q；无 selector disable 参数 |
| D FULL_PROPOSED | Yes | Yes | No | Yes | 内部应保留并记录 | safeguard 可拒绝并选 fallback，记录原因 | q 改 R 和明确 gate | `PARTIAL` | adaptive R 和 safeguard 均存在，但频谱 FAIL，selector直接输出 NaN/disabled，原因/原始 validity/fallback 未记录；`EKF2.cpp:581-601`; selector `823-889` |

## 隔离判定

| 检查 | 状态 | 说明 |
|---|---|---|
| 单一 mode 参数/enum | `NOT_IMPLEMENTED` | 只有 boolean `EKF2_ASP_QLTY`。 |
| BASELINE 不受 q 影响 | `PARTIAL` | q 强制 1，R nominal；但无法保留 diagnostic estimator 输出用于同航线比较。 |
| CONSTANT_R 恒定 | `NOT_IMPLEMENTED` | 没有 `R_const` 参数或运行时 mode。 |
| VARIANCE_ONLY 禁止 Path B | `NOT_IMPLEMENTED` | selector quality path 无独立参数。 |
| FULL_PROPOSED 同时开启 A/B | `PARTIAL` | 两条路径存在，但 selector outcome/reason/fallback 不完整，频谱不可用于正式频率。 |
| 模式写入 ULog | `NOT_IMPLEMENTED` | `msg/Ekf2AirspeedQuality.msg` 没有 `experiment_mode`、`effective_r_mode`、`selector_quality_path_enabled`。 |
| 启动后模式冻结/记录 | `NOT_IMPLEMENTED` | 参数可动态更新，未发现 arming/start lock 或 transition event。 |
| 除目标处理外参数一致 | `UNKNOWN` | 没有统一 mode 配置层或参数快照对比工具；需飞行配置管理。 |

## 最小实现边界

1. 增加单一 `experiment_mode` enum，未知值 fail closed 到 BASELINE，并将 effective mode 每次发布到 ULog。
2. estimator 在四模式下都诊断运行，但其输出作用点由 mode 显式控制。
3. 将 `q -> R` 与 `q -> selector safeguard` 拆成独立布尔决策，由 mode truth table 唯一派生，禁止散落的参数组合。
4. 增加冻结的 `R_const` 参数及单位 `(m/s)^2` 或明确以 stddev 输入后平方；记录 configured 与 actual R。
5. 保留 original PX4 validity/source，另行记录 `quality_rejected`, `pre_quality_source`, `final_selected_source`, `rejection_reason`。
6. arming 后禁止模式改变，或至少发布 mode-change event 并使该 flight segment 不进入论文主结果。

四套不同代码不是要求；更安全的结构是一套 estimator、一个 enum 和两个显式作用开关，由单元测试验证 truth table，减少模式间非目标参数差异。
