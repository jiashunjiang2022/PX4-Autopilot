# 必需修改计划

本文件只定义计划，不包含代码修改。原则是先修数据合同和实验隔离，再补日志，最后做目标板验证。

## P0：正式飞行前必须修复

### P0-1 统一 `FLAP_RATIO=8.0`

- 目的：使 flap frequency、wing phase和Hall-indexed counts/cycle符合已确认机械结构。
- 当前问题：online/default/fallback/tests仍使用 7.5。
- 文件/函数：`src/drivers/encoder/as5600/module.yaml:18`; `AS5600.hpp:81`; `AS5600::update_flap_ratio_param()`；`src/modules/wing_phase/WingPhase.cpp:93,121`; `WingPhaseMathTest.cpp:17,24`。
- 建议修改：所有机械比默认/fallback/test expectation改为 8.0；启动时发布effective ratio；不要替换仓库无关的数值7.5。
- 依赖：确认正式参数文件/SD 参数没有保留旧值。
- 验证方法：parameter metadata test、AS5600 RPM-to-frequency test、8 cycle phase/Hall zero test、参数快照检查。
- 风险：旧机体若使用不同机械比需板/airframe override，不能全平台硬编码。

### P0-2 频谱可靠覆盖 0--6.8 Hz

- 目的：正式 4--5.5 Hz 和平台 6.8 Hz均在有效频带内。
- 当前问题：`upper_limit_hz` 硬限制4 Hz，高中心被钳到错误频点。
- 文件/函数：`src/modules/ekf2/EKF2.cpp:376-400`, `AirspeedQualityEstimator::update()`；`params_airspeed.yaml`。
- 建议修改：参数化 reference lower/upper bounds，明确 flap band edge policy；无法覆盖时窗口显式invalid而不是静默钳位；50 Hz input下上限至少覆盖6.8 Hz并留滤波过渡带。
- 依赖：P0-3 的统一采样合同。
- 验证方法：0--8 Hz tone sweep、band-edge/noise/multi-tone tests，与离线 reference比对。
- 风险：扩大reference band会改变ratio标定，需重新冻结q thresholds。

### P0-3 建立单一、均匀的50 Hz频谱输入

- 目的：给在线频谱确定的source、Fs、timestamp与anti-alias合同。
- 当前问题：raw约20 Hz与validated约10 Hz可能混入同一ring，时间戳语义不同，无resampling/anti-alias。
- 文件/函数：`EKF2::UpdateAirspeedSample()` (`EKF2.cpp:2673-3036`); estimator ring (`EKF2.cpp:150-436`, `EKF2.hpp:199-259`); airspeed producer as needed。
- 建议修改：选定唯一TAS/raw pressure-derived signal；用sample timestamp；anti-alias后重采样到50 Hz；定义gap/duplicate/held策略；记录source/measured Fs。
- 依赖：确认最适合论文定义的物理输入；producer保持至少20 Hz时，禁止用插值制造不存在的信息而声称50 Hz传感器带宽。
- 验证方法：jitter/drop/duplicate/source-switch tests、measured Fs ULog、hardware-in-loop timestamp audit。
- 风险：若原producer仅20 Hz，50 Hz interpolation只提供uniform grid而不增加带宽；论文必须准确描述。

### P0-4 实现四模式单一 truth table

- 目的：得到可复现实验A/B/C/D，唯一差异是指定作用路径。
- 当前问题：boolean同时控制adaptive R、fusion gate和selector；Constant-R/Variance-only不存在。
- 文件/函数：`src/modules/ekf2/params_airspeed.yaml`; `EKF2::UpdateAirspeedSample()`; `AirspeedSelector::Run()`; quality/selector uORB schema。
- 建议修改：enum `experiment_mode`; estimator总是诊断；由mode派生`adaptive_r_enabled`和`selector_quality_enabled`; 增加冻结`R_const`; armed后锁定并记录mode。
- 依赖：P0-5 selector outcome schema。
- 验证方法：四行truth-table unit/integration tests，参数快照与effective mode一致性test。
- 风险：模式运行时切换污染数据；未知enum必须回退BASELINE。

### P0-5 隔离selector safeguard并记录原因/fallback

- 目的：让Path A与Path B独立，区分PX4原始invalid与quality拒绝。
- 当前问题：safeguard无独立enable，直接发布NaN/disabled且不自动fallback，无pre/final source和reason。
- 文件/函数：`src/modules/airspeed_selector/airspeed_selector_main.cpp:673-889`; `msg/versioned/AirspeedValidated.msg`或新status message；logger。
- 建议修改：保持validator原状态；新增outcome topic及reason enum；明确fallback policy；VARIANCE_ONLY完全旁路Path B，FULL_PROPOSED显式启用。
- 依赖：P0-4 mode enum。
- 验证方法：original invalid、quality reject、timeout、source unavailable、fallback success/fail矩阵test；TECS输入检查。
- 风险：错误fallback可导致控制突变；需迟滞、dwell和source transition测试。

### P0-6 建立返修flight ULog profile与producer合同

- 目的：以可承受带宽记录 sensor -> q/R -> EKF -> selector -> TECS/NPFG -> actuator链。
- 当前问题：flapping profile遗漏raw pressure/airspeed、Hall、aid source、TECS/NPFG/status；若只改logger仍不能提高10 Hz selector producer。
- 文件/函数：`src/modules/logger/logged_topics.cpp:383-410`; relevant producers; messages。
- 建议修改：加入缺失topics/instances；设置目标interval；对低频producer分别修改或如实接受；加入sample timestamp/update counter；生成静态bandwidth budget。
- 依赖：P0-3/P0-5 schema；目标SD卡与board性能。
- 验证方法：producer vs ULog measured-rate table、dropout/queue/SD soak、timestamp alignment script。
- 风险：full-rate topic组合导致SD dropout；必须以实测而非配置interval宣称频率。

### P0-7 启用并可靠保存Hall事件链

- 目的：每个机械zero edge可追踪，用于phase校正和编码器噪声区分。
- 当前问题：`RPM_CAP_ENABLE`默认0，board pin未确认；ISR flag coalescing、默认queue和未记录topic可丢边沿。
- 文件/函数：`src/drivers/rpm_capture/RPMCapture.cpp:108-172`; `rpm_capture_params.c:43`; board/airframe config; `msg/HallEvent.msg`; `WingPhase::Run()`; logger。
- 建议修改：正式board配置RPM Input和startup；ISR-safe edge ring/counter；增加state/period/frequency/lost count；event logger与phase correction reason。
- 依赖：真实Hall polarity、每cycle脉冲数和pin mapping。
- 验证方法：signal generator burst test、100% edge/accounting test、encoder-vs-Hall frequency/phase bench。
- 风险：ISR负载和queue overflow；必须有overrun counter和安全invalid状态。

## P1：强烈建议返修前修复

### P1-1 完整质量算法测试与可观测字段

- 目的：可重建每次q变化并防止算法回归。
- 当前问题：无专用unit test；缺timestamp_sample、actual Fs、band limits、update counter、R0和mode。
- 文件/函数：`EKF2.cpp/.hpp`; `msg/Ekf2AirspeedQuality.msg`; test CMake/files。
- 建议修改：补字段和deterministic tests；记录raw delta/temporal component，或将论文符号与现实现明确映射。
- 依赖：P0-2/P0-3/P0-4最终接口。
- 验证方法：golden vectors、ULog reconstruction equality。
- 风险：消息变更影响ulog/bridge consumers，需versioning。

### P1-2 修复window容量、early-boot和unused参数

- 目的：参数语义与内存一致，移除时间边界缺陷。
- 当前问题：256 samples无法支持50 Hz*10 s；早期减法下溢；`EKF2_ASP_TW`未使用。
- 文件/函数：`EKF2.hpp:211`; `EKF2.cpp:317-318`; `params_airspeed.yaml:56-85`。
- 建议修改：限制window或重设容量/存储；饱和time bound；删除/迁移legacy参数并提供release note。
- 依赖：P0-3 Fs/window决策。
- 验证方法：boundary tests N=0/1/200/256/500、boot < window、parameter metadata test。
- 风险：增大member/stack影响EKF内存；优先避免重复local arrays。

### P1-3 修复MS4525DO双读验证

- 目的：避免接受不一致pressure read并提高probe可靠性。
- 当前问题：Run比较缺pressure LSB；probe有恒真自比较。
- 文件/函数：`src/drivers/differential_pressure/ms4525do/MS4525DO.cpp:73,175-176`。
- 建议修改：完整比较status/pressure/temp字段并明确定义stale semantics；添加error计数原因。
- 依赖：MS4525DO datasheet行为与I2C bench。
- 验证方法：mock normal/stale/mismatch/CRC-like vectors及目标sensor bench。
- 风险：过严比较可能拒绝合法转换状态，须按datasheet设计。

### P1-4 目标板CPU/stack/SD预算

- 目的：确认50 Hz输入和完整日志不阻塞EKF或掉日志。
- 当前问题：只有静态O(BN)和约3+3 KiB估算，无WCET/high-water/bandwidth数据。
- 文件/函数：EKF2 estimator；logger profile；性能测试文档/scripts。
- 建议修改：增加可重复bench和acceptance thresholds，不一定改flight code。
- 依赖：P0实现完成、目标board/SD卡。
- 验证方法：worst-case bins/window、work queue deadline、stack high-water、30+ min SD soak。
- 风险：测量instrumentation本身扰动时序，应分级测试。

## P2：可保留或论文说明

### P2-1 Window function和算法选择说明

- 目的：让reviewer理解为何使用mean-detrended rectangular-window Goertzel而非Welch/STFT。
- 当前问题：泄漏与resolution tradeoff未在实现文档量化。
- 文件：algorithm docs/analysis validation；必要时参数文档。
- 建议修改：对Hann/rectangular、window length和bin spacing做离线sensitivity分析；代码可保持Goertzel。
- 依赖：飞行输入合同冻结。
- 验证方法：synthetic与bench data comparison。
- 风险：改window会改变阈值；若不改应在论文限制中说明。

### P2-2 保持已足够的高频控制topic

- 目的：控制SD带宽。
- 当前问题：vehicle state/actuator producer本已高频，无需追求虚假更高logger rate。
- 文件：`src/modules/logger/logged_topics.cpp` profile only。
- 建议修改：attitude/angular velocity保留现有producer；actuator在带宽压力下可上限100 Hz。
- 依赖：P1-4 bandwidth test。
- 验证方法：控制链time alignment和dropout率。
- 风险：过度降采样可能丢flap-cycle harmonics，需基于目标分析带宽选择。

## 不需要修改的核心区域

- EKF `Ekf::updateAirspeed()` 对 `noise_var` 的直接使用和 aid source innovation/test-ratio/fused字段已经正确接线；除新增mode/diagnostic需要外，不建议重写Kalman update。
- AS5600 10 ms调度、rollover、raw/filtered RPM和约100 Hz发布结构可保留，只修ratio和验证。
- 控制链TECS/NPFG/attitude/rate/allocator的基本topic与调用关系存在，主要工作是selector semantics和logger profile，不应无关重构控制律。
- logger interval不能替代producer修改；对已经满足的producer只需配置和带宽验证。
