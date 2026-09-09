# raodong 分支：官方空速逻辑与旁路日志

从 `air` 的 `c9af571575` 创建。官方对照基线为
`ec278758eda642005a479e2f8d278f2be8795eaf`，不是持续跟踪最新官方 main。

## 飞行行为

- EKF 空速输入来源、数值、时间戳、缓存接口、观测噪声和融合条件恢复官方基线。
- 空速 selector 的校验、选择、备用来源和调度恢复官方基线。
- 删除质量自适应 R、常数 R 实验、质量融合开关、selector 质量锁存和自定义堵塞回退。
- 删除 A/B/C/D/E 参数预设和实验控制参数：`EKF2_ASP_MODE`、`EKF2_ASP_RCST`、
  `EKF2_ASP_RMAX`、`EKF2_ASP_QON/QOFF`、`EKF2_ASP_TOFF/TON/THLD`、`ASPD_QBLK_EN`。
- 官方的 `EKF2_ARSP_THR`、`EKF2_EAS_NOISE`、`EKF2_TAS_GATE`、`FW_USE_AIRSPD`、
  `ASPD_PRIMARY`、`ASPD_FALLBACK` 等参数继续工作。

## 保留功能

AS5600 扑翼频率、Hall 校准翼相位、RPM 捕获、扑翼与 PID 日志、SITL 空速注入器、
dataman 队列诊断、自动模式接管等待期间的控制/油门保持、TECS 俯仰配平修正、
滚转限制以及现有板级配置保持不变。

传感器端 50 Hz 质量输入、抗混叠滤波、空速变化率、频谱和质量分数继续用于日志。
`UpdateAirspeedQualityMonitoring()` 独立消费 `airspeed_quality_input` 和 `flap_frequency`，
不读写飞行空速订阅，也不调用 EKF 接口。质量低、质量输入失效或扑翼频率超时均不会
通过这条旁路改变飞行空速、噪声或融合条件。官方自身的异常校验仍然有效。

## 日志语义

- `airspeed_quality_input` 保留原有数据与频率。
- `ekf2_airspeed_quality` 保留 q、频谱、扑翼状态、源设备和输入质量诊断；
  全部记录为 `qmon=true`。使用 `quality_timestamp_sample` 对齐采样时间。
  `timestamp_sample`、`ekf_buffer_timestamp_sample` 为 0；`airspeed_source=-1`；
  `eas2tas`、`nominal_r_as`、`r_as_used` 为 NaN，表示这不是 EKF 观测记录。
  实际融合状态、观测方差和创新查看 `estimator_aid_src_airspeed` 及 EKF 状态日志。
- `airspeed_selector_quality_status` 保留 topic 名，改为官方选择结果的被动记录：
  最终来源、设备 ID、采样时间、有效性，以及是否选用了有效地速减风速/合成空速。
  不再存在实验模式、质量拒绝和自定义堵塞字段。
- `rate_ctrl_terms`、`wing_phase` 等日志及 `SDLOG_PROFILE` 配置保持原样，见
  [扑翼控制日志采集](flapping-logging.md)。

两种质量诊断消息移除了实验字段；旧的实验分析脚本需要按新 schema 调整。
`docs/ral_revision/` 和 `artifacts/ral_revision_*` 中已有的报告、测试结果及证据工具是
air 分支历史材料，不是 raodong 的验收结果；不要用它们的五模式参数检查器验收此分支。

## 已有飞控参数

刷固件不会清除已保存的官方参数。尤其此前使用 E 预设时，`FW_USE_AIRSPD=0` 和
`EKF2_ARSP_THR=0` 仍会保留，它们现在仍分别关闭控制器空速使用和 EKF 空速融合。
本分支不会在启动时强制改写这些参数，也不会清除标定和控制器参数。
部署前按飞机实际配置核对上述官方参数；不要继续加载旧 A～E 预设。

## 源码验收

`python3 Tools/test_raodong_airspeed.py` 对照固定 Git 基线检查官方空速核心、输入函数、
selector 选择函数、诊断隔离、实验参数删除，并检查要求保留的模块未被改变。
固件构建目标仍为 `px4_fmu-v6c_default`，SITL 为 `px4_sitl_default`。

本次验证：两个目标均编译通过；7 项源码边界检查通过；原有 EKF 空速测试 5 项通过；
质量频谱/诊断、质量输入、PID、AS5600 数学、Hall 相位数学和自动接管测试共 37 项通过。
C++ 测试在 macOS 使用 GoogleTest v1.16.0 独立构建，EKF 测试链接本次 SITL 构建的库。
未刷写实机或执行飞行试验。
