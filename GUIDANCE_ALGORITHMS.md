# 固定翼制导算法对比

本项目提供了 **3 种** 固定翼路径跟踪制导算法，可通过参数 `FW_GUIDANCE_MODE`
在运行时自由切换，用于做 A/B 测试：

| 算法 | 模式值 | 主要参数 | 特点 | 适用场景 |
|------|--------|----------|------|----------|
| **NPFG** | 0 | `NPFG_*` 全套参数 | 官方默认的非线性路径跟踪，性能全面 | 高速 / 曲线密集任务的通用选择 |
| **L1** | 1 | `FW_L1_PERIOD`、`FW_L1_DAMPING` | 基于 L1 控制律，参数含义与 PX4 官方一致 | 需要复现/比较 PX4 原生 L1 行为时 |
| **PID** | 2 | `FW_PID_XTE_*` 系列参数 | 简单可解释的误差反馈控制器 | 需要快速调参或教学演示时 |

> **提示**：首段航线在 L1 模式下仍会退回 NPFG，以避免起飞后 180° 翻转问题。
> 正式对比时建议从第二段航线或 loiter 段开始采样。

---

## NPFG（模式 0）

**原理**：使用 PX4 原生 Nonlinear Path Following Guidance（NPFG）模块，考虑到风、车辆性能、路径曲率，自动调节周期与阻尼。

**可调参数**：
- `NPFG_PERIOD`、`NPFG_DAMPING`
- `NPFG_ROLL_TIME_CONST`、`NPFG_SW_DST_MLT`、`NPFG_PERIOD_SF`

**优势**：在高速、复杂航路下表现稳定，是 PX4 默认推荐方案。

---

## L1 Guidance（模式 1）

**原理**：恢复至 PX4 官方 L1 控制律实现，横向加速度由 `FW_L1_PERIOD` 和 `FW_L1_DAMPING`
推导得到，公式与 upstream 保持一致。

**可调参数**：
- `FW_L1_PERIOD`
- `FW_L1_DAMPING`
- `FW_L1_ROLL_LIM`

**使用建议**：
- 若需要和官方固件直接对比，请使用同一套 L1 参数。
- 如需更保守的入轨，可结合 `FW_L1_ROLL_LIM` 限制滚转。

---

## PID Guidance（模式 2）

**原理**：对横向误差 (`cross_track_error`) 使用 PID 控制，支持积分限幅、误差过零重置、
以及几何前视航向修正，避免出现蛇形摆动。

**可调参数**：
- `FW_PID_XTE_KP`、`FW_PID_XTE_KI`、`FW_PID_XTE_KD`
- `FW_PID_XTE_ILIM`（积分限幅）
- `FW_PID_XTE_MAXA`（最大横向加速度限制）

**使用建议**：
- `KP` 决定响应速度，`KD` 负责阻尼，`KI` 只在需要消除稳态误差时少量启用。
- 当误差跨越 0 或进入 ±0.5 m 内会自动清零积分，便于切换航线时快速收敛。

---

## 数据记录与对比

- `fixed_wing_lateral_guidance_status` 现在在三种算法下都会更新：
  - `course_setpoint` / `lateral_acceleration_ff` 反映当前期望航向与横向加速度。
  - `signed_track_error` 始终为最新横向误差。
  - 对于 NPFG，`bearing_feas*`、`track_error_bound`、`adapted_period` 等字段保留原始信息；
    当算法为 L1/PID 时，这些字段会填 `NaN`，可据此区分。
- `fixed_wing_lateral_status`（由纵向/侧向控制器发布）同样包含实际加速度与滚转指令，可与以上话题联合分析。
- 在 QGroundControl 中修改 `FW_GUIDANCE_MODE` 后即可实时对比三种算法的航迹表现。

---

## 测试流程示例

```bash
make px4_sitl gazebo-classic
# 启动后在 QGC 中依次设置 FW_GUIDANCE_MODE = 0 / 1 / 2
# 记录 fixed_wing_lateral_guidance_status 和 fixed_wing_lateral_status 对比控制效果
```

如需脚本化测试，可在 SITL 中预设同一条任务，循环修改 `FW_GUIDANCE_MODE` 并回放 ULog。

