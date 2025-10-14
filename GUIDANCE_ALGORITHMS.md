# 固定翼制导算法对比

本项目实现了4种不同的固定翼路径跟踪制导算法，用于对比研究。

## 🎯 算法概述

| 算法 | 模式值 | 复杂度 | 特点 | 适用场景 |
|------|--------|--------|------|----------|
| **L1** | 0 | 中等 | PX4原生，经过充分验证 | 通用，默认推荐 |
| **Pure Pursuit** | 1 | 简单 | 经典几何算法，易于理解 | 教学、低速、大转弯半径 |
| **LOS** | 2 | 简单 | 鲁棒，带侧风补偿 | 直线段，有风环境 |
| **NPFG** | 3 | 复杂 | 先进非线性制导 | 高速、复杂路径 |

## 📚 算法详解

### 1. L1 Guidance（L1制导）

**原理**：基于L1自适应控制理论的制导算法

**核心公式**：
```
L1_distance = period * ground_speed / (2 * π)
lateral_accel = K * v² / L1_distance * sin(eta)
```

**优点**：
- ✅ 经过多年验证，非常稳定
- ✅ 自适应特性好
- ✅ 适用范围广

**缺点**：
- ❌ 参数调优相对复杂
- ❌ 理论理解门槛较高

**参数**：
- `FW_L1_PERIOD`：L1周期（默认25s）
- `FW_L1_DAMPING`：阻尼比（默认0.75）
- `FW_L1_ROLL_LIM`：最大滚转角限制（默认30°）

---

### 2. Pure Pursuit（纯追踪）

**原理**：追踪路径上前方固定距离的"胡萝卜"点

**核心公式**：
```
lookahead_distance = gain * ground_speed
curvature = 2 * sin(alpha) / lookahead_distance
lateral_accel = v² * curvature
```

**优点**：
- ✅ 算法简单，易于理解
- ✅ 计算量小
- ✅ 适合教学和入门

**缺点**：
- ❌ 不考虑风的影响
- ❌ 在高速或小转弯半径时性能下降
- ❌ 轨迹跟踪精度相对较低

**参数**（硬编码）：
- `lookahead_gain`：前视距离增益（默认2.0）
- `min_lookahead`：最小前视距离（默认10m）
- `max_lookahead`：最大前视距离（默认50m）

**适用场景**：
- 低速飞行（<20 m/s）
- 大转弯半径路径
- 教学演示

---

### 3. LOS (Line-of-Sight)（视线制导）

**原理**：基于视线角的路径跟踪，广泛应用于海洋船舶导航

**核心公式**：
```
course_correction = -atan(cross_track_error / lookahead_distance)
desired_course = path_course + course_correction
wind_correction = asin(crosswind / airspeed)  # 侧风补偿
```

**优点**：
- ✅ 简单鲁棒
- ✅ 有理论基础（Lyapunov稳定性）
- ✅ 内置侧风补偿
- ✅ 适合直线段跟踪

**缺点**：
- ❌ 在曲线路径上性能一般
- ❌ 需要准确的风速估计

**参数**（硬编码）：
- `lookahead_distance`：前视距离（默认20m）
- `max_course_error`：最大航向修正角（默认60°）
- `enable_wind_compensation`：启用侧风补偿（默认true）

**适用场景**：
- 直线段导航
- 有风环境
- 需要精确轨迹跟踪

---

### 4. NPFG (Nonlinear Path Following Guidance)（非线性路径跟踪制导）

**原理**：基于非线性控制理论的先进制导算法

**核心特性**：
- 自适应周期调整
- 考虑风速、空速、地速
- 轨迹可行性评估
- 适应性强

**优点**：
- ✅ 最先进的算法
- ✅ 高速性能好
- ✅ 自适应能力强
- ✅ 适合复杂路径

**缺点**：
- ❌ 参数多，调优复杂
- ❌ 计算量相对较大
- ❌ 理解门槛高

**参数**（PX4原生）：
- `NPFG_PERIOD`：标称周期（默认10s）
- `NPFG_DAMPING`：阻尼比（默认0.7）
- `NPFG_LB_PERIOD`：启用周期下界
- `NPFG_UB_PERIOD`：启用周期上界
- `NPFG_ROLL_TIME_CONST`：滚转时间常数
- `NPFG_SW_DST_MLT`：切换距离倍数
- `NPFG_PERIOD_SF`：周期安全系数

**适用场景**：
- 高速飞行
- 复杂曲线路径
- 需要最优性能

---

## 🧪 测试和对比

### 测试步骤

1. **编译代码**：
   ```bash
   cd ~/PX4/PX4-Autopilot
   git pull origin pid
   make clean
   make px4_sitl gazebo-classic
   ```

2. **在QGroundControl中切换模式**：
   - 打开参数设置
   - 找到 `FW_GUIDANCE_MODE`
   - 选择不同的值（0-3）
   - 重启飞行器

3. **测试场景**：
   - **直线段**：比较轨迹跟踪精度
   - **转弯**：比较转弯平滑度和响应速度
   - **有风环境**：比较抗风能力
   - **高速飞行**：比较稳定性

### 对比维度

| 维度 | L1 | Pure Pursuit | LOS | NPFG |
|------|----|--------------|----|------|
| 轨迹精度 | ⭐⭐⭐⭐ | ⭐⭐⭐ | ⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ |
| 响应速度 | ⭐⭐⭐⭐ | ⭐⭐⭐ | ⭐⭐⭐ | ⭐⭐⭐⭐⭐ |
| 抗风能力 | ⭐⭐⭐ | ⭐⭐ | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐ |
| 稳定性 | ⭐⭐⭐⭐⭐ | ⭐⭐⭐ | ⭐⭐⭐⭐ | ⭐⭐⭐⭐ |
| 易用性 | ⭐⭐⭐ | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐ | ⭐⭐ |
| 计算量 | ⭐⭐⭐ | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐ | ⭐⭐ |

---

## 🔧 故障排除

### L1模式问题
- **现象**：转弯过冲
- **解决**：增大 `FW_L1_PERIOD` 或增大 `FW_L1_DAMPING`

### Pure Pursuit问题
- **现象**：轨迹振荡
- **解决**：前视距离可能太小，需要在代码中调整 `_lookahead_gain`

### LOS问题
- **现象**：侧风下偏离轨迹
- **解决**：检查风速估计是否准确

### NPFG问题
- **现象**：完全失控
- **解决**：
  1. 检查参数是否正确设置
  2. 确保制导接口正确初始化
  3. 查看PX4控制台的错误信息

---

## 📖 参考文献

### L1 Guidance
- Park, S., Deyst, J., & How, J. P. (2004). "A new nonlinear guidance logic for trajectory tracking"

### Pure Pursuit
- Coulter, R. C. (1992). "Implementation of the pure pursuit path tracking algorithm"

### LOS Guidance
- Fossen, T. I. (2011). "Handbook of Marine Craft Hydrodynamics and Motion Control"
- Børhaug, E., Pavlov, A., & Pettersen, K. Y. (2008). "Integral LOS control for path following of underactuated marine surface vessels"

### NPFG
- Stastny, T., & Siegwart, R. (2019). "On Flying Backwards: Preventing Run-away of Small, Low-speed, Fixed-wing UAVs in Strong Winds"
- Stastny, T. (2020). "Low-Altitude Control and Local Re-Planning Strategies for Small Fixed-Wing UAVs" (Doctoral Thesis)

---

## 💡 建议

### 初学者
推荐顺序：**Pure Pursuit → L1 → LOS → NPFG**

### 实际应用
- **通用场景**：使用 **L1**（默认）
- **简单任务**：使用 **Pure Pursuit**
- **有风环境**：使用 **LOS**
- **高性能需求**：使用 **NPFG**（需要调优）

### 研究对比
建议在相同的测试场景下记录以下数据：
- 轨迹误差（RMS）
- 横向加速度（最大值和平均值）
- 控制输入变化率
- 能量消耗

---

## 🚀 下一步

1. **测试所有4种算法**
2. **记录性能数据**
3. **调优参数**
4. **撰写对比报告**

祝实验顺利！✈️

