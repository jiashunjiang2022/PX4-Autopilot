# 最终诊断 - TECS输出分析

## 版本

**Git**: `49fbcdf8b6`

## 目的

查看TECS实际输出了什么俯仰和油门指令。

## 新增调试输出

每2秒显示：

```
ERROR [fw_mode_manager] TECS: alt_sp=XXX, alt_ref=XXX, pitch_sp=XX°, throttle_sp=0.XX, hgt_rate_sp=X.XX
WARN  [fw_mode_manager] Config: climb=3.0, sink=2.0, pitch[-15.0°,45.0°]
```

### 关键指标

1. **pitch_sp**（俯仰角设定值）：
   - 如果 > 10° → TECS在要求爬升
   - 如果 ≈ 0° → TECS在平飞
   - 如果 < 0° → TECS在要求下降

2. **throttle_sp**（油门设定值）：
   - 如果 > 0.7 → TECS在加油门
   - 如果 ≈ 0.5 → TECS在维持
   - 如果 < 0.3 → TECS在减油门

3. **hgt_rate_sp**（高度率设定值）：
   - 如果 > 1.0 → TECS想要爬升
   - 如果 ≈ 0 → TECS想要保持高度
   - 如果 < -1.0 → TECS想要下降

## 可能的诊断结果

### 场景A：TECS指令正确，飞机不响应

```
TECS: pitch_sp=15.0°, throttle_sp=0.85, hgt_rate_sp=3.0
但飞机还在下降
```

→ **问题在仿真环境或飞机模型**：
  - 推力不足
  - 飞机太重
  - 气动模型有问题

### 场景B：TECS指令错误（不要求爬升）

```
TECS: pitch_sp=0.0°, throttle_sp=0.5, hgt_rate_sp=0.0
飞机平飞或下降
```

→ **TECS控制器问题**：
  - 参数配置错误
  - TECS算法bug
  - 高度设定值未正确处理

### 场景C：TECS指令负值（要求下降）

```
TECS: pitch_sp=-5.0°, throttle_sp=0.3, hgt_rate_sp=-2.0
飞机下降
```

→ **TECS逻辑错误**：
  - 可能误判需要下降
  - 或者speed_weight=2.0（只控制速度）

## 测试步骤

```bash
cd /home/jjs/PX4/PX4-Autopilot
make clean
make px4_sitl gazebo-classic
```

## 关键观察

起飞后看到AUTO_POSITION时，立即看：
```
WARN [fw_mode_manager] AUTO_POSITION: alt_sp=530.8, airspeed_sp=15.0, curr_alt=506
ERROR [fw_mode_manager] TECS: ... pitch_sp=??°, throttle_sp=??, hgt_rate_sp=??
```

**请特别关注`pitch_sp`和`throttle_sp`的值！**

这两个值将100%确定问题所在：
- 如果TECS指令climb → 问题在飞机/仿真
- 如果TECS不指令climb → 问题在TECS

请测试并告诉我TECS的输出值！🔍

