# 🔴 TECS Altitude Reference Bug - 根本原因

## 版本
**Git**: `a53756c1ce`

## 问题现象

飞机即使目标高度高于当前高度，仍然持续下降。

## 🎯 真正的根因（从日志确认）

```
TECS: alt_sp=530.8, alt_ref=83.3, pitch_sp=-16.0°, throttle_sp=0.07
TECS: alt_sp=530.8, alt_ref=92.4, pitch_sp=-16.0°, throttle_sp=0.07
TECS: alt_sp=530.8, alt_ref=98.5, pitch_sp=-15.3°, throttle_sp=0.08
...
AUTO_POSITION: alt_sp=530.8, curr_alt=506.7
```

### 问题分析

1. **我们发送给TECS**：
   - `alt_sp = 530.8米`（目标高度，正确！）
   - `curr_alt = 506米`（当前高度）

2. **TECS内部状态**：
   - `alt_ref = 83-116米`（❌ 完全错误！）
   - 应该是：`alt_ref ≈ 506米`（当前）→ 爬升到530.8米（目标）

3. **TECS的判断**：
   - TECS认为参考高度是83-116米
   - 飞机实际在506米
   - TECS错误地认为飞机高了400米！
   - **所以指令下降**：`pitch_sp=-16°`, `throttle_sp=0.07`

## 🔍 `altitude_reference`是什么？

从TECS源码（`TECS.cpp`）：

```cpp
// altitude_reference是轨迹生成器的当前位置
_alt_control_traj_generator.getCurrentPosition()

// 它应该：
// 1. 初始化时 = 当前高度（506米）
// 2. 逐渐爬升到 = 目标高度（530米）
```

## 💡 可能的原因

### 原因1：TECS初始化时用错了高度值

```cpp
// 可能在TAKEOFF→AUTO切换时
_tecs.initialize(altitude, ...);  // altitude = 83米？？？
```

**怀疑**：传给`initialize()`的`altitude`参数：
- 可能是AGL（相对地面）而不是AMSL（海拔）
- 可能是某个错误的高度源
- 可能用了起飞点高度而不是当前高度

### 原因2：坐标系转换错误

- PX4使用NED坐标系（Z轴向下）
- 可能某处用了`-local_pos.z`但没有加上home高度

### 原因3：TECS频繁重置

```cpp
// TECS.cpp:733
if (dt > DT_MAX || _update_timestamp == 0UL) {
    initialize(altitude, ...);  // 每次重置都用错误的高度？
}
```

## 📊 次要问题：俯仰角限制错误

```
Config: pitch[-859.4°, 1718.9°]
```

这是**弧度被当成角度**显示了：
- `-859° = -15弧度 ≈ -860°` ✓
- `1718° = 30弧度 ≈ 1719°` ✓

**但这只是显示问题**，不影响控制（TECS内部用弧度）。

真正的pitch限制应该在`FwLateralLongitudinalControl`中减去`FW_PSP_OFF`后传给TECS。

## ✅ 下一步诊断

### 1. 找到TECS初始化的位置

需要检查：
```cpp
src/modules/fw_lateral_longitudinal_control/FwLateralLongitudinalControl.cpp
```

特别是：
- `_tecs.initialize()` 在哪里被调用？
- 传入的`altitude`参数来自哪里？
- 是否在TAKEOFF→AUTO切换时被调用？

### 2. 验证高度来源

```cpp
// 应该使用AMSL (Mean Sea Level)
_long_control_state.altitude_msl  // ✓ 正确

// 不应该使用
_local_pos.z  // ✗ 这是NED坐标（向下为正）
agl  // ✗ 这是相对地面高度
```

### 3. 检查模式切换时的高度传递

从`fw_mode_manager`到`fw_lateral_longitudinal_control`：
- `fw_longitudinal_control_sp`中的`altitude`是什么？
- 是否正确传递了AMSL？

## 🎯 预期修复

找到并修复TECS初始化或更新时使用错误高度的地方，确保：
```cpp
_tecs.initialize(
    _long_control_state.altitude_msl,  // 使用AMSL！
    height_rate,
    airspeed,
    eas2tas
);
```

## 测试方法

修复后应该看到：
```
TECS: alt_sp=530.8, alt_ref=506.0, pitch_sp=10.0°, throttle_sp=0.75
                            ^^^^^ 应该接近当前高度！
```

然后`alt_ref`会逐渐从506爬升到530.8，同时：
- `pitch_sp > 0`（抬头）
- `throttle_sp > 0.6`（加油门）
- 飞机实际爬升✅

---

**这就是真正的根本原因！** 所有之前的修复都是对的（空速、配置、FOH），但TECS的`altitude_reference`初始化错误导致它完全误判形势。🎯

