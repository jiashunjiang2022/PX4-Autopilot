# TECS不响应保护的根本问题

## 🔴 严重问题发现

从日志清楚看到：
```
行296-1008持续输出：
- FOH计算: Alt SP=549.1米
- 保护修改: Alt SP 549.1 -> 523.2米  
- 实际Current: 496 → 495 → 494 → ... → 480米（持续下降！）
```

**保护虽然触发且修改了高度设定值，但飞机完全不响应，仍在下降！**

## 可能的原因

### 原因1：发布延迟或丢失

虽然代码顺序是：
```cpp
行821-827: 修改position_sp_alt = safe_altitude
行832-841: 发布fw_longitudinal_control_sp (包含修改后的altitude)
```

但可能：
- uORB发布有延迟
- TECS订阅更新不及时
- 或者发布的值被后续代码覆盖

### 原因2：TECS被其他控制模式覆盖

`FwLateralLongitudinalControl.cpp`第204-224行：
```cpp
if (_fw_longitudinal_ctrl_sub.updated()) {
    _fw_longitudinal_ctrl_sub.copy(&_long_control_sp);
}

tecs_update_pitch_throttle(..., altitude_sp, ...);
```

**问题**：如果`control_auto_position`不是每个循环都被调用，TECS可能使用旧的设定值！

### 原因3：滑翔模式被意外启用

代码第843-851行：
```cpp
if (pos_sp_curr.gliding_enabled) {
    if (_current_altitude > 100.0f) {
        throttle_min = 0.0;
        throttle_max = 0.0;  // ← 零油门！
        _ctrl_configuration_handler.setSpeedWeight(2.f);
    }
}
```

**如果滑翔模式启用**：
- 油门被设为0
- 飞机会自然下滑
- 无法爬升

### 原因4：TECS参数问题

可能TECS的爬升率限制太小，无法响应大的高度误差。

## 立即诊断

需要添加更多调试信息：

### 添加1：实际发布的值

```cpp
_longitudinal_ctrl_sp_pub.publish(fw_longitudinal_control_sp);

// 立即确认发布的值
PX4_INFO("PUBLISHED: Alt SP=%.1f, pitch_direct=%s, throttle_direct=%s",
         (double)fw_longitudinal_control_sp.altitude,
         PX4_ISFINITE(pitch_direct_cmd) ? "SET" : "NAN",
         PX4_ISFINITE(throttle_direct_cmd) ? "SET" : "NAN");
```

### 添加2：滑翔模式检查

```cpp
if (pos_sp_curr.gliding_enabled) {
    PX4_ERR("!!! GLIDING MODE ENABLED - THROTTLE DISABLED !!!");
}
```

### 添加3：油门限制检查

```cpp
_ctrl_configuration_handler.setThrottleMax(throttle_max);

PX4_INFO("Throttle limits: min=%s, max=%s",
         PX4_ISFINITE(throttle_min) ? std::to_string(throttle_min).c_str() : "NAN",
         PX4_ISFINITE(throttle_max) ? std::to_string(throttle_max).c_str() : "NAN");
```

## 我的怀疑

**最可能的原因：滑翔模式被意外启用！**

如果你的航点设置了`gliding_enabled = true`：
- 油门被强制为0
- 飞机只能滑翔下降
- 无论高度设定值多高都无法爬升

## 立即修复

让我添加这些调试并强制禁用滑翔模式：


