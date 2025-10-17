# 🎯 TECS Bug 100%确认

## 证据

```
行957: fw_lat_lon传给TECS: curr_alt=510.2(AMSL), alt_sp=530.8(AMSL)  ✓
行959: TECS内部状态:       alt_ref=122.2  ❌

行962: fw_lat_lon传给TECS: curr_alt=501.9(AMSL), alt_sp=530.8(AMSL)  ✓
行964: TECS内部状态:       alt_ref=128.3  ❌

行968: fw_lat_lon传给TECS: curr_alt=493.2(AMSL), alt_sp=530.8(AMSL)  ✓
行970: TECS内部状态:       alt_ref=134.3  ❌

Home高度: 488.2m
```

## 问题

1. **输入100%正确**：
   - 我们传的是AMSL（510m, 530m）
   - 所有值都正确

2. **TECS内部100%错误**：
   - `alt_ref = 122-134m`
   - 应该是 ~510m（当前高度）
   - **差了380-390米！**

3. **`alt_ref`的规律**：
   ```
   curr_alt=510 → alt_ref=122  差值=388
   curr_alt=502 → alt_ref=128  差值=374  
   curr_alt=493 → alt_ref=134  差值=359
   
   平均差值 ≈ 380m ≈ home(488) - 110
   ```

4. **TECS的错误判断**：
   - TECS认为：当前500m，参考122m → **太高了！指令下降**
   - 实际：当前510m，目标530m → **应该爬升**
   - 结果：`pitch_sp = -15.8°`（低头）

## 根因

**TECS的`altitude_reference`初始化或更新时用了错误的坐标系！**

可能的问题：
1. 初始化时用了local.z（NED）而不是AMSL
2. 初始化时用了AGL而不是AMSL  
3. `_alt_control_traj_generator.reset()`用了错误的值

## 122m这个值从哪来？

观察：
- `122m ≈ 510m - 488m + 100`
- `122m ≈ AGL(22m) + 100`

**可能是**：
- TECS被初始化为`local.z`（NED坐标）
- 或者某个地方用了`-local.z`但没有加`ref_alt`

## 修复方向

检查`TECS::initialize()`和`TECS::update()`中`altitude`参数的使用：

```cpp
// src/lib/tecs/TECS.cpp:687
void TECS::initialize(const float altitude, ...)
{
    TECSAltitudeReferenceModel::AltitudeReferenceState current_state{
        .alt = altitude,  // ← 这里应该是AMSL！
        .alt_rate = altitude_rate
    };
    _altitude_reference_model.initialize(current_state);
}

// src/lib/tecs/TECS.cpp:733
if (dt > DT_MAX || _update_timestamp == 0UL) {
    initialize(altitude, hgt_rate, equivalent_airspeed, eas_to_tas);
    // ← 这里的altitude是从哪来的？
}
```

**我们已经确认传给TECS.update()的altitude是AMSL（510m）**，所以：
- 要么`initialize()`没被正确调用
- 要么`_alt_control_traj_generator`在其他地方被重置了
- 要么有个地方直接修改了`_alt_control_traj_generator`的状态

## 下一步

需要在TECS内部添加调试，看看：
1. `initialize()`何时被调用，传入的`altitude`是多少
2. `_alt_control_traj_generator.reset()`何时被调用
3. `altitude_reference`是如何从正确值变成错误值的

