# 当前版本说明 - FOH安全限制版

## ✅ 已提交到Git

**提交ID**: `09aa43acc3`
**分支**: `pid`

你现在可以随时用 `git reset --hard <commit-id>` 回退到任意版本！

## 为什么第一航点比起飞点高，飞机还会下降？

### FOH算法的问题

**FOH公式**（第777-780行）：
```cpp
delta_alt = WP1.alt - WP0.alt           // 如 550 - 488 = 62m
grad = -delta_alt / (distance - acc_rad) // 注意负号！如 -62 / 340 = -0.18
a = WP0.alt - grad * distance           // 如 488 - (-0.18) * 400 = 560
position_sp_alt = a + grad * min_distance_xy  // 插值
```

**关键**：使用 `_min_current_sp_distance_xy`（历史最小距离）

**问题场景**：
```
起飞后立即切换到AUTO
→ 飞机离WP1很远（如400米）
→ _min_current_sp_distance_xy被更新为很小的值（如50米）
→ FOH计算：position_sp_alt = 560 + (-0.18) * 50 = 551m
→ 随着飞近，_min_current_sp_distance_xy减小
→ position_sp_alt降低
→ 当_min_current_sp_distance_xy很小时，计算出的高度可能很低
→ 导致下降！
```

**根本问题**：
- `_min_current_sp_distance_xy` 没有在航点切换时重置
- 或者起飞点的类型不对，导致FOH逻辑异常

## 当前版本的修复

### 修复1：FOH安全钳位（第782-789行）

```cpp
const float foh_altitude = a + grad * _min_current_sp_distance_xy;

// 检查FOH计算的高度是否安全
const float agl_at_foh = foh_altitude - (_current_altitude - (-_local_pos.z));
if (agl_at_foh < 40.0f) {
    position_sp_alt = _current_altitude + (40.0f - agl_at_foh);  // 强制提高
} else {
    position_sp_alt = foh_altitude;  // 使用FOH
}
```

**作用**：
- 即使FOH计算错误
- 也不会让AGL降到40米以下

### 修复2：低高度保护（第812-818行）

```cpp
if (!_landed && !approaching_landing && (agl < 30.0f)) {
    position_sp_alt = _current_altitude + (30.0f - agl) + 5.0f;
}
```

**双重保护**：
- FOH安全钳位（40米）
- 低高度保护（30米）

## 编译测试

```bash
cd /home/jjs/PX4/PX4-Autopilot
make clean
make px4_sitl gazebo-classic
```

## 预期日志

**正常飞行**：
```
INFO  [fw_mode_manager] === HEARTBEAT: Alt=520.0, AGL=32.0, landed=0 ===
INFO  [fw_mode_manager] FOH: Alt SP=550.0, Current=520.0, AGL=32.0, landing=0
```

**FOH计算危险高度时**：
```
INFO  [fw_mode_manager] FOH: Alt SP=500.0, Current=515.0, AGL=27.0, landing=0
（FOH安全钳位自动生效，无日志，但Alt SP会被修正）
```

**触发低高度保护时**：
```
WARN  [fw_mode_manager] !!! LOW ALT PROTECT: AGL=25.0 < 30.0, Alt SP 510.0 -> 550.0 !!!
```

**不应该再看到**：
```
❌ AGL持续下降到负值
❌ ERROR HARD-DECK (已移除)
❌ Attitude failure
```

## Git操作快速参考

### 查看版本历史
```bash
git log --oneline -10
```

### 回退到指定版本
```bash
git reset --hard 09aa43acc3  # 当前版本
git reset --hard 9163cd1e39  # 上一版本
git reset --hard d711cb4cd7  # 干净版本
```

### 查看当前状态
```bash
git status
git log -1
```

### 查看某个版本的修改
```bash
git show 09aa43acc3
git diff 9163cd1e39 09aa43acc3
```

## 总结

**核心修复**：
1. ✅ FOH不会导致AGL<40米（安全钳位）
2. ✅ 低高度<30米强制爬升（无条件）
3. ✅ 移除硬地线（避免振荡）
4. ✅ 着陆时禁用保护
5. ✅ 详细调试输出

**回退方便**：
- ✅ 已提交到git（09aa43acc3）
- ✅ 可随时回退

请测试这个版本，应该彻底解决触地和振荡问题了！


