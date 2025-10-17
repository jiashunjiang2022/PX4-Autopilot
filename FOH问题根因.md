# FOH导致持续下降的根本原因

## 🔴 问题发现

从用户描述：
- **刚起飞时高度能上升** → 起飞模式工作正常
- **对准航点后持续下降** → 切换到AUTO模式后FOH生效

## FOH逻辑分析

### FOH公式（第777-780行）

```cpp
const float grad = -delta_alt / (d_curr_prev - acc_rad);
const float a = pos_sp_prev.alt - grad * d_curr_prev;
position_sp_alt = a + grad * _min_current_sp_distance_xy;
```

### 问题场景

假设：
- **起飞点高度**: 488米
- **目标航点高度**: 550米（更高）
- **delta_alt** = 550 - 488 = 62米
- **d_curr_prev** = 300米（两航点距离）
- **acc_rad** = 50米（接受半径）

计算：
```
grad = -62 / (300 - 50) = -62 / 250 = -0.248
a = 488 - (-0.248 * 300) = 488 + 74.4 = 562.4
```

当飞机接近航点时，`_min_current_sp_distance_xy`减小：
```
距离300米: Alt SP = 562.4 + (-0.248 * 300) = 488米  ← 起飞点高度
距离200米: Alt SP = 562.4 + (-0.248 * 200) = 512米  ← 开始爬升
距离100米: Alt SP = 562.4 + (-0.248 * 100) = 537米  
距离50米:  Alt SP = 562.4 + (-0.248 * 50)  = 550米  ← 到达目标
```

**关键问题**：
- 当飞机刚"对准"航点（距离还很远，比如270米）时
- FOH计算：`Alt SP = 562.4 + (-0.248 * 270) = 495米`
- **但起飞点是488米，飞机可能刚爬到495米**
- 如果飞机已经在500米，FOH会指令下降到495米！

### 真正的根因

**FOH假设飞机应该沿着线性路径爬升**：
- 距离越远 → 高度越低
- 距离越近 → 高度越高

但如果**飞机已经爬高了**（起飞阶段爬升），而距离还很远，FOH会指令**下降**回到线性路径上！

这就是为什么：
1. 起飞时能爬升（使用起飞模式，不受FOH控制）
2. 对准航点后下降（切换到AUTO，FOH生效，指令下降到线性路径）

## 解决方案

### 方案1：低空禁用FOH（最安全）

```cpp
// 低空时不使用FOH插值，直接飞目标高度
if (agl < 100.0f) {
    position_sp_alt = pos_sp_curr.alt;  // 直接飞目标高度
} else {
    // 正常FOH逻辑
    position_sp_alt = a + grad * _min_current_sp_distance_xy;
}
```

### 方案2：FOH不得低于当前高度

```cpp
const float foh_alt = a + grad * _min_current_sp_distance_xy;
position_sp_alt = math::max(foh_alt, _current_altitude);  // 不得低于当前
```

### 方案3：FOH不得导致AGL<安全高度

```cpp
const float foh_alt = a + grad * _min_current_sp_distance_xy;
const float min_safe_msl = _current_altitude - agl + 50.0f;  // AGL保持>50米
position_sp_alt = math::max(foh_alt, min_safe_msl);
```

## 我的建议

**立即采用方案1**：低空完全禁用FOH，直接飞目标高度。

这是最安全、最直接的方案。

