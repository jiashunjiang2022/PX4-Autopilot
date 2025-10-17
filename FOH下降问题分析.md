# FOH导致下降的根本原因分析

## 你的问题非常关键！

**问题**：第一个航点明明比起飞点高，为什么飞机会下降？

## FOH算法详解

### FOH的计算公式（第777-780行）

```cpp
const float delta_alt = pos_sp_curr.alt - pos_sp_prev.alt;  // 高度差
const float grad = -delta_alt / (d_curr_prev - acc_rad);     // 梯度（注意负号！）
const float a = pos_sp_prev.alt - grad * d_curr_prev;        // 截距
position_sp_alt = a + grad * _min_current_sp_distance_xy;    // 插值
```

### 问题：prev航点是什么？

**关键**：起飞后的第一段飞行，`pos_sp_prev` 是什么？

#### 情况A：prev是起飞点（TAKEOFF类型）
```cpp
if (_position_setpoint_previous_valid &&
    ((pos_sp_prev.type == SETPOINT_TYPE_POSITION) ||
     (pos_sp_prev.type == SETPOINT_TYPE_LOITER)))
```

**如果起飞点类型是TAKEOFF**：
- 条件不满足（起飞点不是POSITION或LOITER）
- **不使用FOH**
- 直接使用 `position_sp_alt = pos_sp_curr.alt`
- **应该是正确的高度**

#### 情况B：prev是普通航点但高度很低
如果任务设置是：
```
WP0（起飞点，488m）
WP1（第一个航点，550m）
```

如果WP0是POSITION类型：
- FOH会插值
- `delta_alt = 550 - 488 = 62m`
- `grad = -62 / (distance - acc_rad)`（负号！）
- 在距离WP1较远时，`position_sp_alt` 会比WP1低

**这可能导致中途高度低！**

### 情况C：_min_current_sp_distance_xy的影响

```cpp
_min_current_sp_distance_xy = math::min(d_curr, _min_current_sp_distance_xy, d_curr_prev);
```

**问题**：
- `_min_current_sp_distance_xy` 是历史最小距离
- 如果这个值很小，FOH插值会计算出很低的高度
- **可能需要在切换航点时重置这个变量！**

## 根本问题：_min_current_sp_distance_xy未重置

### 查看初始化

我需要查看这个变量在哪里初始化/重置：

```cpp
_min_current_sp_distance_xy = FLT_MAX;  // 初始值
```

**问题**：
- 从上一个航点继承过来的小值
- 导致FOH计算错误的高度

## 解决方案

### 方案1：在航点切换时重置_min_current_sp_distance_xy

```cpp
// 检测航点切换
static Vector2f last_curr_wp{NAN, NAN};
Vector2f curr_wp_local = _global_local_proj_ref.project(pos_sp_curr.lat, pos_sp_curr.lon);

if (!curr_wp_local.isAllFinite() || 
    (curr_wp_local - last_curr_wp).norm() > 1.0f) {
    // 航点变化了，重置最小距离
    _min_current_sp_distance_xy = FLT_MAX;
}
last_curr_wp = curr_wp_local;
```

### 方案2：简化FOH，起飞后不使用插值

```cpp
// 起飞后的第一段，不使用FOH
const bool first_waypoint_after_takeoff = 
    (_position_setpoint_previous_valid && 
     pos_sp_prev.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF);

if (first_waypoint_after_takeoff) {
    position_sp_alt = pos_sp_curr.alt;  // 直接使用目标高度
} else {
    // 正常FOH逻辑
}
```

### 方案3：完全禁用FOH（最简单）

```cpp
// 直接使用当前航点高度，不使用FOH插值
position_sp_alt = pos_sp_curr.alt;
```

**优点**：
- 最简单
- 最可预测
- 避免所有FOH相关问题

**缺点**：
- 高度变化不平滑
- 可能在切换航点时有跳变

## 我的建议

**先尝试方案2**：起飞后的第一段不使用FOH

这样：
- 起飞→第一航点：直接飞到目标高度（不下降）
- 后续航点：仍使用FOH（平滑过渡）

## 提交到GitHub

我现在会：
1. 实施方案2（禁用起飞后第一段的FOH）
2. 提交到GitHub
3. 方便你后续回退

让我实施这个修复：


