# 关键诊断 - TECS是否收到高度设定值？

## 🔴 问题确认

从最新日志**100%确认**：

```
行513-516:
  Current WP: alt=530.8        ← 目标航点530.8米  
  Previous WP: alt=506.5       ← 起飞点506.5米
  Current Aircraft: alt=506.6  ← 当前506.6米
  FOH calculated: 530.8        ← FOH正确计算为530.8米 ✅

行518:
  LOW ALT PROTECT: Alt SP 530.8->538.2  ← 保护正确提升到538.2米 ✅

但是：
  行836: Current Aircraft: alt=491.6  ← 下降到491.6米 ❌
  行947: Current Aircraft: alt=479.8  ← 继续下降到479.8米 ❌
```

**结论**：
1. ✅ 航点高度正确：530.8米
2. ✅ FOH计算正确：530.8米  
3. ✅ 保护触发正确：538.2米
4. ❌ **但TECS完全不响应！飞机持续下降！**

## 🔍 可能的原因

### 原因A：`position_sp_alt`被覆盖

虽然我们设置了`position_sp_alt = 538.2`，但在发布之前可能被其他代码覆盖。

### 原因B：TECS收到但不响应

TECS收到了538.2米的设定值，但由于某种原因无法执行：
- TECS参数问题（增益太小）
- 俯仰/油门限制太严
- TECS内部逻辑问题

### 原因C：控制配置覆盖

发布后`_ctrl_configuration_handler`可能修改了油门/俯仰限制，导致TECS无法爬升。

## ✅ 新增的诊断

现在会在发布后立即显示：

```
ERROR [fw_mode_manager] >>> PUBLISHED TO TECS: altitude=XXX.X, airspeed=XX.X, pitch=AUTO, throttle=AUTO <<<
```

### 测试A：确认发布值

如果看到：
```
LOW ALT PROTECT: Alt SP 530.8->538.2
>>> PUBLISHED TO TECS: altitude=538.2 <<<
```
→ **我们的代码没问题**，问题在TECS层面

如果看到：
```
LOW ALT PROTECT: Alt SP 530.8->538.2
>>> PUBLISHED TO TECS: altitude=530.8 <<<  （不是538.2！）
```
→ `position_sp_alt`被覆盖了，需要找出在哪里

### 测试B：检查控制模式

如果看到：
```
>>> PUBLISHED TO TECS: ... pitch=DIRECT, throttle=DIRECT <<<
```
→ 直接控制模式激活，TECS被bypass了

## 💡 如果TECS收到正确值但不响应

### 可能需要检查的参数

```bash
# 在QGC Parameters中检查：
FW_T_CLMB_MAX    # 最大爬升率（默认5 m/s，太小会导致爬升慢）
FW_T_SINK_MAX    # 最大下沉率
FW_P_LIM_MAX     # 最大俯仰角（如果太小无法爬升）
FW_P_LIM_MIN     # 最小俯仰角
FW_THR_MAX       # 最大油门（如果太小推力不足）
FW_T_SPDWEIGHT   # 速度vs高度权重（2.0=只控制速度，0=只控制高度）
```

### 关键怀疑：`FW_T_SPDWEIGHT`

如果`FW_T_SPDWEIGHT = 2.0`：
- TECS只关注速度，完全忽略高度
- 即使高度设定值是538米，TECS也不会爬升
- 这可能是**滑翔模式**被意外启用了！

## 📋 立即测试

```bash
cd /home/jjs/PX4/PX4-Autopilot  
make px4_sitl_default
./build/px4_sitl_default/bin/px4 -d
```

或者如果protobuf问题解决了：
```bash
make px4_sitl gazebo-classic
```

### 观察重点

1. **保护触发时**：
   ```
   LOW ALT PROTECT: Alt SP 530.8->538.2
   ```

2. **立即看下一行**：
   ```
   >>> PUBLISHED TO TECS: altitude=??? <<<
   ```

3. **对比这两个值**：
   - 如果相同（538.2）→ TECS问题
   - 如果不同 → 我们的代码问题

## 🎯 预期结果

### 场景1：我们的代码有问题

```
LOW ALT PROTECT: Alt SP 530.8->538.2
>>> PUBLISHED TO TECS: altitude=530.8 <<<  ❌ 不一致！
```
→ 需要找出`position_sp_alt`在哪里被覆盖

### 场景2：TECS有问题

```
LOW ALT PROTECT: Alt SP 530.8->538.2
>>> PUBLISHED TO TECS: altitude=538.2 <<<  ✅ 一致！
但飞机还在下降...
```
→ 需要检查TECS参数或滑翔模式

### 场景3：直接控制被激活

```
>>> PUBLISHED TO TECS: ... pitch=DIRECT, throttle=DIRECT <<<
```
→ TECS被bypass，需要找出谁激活了直接控制

## 🚨 最可能的原因

根据之前的代码检查，我怀疑：

**滑翔模式被意外启用** (`pos_sp_curr.gliding_enabled = true`)

这会导致：
```cpp
throttle_max = 0.0;  // 油门被限制为0
_ctrl_configuration_handler.setSpeedWeight(2.f);  // 只控制速度
```

结果：
- TECS收到538.2米的高度设定值
- 但油门被限制为0
- 或者TECS只关注速度，忽略高度
- **飞机只能滑翔下降！**

请测试并反馈`>>> PUBLISHED TO TECS`显示的值！

