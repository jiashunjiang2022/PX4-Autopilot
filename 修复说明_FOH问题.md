# FOH导致持续下降问题 - 最终修复

## 🎯 根本原因确认

根据你的描述："**刚起飞时高度能上升，对准航点后一直下降**"

### 问题分析

1. **起飞阶段（TAKEOFF模式）**：
   - 使用起飞专用控制逻辑
   - 飞机正常爬升（比如从488米爬到500米）
   - ✅ 工作正常

2. **对准航点后（AUTO模式）**：
   - 切换到AUTO模式，FOH（First Order Hold）生效
   - FOH计算线性高度路径：起点488米 → 终点550米
   - 如果飞机已经在500米，但距离还很远（比如270米）
   - FOH计算：应该在495米（线性路径上）
   - **FOH指令下降：500 → 495米** ❌
   - 飞机开始下降，触发低高度保护
   - 但FOH持续指令下降，保护无法对抗

### FOH的设计逻辑

FOH假设飞机应该沿着**线性路径**爬升：
```
距离远 → 高度低
距离近 → 高度高
```

**问题**：起飞时飞机已经爬高，但FOH要求回到线性路径 → 指令下降！

## ✅ 最终修复方案

### 修复1：低空禁用FOH

```cpp
const float DISABLE_FOH_AGL = 80.0f;

if (agl < DISABLE_FOH_AGL) {
    // 低空时直接飞目标航点高度，不使用FOH插值
    position_sp_alt = pos_sp_curr.alt;
}
```

**效果**：
- AGL < 80米时，直接命令飞机爬升到目标航点高度
- 不再计算线性路径
- 避免下降指令

### 修复2：FOH安全限制

```cpp
const float foh_alt = a + grad * _min_current_sp_distance_xy;
const float min_safe_msl = ground_alt + 50.0f;

if (foh_alt < min_safe_msl) {
    position_sp_alt = min_safe_msl;  // 限制最低高度
}
```

**效果**：
- 即使FOH计算下降，也不得导致AGL < 50米
- 双重保险

### 修复3：简化低高度保护

```cpp
if (!_landed && !approaching_landing && (agl < 40.0f)) {
    if (position_sp_alt < safe_altitude) {
        position_sp_alt = safe_altitude;  // 强制安全高度
    }
}
```

**效果**：
- 移除了"position_sp_alt < _current_altitude"条件
- 只要低空就检查并修正

## 📊 预期效果

### 起飞到第一航点

```
时刻    模式     AGL    FOH状态        高度指令      实际效果
--------------------------------------------------------------
0s     TAKEOFF   5m    禁用           爬升          ✅ 正常爬升
10s    TAKEOFF  20m    禁用           爬升          ✅ 继续爬升
20s    AUTO     35m    禁用(低空)     550m(目标)    ✅ 持续爬升
40s    AUTO     60m    禁用(低空)     550m(目标)    ✅ 持续爬升
60s    AUTO     85m    启用+限制      线性路径      ✅ 平稳过渡
80s    到达    550m    启用           550m          ✅ 到达目标
```

### 关键改进

- ✅ **AGL < 80米时完全禁用FOH**：避免危险下降指令
- ✅ **FOH启用后有安全限制**：不得导致AGL < 50米
- ✅ **低高度保护始终工作**：AGL < 40米强制修正

## 🧪 测试步骤

```bash
cd /home/jjs/PX4/PX4-Autopilot
make clean
make px4_sitl gazebo-classic
```

### 观察重点

1. **起飞阶段**：
   - 高度应该持续上升
   - ✅ 正常

2. **对准航点后（原来会下降的时刻）**：
   - 应该看到：`FOH DISABLED at low altitude: AGL=XX < 80`
   - 高度应该**继续上升**，不再下降
   - ✅ **这是修复的关键点**

3. **爬升到80米以上**：
   - FOH重新启用
   - 但有安全限制
   - 高度平稳过渡到目标高度

## 🔧 Git版本

**当前版本**: `04b8a358d5`

```bash
# 查看提交
git log -1

# 如果需要回退到之前版本
git log --oneline -5  # 查看历史
git reset --hard <commit-id>  # 回退
```

## 📝 测试反馈

请测试后告诉我：

1. ✅ 起飞阶段是否正常？
2. ✅ **对准航点后是否还会下降？（这是关键！）**
3. ✅ 是否能顺利爬升到目标高度？
4. ✅ 是否看到"FOH DISABLED"消息？

如果这次修复成功，你应该能看到飞机**从起飞到第一航点全程保持爬升**，不再出现"对准后下降"的问题！

