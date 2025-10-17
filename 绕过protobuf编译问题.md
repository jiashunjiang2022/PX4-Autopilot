# 绕过Protobuf编译问题并测试

## 问题

系统protobuf版本与Gazebo要求的不匹配，导致编译失败。

## 解决方案

有三种方式测试修改：

### 方案1：不使用Gazebo仿真（推荐用于调试）

```bash
cd /home/jjs/PX4/PX4-Autopilot

# 只编译PX4可执行文件（不编译Gazebo插件）
make px4_sitl_default

# 手动运行（不启动Gazebo）
./build/px4_sitl_default/bin/px4 -d
```

然后可以：
- 通过QGC连接
- 手动发送任务
- 查看日志输出（包括我们新增的WAYPOINT INFO）

### 方案2：使用jMAVSim（轻量级仿真）

```bash
cd /home/jjs/PX4/PX4-Autopilot
make px4_sitl jmavsim
```

jMAVSim不使用protobuf，应该能正常编译。

### 方案3：修复Protobuf问题

```bash
# 检查protobuf版本
protoc --version

# 如果版本太新（> 3.21），可能需要降级
# 或者重新构建Gazebo插件
cd /home/jjs/PX4/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic
rm -rf build
```

## 当前代码已提交

**Git版本**: `c2412eabd1`

### 新增调试输出

每2秒输出一次（低高度时更频繁）：

```
=== WAYPOINT INFO ===
  Current WP: lat=XX.XXXXXX, lon=XX.XXXXXX, alt=XXX.X (type=X)
  Previous WP: lat=XX.XXXXXX, lon=XX.XXXXXX, alt=XXX.X (type=X)
  Current Aircraft: lat=XX.XXXXXX, lon=XX.XXXXXX, alt=XXX.X
  FOH calculated: position_sp_alt=XXX.X
  AGL=XX.X, Airspeed=XX.X, Vz=X.XX
```

还会看到：
```
WARN [fw_mode_manager] FOH DISABLED: AGL=XX.X < 80.0, using target WP alt XXX.X
```

## 关键检查点

1. **航点高度是否正确？**
   - `Current WP: alt=XXX.X` 应该是你设置的目标高度
   - `Previous WP: alt=XXX.X` 应该是起飞点高度

2. **FOH是否被禁用？**
   - 如果AGL<80米，应该看到"FOH DISABLED"
   - 此时`position_sp_alt`应该等于`Current WP alt`

3. **FOH计算是否正确？**
   - 如果AGL>80米且FOH启用
   - `FOH calculated`显示的高度是否合理

## 如果无法编译Gazebo

你可以：
1. 使用方案1或2进行测试
2. 或者直接告诉我你在QGC中设置的航点信息：
   - 起飞点高度（AMSL）
   - 第一个航点高度（AMSL）
   - 两点之间的距离

我可以手算FOH会给出什么高度设定值。

