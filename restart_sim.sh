#!/bin/bash

echo "=== PX4仿真环境重启脚本 ==="

# 关闭现有仿真
echo "关闭现有仿真环境..."
pkill -9 px4 gzserver gzclient gazebo 2>/dev/null
sleep 2

# 启动新仿真
echo "启动新的仿真环境..."
make px4_sitl gazebo-classic_plane

echo "仿真环境重启完成！"
