#!/bin/bash

# 快速启动脚本 - 用于Ubuntu环境
echo "=== PX4快速启动 ==="

# 检查并关闭现有进程
if pgrep -x "px4" > /dev/null; then
    echo "关闭现有仿真..."
    pkill -9 px4 gzserver gzclient gazebo 2>/dev/null
    sleep 2
fi

# 启动仿真
echo "启动PX4仿真..."
make px4_sitl gazebo-classic_plane
