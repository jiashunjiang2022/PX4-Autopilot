#!/bin/bash

echo "=== PX4仿真环境关闭脚本 ==="

# 终止所有相关进程
echo "正在关闭PX4仿真环境..."
pkill -9 px4 gzserver gzclient gazebo 2>/dev/null

# 等待进程完全关闭
sleep 1

# 检查是否还有残留进程
if pgrep -x "px4" > /dev/null; then
    echo "强制终止残留进程..."
    pkill -9 px4
fi

echo "仿真环境已关闭！"
