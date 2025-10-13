#!/bin/bash

echo "=== PX4仿真环境启动脚本 ==="

# 检查是否有进程在运行
if pgrep -x "px4" > /dev/null; then
    echo "检测到PX4进程正在运行，正在关闭..."
    pkill -9 px4 gzserver gzclient gazebo 2>/dev/null
    sleep 2
fi

# 检查是否需要重新编译
if [ ! -d "build/px4_sitl_default" ] || [ ! -f "build/px4_sitl_default/bin/px4" ]; then
    echo "检测到需要重新编译..."
    make clean
    make px4_sitl gazebo-classic_plane
else
    echo "使用现有编译文件启动仿真..."
    make px4_sitl gazebo-classic_plane
fi

echo "仿真环境启动完成！"
