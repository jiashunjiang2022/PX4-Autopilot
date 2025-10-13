#!/bin/bash

# PX4 Ubuntu开发环境便捷脚本
# 用于从GitHub拉取代码并启动仿真

echo "=== PX4 Ubuntu开发环境 ==="

# 检查是否在正确的目录
if [ ! -f "Makefile" ]; then
    echo "错误：请在PX4-Autopilot目录中运行此脚本"
    exit 1
fi

# 检查git状态
echo "检查Git状态..."
git status --porcelain

# 拉取最新代码
echo "拉取最新代码..."
git pull origin pid

# 检查是否有编译文件
if [ ! -d "build/px4_sitl_default" ] || [ ! -f "build/px4_sitl_default/bin/px4" ]; then
    echo "检测到需要重新编译..."
    make sim-clean
else
    echo "使用现有编译文件..."
    make sim-start
fi

echo "开发环境准备完成！"
echo "使用以下命令管理仿真："
echo "  make sim-start  - 启动仿真"
echo "  make sim-stop   - 关闭仿真"
echo "  make sim-restart - 重启仿真"
