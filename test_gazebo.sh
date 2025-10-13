#!/bin/bash

echo "=== Gazebo测试脚本 ==="

# 1. 清理进程
echo "1. 清理所有相关进程..."
pkill -9 px4 gzserver gzclient gazebo 2>/dev/null
sleep 2

# 2. 检查模型文件
echo -e "\n2. 检查模型文件..."
if [ -f "Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/plane/plane.sdf" ]; then
    echo "✓ plane.sdf 文件存在"
    xmllint --noout Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/plane/plane.sdf 2>/dev/null
    if [ $? -eq 0 ]; then
        echo "✓ plane.sdf 文件格式正确"
    else
        echo "✗ plane.sdf 文件格式错误"
    fi
else
    echo "✗ plane.sdf 文件不存在"
fi

# 3. 检查系统资源
echo -e "\n3. 检查系统资源..."
echo "内存使用:"
free -h | grep Mem
echo "磁盘空间:"
df -h / | tail -1

# 4. 尝试编译
echo -e "\n4. 尝试编译..."
echo "清理编译缓存..."
make clean >/dev/null 2>&1

echo "开始编译（使用iris模型）..."
timeout 300 make px4_sitl gazebo-classic_iris

if [ $? -eq 0 ]; then
    echo "✓ 编译成功"
else
    echo "✗ 编译超时或失败"
fi

echo -e "\n=== 测试完成 ==="
