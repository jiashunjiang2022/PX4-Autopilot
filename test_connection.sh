#!/bin/bash

echo "=== PX4连接测试脚本 ==="

# 1. 检查进程
echo "1. 检查PX4和Gazebo进程..."
ps aux | grep -E "(px4|gazebo)" | grep -v grep

# 2. 检查端口
echo -e "\n2. 检查MAVLink端口..."
netstat -tulpn | grep 14550

# 3. 检查网络连接
echo -e "\n3. 测试本地连接..."
ping -c 1 127.0.0.1

# 4. 检查防火墙
echo -e "\n4. 检查防火墙状态..."
sudo ufw status

echo -e "\n=== 测试完成 ==="
echo "如果看到PX4进程在运行且端口14550被监听，说明仿真器正常"
echo "请在QGroundControl中使用UDP连接，端口14550"
