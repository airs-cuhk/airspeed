#!/bin/bash

# 数据收集服务快速启动脚本
# Quick Start Script for Data Collection Service


echo "🚀 启动数据收集服务..."
echo "Starting Data Collection Service..."

# 切换到工作空间
cd /home/airspeedbox/code/vr_robot_control_ws

# 构建包
echo "📦 构建包..."
colcon build --packages-select data_collection_service robot_interface

# 加载环境
echo "🔧 加载环境..."
source install/setup.bash

# 启动数据存储节点
echo "🎯 启动数据存储节点..."
echo "按 Ctrl+C 停止服务"
echo "Press Ctrl+C to stop the service"

ros2 launch data_collection_service data_storage.launch.py
