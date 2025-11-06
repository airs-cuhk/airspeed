#!/bin/bash

# Robot Interface Launch Script
# 机器人接口启动脚本
# 一键启动机器人接口节点

# 设置工作空间路径
WORKSPACE_PATH="/home/airspeedbox/code/vr_robot_control_ws"

# 检查工作空间是否存在
if [ ! -d "$WORKSPACE_PATH" ]; then
    echo "错误: 工作空间路径不存在: $WORKSPACE_PATH"
    exit 1
fi

# 切换到工作空间目录
cd "$WORKSPACE_PATH"

# 检查ROS2环境
if [ -z "$ROS_DISTRO" ]; then
    echo "警告: ROS2环境未设置，正在尝试加载ROS2 Humble..."
    source /opt/ros/humble/setup.bash
fi

# 构建项目（如果需要）
echo "正在构建robot_interface包..."
colcon build --packages-select robot_interface --cmake-args -DCMAKE_BUILD_TYPE=Release

# 检查构建是否成功
if [ $? -ne 0 ]; then
    echo "错误: 构建失败"
    exit 1
fi

# 加载环境
echo "正在加载ROS2环境..."
source install/setup.bash

# 启动机器人接口节点
echo "正在启动机器人接口节点..."
ros2 launch robot_interface robot_interface.launch.py

echo "机器人接口节点已启动"
