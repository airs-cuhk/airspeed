#!/usr/bin/env python3
"""
Robot Interface Launch File
机器人接口启动文件

启动机器人接口节点，用于接收遥操作数据并控制机器人运动
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """生成启动描述"""
    
    # 获取包路径
    package_dir = get_package_share_directory('robot_interface')
    config_file = os.path.join(package_dir, 'robot_config.yaml')
    
    # 声明启动参数
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='Path to robot configuration file'
    )
    
    # 机器人接口节点
    robot_interface_node = Node(
        package='robot_interface',  # 需要根据实际包名调整
        executable='robot_interface_node',
        name='robot_interface_node',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('config_file')
        }],
        remappings=[
            # 可以在这里添加话题重映射
        ]
    )
    
    return LaunchDescription([
        config_file_arg,
        robot_interface_node,
    ])
