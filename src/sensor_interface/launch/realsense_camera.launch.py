#!/usr/bin/env python3
"""
Realsense Camera Launch File
Realsense相机启动文件
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """生成启动描述"""
    
    # 声明启动参数
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='',
        description='Path to the configuration file'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # 获取配置参数
    config_file = LaunchConfiguration('config_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # 默认配置文件路径
    default_config_path = PathJoinSubstitution([
        FindPackageShare('sensor_interface'),
        'config',
        'realsense_config.yaml'
    ])
    
    # 创建Realsense相机节点
    realsense_camera_node = Node(
        package='sensor_interface',
        executable='realsense_camera_node.py',
        name='realsense_camera_node',
        output='screen',
        parameters=[
            {
                'config_file': config_file,
                'use_sim_time': use_sim_time
            }
        ],
        remappings=[
            ('/camera/color/image_raw', '/camera/color/image_raw'),
            ('/camera/depth/image_raw', '/camera/depth/image_raw'),
            ('/camera/color/camera_info', '/camera/color/camera_info'),
            ('/camera/depth/camera_info', '/camera/depth/camera_info'),
            ('/camera/depth/points', '/camera/depth/points')
        ]
    )
    
    # 创建启动描述
    return LaunchDescription([
        config_file_arg,
        use_sim_time_arg,
        LogInfo(msg='Starting Realsense Camera Node'),
        realsense_camera_node
    ])




