#!/usr/bin/env python3
"""
Complete System Launch File
完整系统启动文件

启动所有相关节点：
1. Realsense相机节点
2. 机器人接口节点
3. 数据存储节点
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    """生成启动描述"""
    
    # 声明启动参数
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='',
        description='Path to the data storage configuration file'
    )
    
    realsense_config_arg = DeclareLaunchArgument(
        'realsense_config_file',
        default_value='',
        description='Path to the realsense camera configuration file'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # 获取配置参数
    config_file = LaunchConfiguration('config_file')
    realsense_config_file = LaunchConfiguration('realsense_config_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # 默认配置文件路径
    default_data_storage_config = PathJoinSubstitution([
        FindPackageShare('data_collection_service'),
        'data_collection_service_config.yaml'
    ])
    
    default_realsense_config = PathJoinSubstitution([
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
                'config_file': realsense_config_file,
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
    
    # 创建机器人接口节点
    robot_interface_node = Node(
        package='robot_interface',
        executable='robot_interface_node',
        name='robot_interface_node',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time
            }
        ]
    )
    
    # 创建数据存储节点
    data_storage_node = Node(
        package='data_collection_service',
        executable='data_storage_node',
        name='data_storage_node',
        output='screen',
        parameters=[
            {
                'config_file': config_file,
                'use_sim_time': use_sim_time
            }
        ]
    )
    
    # 创建启动描述
    return LaunchDescription([
        config_file_arg,
        realsense_config_arg,
        use_sim_time_arg,
        LogInfo(msg='Starting Complete VR Robot Control System'),
        LogInfo(msg='Launching Realsense Camera Node'),
        realsense_camera_node,
        LogInfo(msg='Launching Robot Interface Node'),
        robot_interface_node,
        LogInfo(msg='Launching Data Storage Node'),
        data_storage_node,
        LogInfo(msg='All nodes started successfully')
    ])




