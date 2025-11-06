#!/usr/bin/env python3
"""
Data Storage Launch File
数据存储启动文件

启动数据存储节点，支持参数配置
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """生成启动描述"""
    
    # 获取包路径
    data_storage_package_share = FindPackageShare(package='data_collection_service').find('data_collection_service')
    
    # 声明启动参数
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(data_storage_package_share, 'data_collection_service_config.yaml'),
        description='Path to the data storage configuration file'
    )
    
    storage_format_arg = DeclareLaunchArgument(
        'storage_format',
        default_value='hdf5',
        description='Data storage format (hdf5, csv, json, pickle)'
    )
    
    storage_directory_arg = DeclareLaunchArgument(
        'storage_directory',
        default_value='/home/airspeedbox/code/vr_robot_control_ws/robot_data',
        description='Directory to store data files'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level (debug, info, warn, error)'
    )
    
    # 获取启动配置
    config_file = LaunchConfiguration('config_file')
    storage_format = LaunchConfiguration('storage_format')
    storage_directory = LaunchConfiguration('storage_directory')
    log_level = LaunchConfiguration('log_level')
    
    # 数据存储节点
    data_storage_node = Node(
        package='data_collection_service',
        executable='data_storage_node',
        name='data_storage_node',
        output='screen',
        parameters=[{
            'config_file': config_file,
            'storage_format': storage_format,
            'storage_directory': storage_directory,
            'log_level': log_level
        }],
        remappings=[
            # 确保订阅来自robot_interface的话题
            ('/robot_joint_angles', '/robot_joint_angles'),
            ('/robot_cartesian_pose', '/robot_cartesian_pose')
        ]
    )
    
    # 日志信息
    log_info = LogInfo(
        msg=[
            'Starting Data Storage Node with configuration: ',
            config_file,
            ', Storage format: ',
            storage_format,
            ', Storage directory: ',
            storage_directory
        ]
    )
    
    return LaunchDescription([
        config_file_arg,
        storage_format_arg,
        storage_directory_arg,
        log_level_arg,
        log_info,
        data_storage_node
    ])
