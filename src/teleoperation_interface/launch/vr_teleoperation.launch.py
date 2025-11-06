#!/usr/bin/env python3
"""
VR Teleoperation Launch File
VR遥操作启动文件

该启动文件用于启动完整的VR遥操作接口，包括：
1. VR数据服务器 (vr_data_server.py) - 接收VR设备数据并发布到ROS2话题
2. VR遥操作节点 (vr_teleoperation_node.py) - 处理数据并发布ROS2话题

使用方法:
ros2 launch vr_teleoperation vr_teleoperation.launch.py

参数:
- config_file: 配置文件路径 (默认: src/vr_teleoperation/teleoperation_config.yaml)
- server_port: 数据服务器端口 (默认: 5000)
- server_host: 数据服务器主机地址 (默认: 127.0.0.1)

注意：不再使用JSON文件传输数据，改用ROS2话题 /vr_raw_data
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """生成启动描述"""
    
    # 获取包路径
    pkg_share = FindPackageShare(package='teleoperation_interface').find('teleoperation_interface')
    
    # 声明启动参数
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_share, 'teleoperation_config.yaml'),
        description='VR遥操作配置文件路径'
    )
    
    server_port_arg = DeclareLaunchArgument(
        'server_port',
        default_value='5000',
        description='VR数据服务器端口'
    )
    
    server_host_arg = DeclareLaunchArgument(
        'server_host',
        default_value='127.0.0.1',
        description='VR数据服务器主机地址'
    )
    
    # 获取配置值
    config_file = LaunchConfiguration('config_file')
    server_port = LaunchConfiguration('server_port')
    server_host = LaunchConfiguration('server_host')
    
    # 启动信息
    launch_info = LogInfo(
        msg=[
            "启动VR遥操作接口（ROS2原生通信模式）...\n",
            "配置文件: ", config_file, "\n",
            "服务器地址: ", server_host, ":", server_port, "\n",
            "数据流: VR设备 → 数据服务器 → ROS2话题(/vr_raw_data) → 遥操作节点\n",
            "请确保VR设备已连接并配置正确"
        ]
    )
    
    # VR数据服务器进程
    # 直接使用src目录下的vr_data_server.py
    # 使用系统的Python 3.10（而不是Anaconda的Python）
    src_vr_data_server = os.path.join(pkg_share, '..', '..', '..', '..', 'src', 'teleoperation_interface', 'vr_teleoperation', 'vr_data_server.py')
    vr_data_server = ExecuteProcess(
        cmd=[
            '/usr/bin/python3',  # 明确使用系统Python 3.10
            src_vr_data_server,
            '--port', server_port,
            '--host', server_host
        ],
        name='vr_data_server',
        output='screen',
        shell=False
    )
    
    # VR遥操作ROS2节点
    vr_teleoperation_node = Node(
        package='teleoperation_interface',
        executable='vr_teleoperation_node',
        name='vr_teleoperation_node',
        output='screen',
        parameters=[
            {
                'config_file': config_file
            }
        ],
        remappings=[
            # 可以在这里添加话题重映射
        ]
    )
    
    return LaunchDescription([
        # 声明参数
        config_file_arg,
        server_port_arg,
        server_host_arg,
        
        # 启动信息
        launch_info,
        
        # 启动进程
        vr_data_server,
        vr_teleoperation_node,
    ])
