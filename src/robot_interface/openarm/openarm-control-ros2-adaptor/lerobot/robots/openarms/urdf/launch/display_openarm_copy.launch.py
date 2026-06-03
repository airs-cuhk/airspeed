# Copyright 2025 Enactic, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 指定URDF文件路径
    urdf_path = "/home/intern/copy_openarm_huang/openarms/OpenArm/lerobot/src/lerobot/robots/openarms/urdf/openarm_bimanual_copy1.urdf"
    
    # 读取URDF文件内容
    with open(urdf_path, 'r') as f:
        robot_description = f.read()
    
    # 获取RViz配置文件路径
    rviz_config_path = os.path.join(
        "/home/intern/copy_openarm_huang/openarms/OpenArm/lerobot/src/lerobot/robots/openarms/urdf/rviz",
        "bimanual.rviz"
    )
    
    return LaunchDescription([
        # 启动robot_state_publisher节点，发布机器人描述
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        
        # 启动joint_state_publisher_gui节点，允许手动控制关节
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="screen"
        ),
        
        # 启动rviz2节点，加载指定的配置文件
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["--display-config", rviz_config_path],
            output="screen"
        ),
    ])