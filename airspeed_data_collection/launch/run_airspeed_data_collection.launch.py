import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value=os.path.join(get_package_share_directory('airspeed_data_collection'), 'config/config.json'),
            description='Path to the config file'
        ),
        # data_collection_node
        Node(
            name='data_collection',
            package='airspeed_data_collection',
            executable='data_collection',
            output='screen',
            parameters=[{'config': LaunchConfiguration('config')}]
        ),
    ])
