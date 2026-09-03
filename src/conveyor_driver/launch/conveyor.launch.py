from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('conveyor_driver'),
        'config',
        'conveyor.yaml',
    )
    return LaunchDescription([
        Node(
            package='conveyor_driver',
            executable='conveyor_node',
            name='conveyor_driver',
            output='screen',
            parameters=[config_path],
        )
    ])
