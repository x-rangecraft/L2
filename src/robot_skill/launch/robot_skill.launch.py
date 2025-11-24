from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_skill')
    config_file = os.path.join(pkg_share, 'config', 'robot_skill_config.yaml')

    return LaunchDescription([
        Node(
            package='robot_skill',
            executable='robot_skill_node',
            name='robot_skill',
            output='screen',
            parameters=[config_file],
        )
    ])
