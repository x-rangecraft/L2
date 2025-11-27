"""
Perception Node Launch 文件

TODO: Phase 10 实现
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """生成 launch 描述"""
    return LaunchDescription([
        Node(
            package='perception',
            executable='perception_node',
            name='perception_node',
            output='screen',
            # TODO: 添加参数配置
        ),
    ])

