"""
Perception Node Launch 文件
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """生成 launch 描述"""
    
    # 获取包路径
    pkg_share = get_package_share_directory('perception')
    
    # 声明参数
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='日志级别 (debug, info, warn, error)'
    )
    
    # 节点
    perception_node = Node(
        package='perception',
        executable='perception_node.py',
        name='perception_node',
        output='screen',
        parameters=[
            os.path.join(pkg_share, 'config', 'perception_config.yaml')
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )
    
    return LaunchDescription([
        log_level_arg,
        perception_node,
    ])
