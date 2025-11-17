from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory('robot_driver'))
    default_params = package_share / 'config' / 'robot_driver_config.yaml'

    can_arg = DeclareLaunchArgument('can_channel', default_value='can0', description='CAN interface name, e.g. can0')
    xyz_arg = DeclareLaunchArgument('xyz_only_mode', default_value='false', description='XYZ-only Cartesian mode flag')
    zero_g_arg = DeclareLaunchArgument(
        'zero_gravity_default',
        default_value='true',
        description='Enable zero gravity after initial safe pose',
    )
    params_arg = DeclareLaunchArgument(
        'params',
        default_value=str(default_params),
        description='Full path to robot_driver parameter YAML file',
    )

    def launch_setup(_context):
        params_file = Path(LaunchConfiguration('params').perform(_context))
        if not params_file.exists():
            raise FileNotFoundError(f'Parameter file not found: {params_file}')

        node = Node(
            package='robot_driver',
            executable='robot_driver_node',
            name='robot_driver',
            output='screen',
            parameters=[
                params_file,
                {
                    'can_channel': LaunchConfiguration('can_channel'),
                    'xyz_only_mode': LaunchConfiguration('xyz_only_mode'),
                    'zero_gravity_default': LaunchConfiguration('zero_gravity_default'),
                },
            ],
        )
        return [node]

    return LaunchDescription([
        can_arg,
        xyz_arg,
        zero_g_arg,
        params_arg,
        OpaqueFunction(function=launch_setup),
    ])
