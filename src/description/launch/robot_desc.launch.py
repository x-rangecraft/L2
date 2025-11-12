from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    urdf_arg = DeclareLaunchArgument(
        "urdf_path",
        default_value="package://description/urdf/yam.urdf",
        description="Absolute or package:// path to the URDF/xacro file",
    )
    xacro_arg = DeclareLaunchArgument(
        "xacro_args",
        default_value="",
        description="Comma-separated KEY:=VALUE pairs forwarded to xacro",
    )
    frame_prefix_arg = DeclareLaunchArgument(
        "frame_prefix",
        default_value="",
        description="Optional frame prefix applied to link_/joint_ names",
    )
    publish_rate_arg = DeclareLaunchArgument(
        "publish_rate",
        default_value="1.0",
        description="Publish rate for /robot_description (Hz)",
    )
    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="ROS namespace for the robot_desc_node",
    )
    node_name_arg = DeclareLaunchArgument(
        "node_name",
        default_value="robot_desc_node",
        description="Node name override",
    )

    robot_node = Node(
        package="description",
        executable="robot_desc_node",
        name=LaunchConfiguration("node_name"),
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        parameters=[
            {
                "urdf_path": LaunchConfiguration("urdf_path"),
                "frame_prefix": LaunchConfiguration("frame_prefix"),
                "publish_rate": ParameterValue(LaunchConfiguration("publish_rate"), value_type=float),
                "xacro_args": ParameterValue(LaunchConfiguration("xacro_args"), value_type=str),
            }
        ],
    )

    return LaunchDescription(
        [
            urdf_arg,
            xacro_arg,
            frame_prefix_arg,
            publish_rate_arg,
            namespace_arg,
            node_name_arg,
            robot_node,
        ]
    )
