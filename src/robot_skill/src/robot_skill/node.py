from __future__ import annotations

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node as RclpyNode

from robot_skill.state_machine import SkillStateMachine


class RobotSkillNode(RclpyNode):
    """Skeleton node responsible for coordinating robot skills."""

    def __init__(self) -> None:
        super().__init__('robot_skill')
        self._declare_parameters()
        self._pose_pub = self.create_publisher(
            PoseStamped, '/robot_driver/robot_command', 10
        )
        self._state_machine = SkillStateMachine(self)
        self._heartbeat_timer = self.create_timer(5.0, self._log_heartbeat)
        self.get_logger().info('robot_skill node initialized')

    def _declare_parameters(self) -> None:
        self.declare_parameter('approach_offset_z', 0.20)
        self.declare_parameter('pre_grasp_offset_z', 0.02)
        self.declare_parameter('lift_height', 0.15)
        self.declare_parameter('position_tolerance', 0.01)
        self.declare_parameter('move_timeout', 10.0)
        self.declare_parameter('gripper_timeout', 5.0)
        self.declare_parameter('xyz_only_mode', False)
        self.declare_parameter('auto_open_gripper', True)

    def _log_heartbeat(self) -> None:
        params = {
            'approach_offset_z': self.get_parameter('approach_offset_z')
            .get_parameter_value()
            .double_value,
            'lift_height': self.get_parameter('lift_height')
            .get_parameter_value()
            .double_value,
            'xyz_only_mode': self.get_parameter('xyz_only_mode')
            .get_parameter_value()
            .bool_value,
        }
        self.get_logger().info(f'robot_skill heartbeat: {params}')

    def trigger_demo_sequence(self) -> None:
        """Placeholder API to show where real skill execution will hook in."""
        self._state_machine.reset()
        self.get_logger().info('Demo sequence triggered (no-op).')


def main() -> None:
    rclpy.init()
    node = RobotSkillNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('robot_skill shutdown requested by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


__all__ = ['RobotSkillNode', 'main']
