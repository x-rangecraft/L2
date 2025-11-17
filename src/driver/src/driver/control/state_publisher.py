"""State publisher helpers for robot_driver."""
from __future__ import annotations

from dataclasses import dataclass

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

from driver.config.parameter_schema import DriverParameters
from driver.hardware.hardware_commander import HardwareCommander


@dataclass
class DiagnosticsCache:
    last_publish_time: float = 0.0


class StatePublisher:
    def __init__(self, node, commander: HardwareCommander, params: DriverParameters):
        self._node = node
        self._commander = commander
        self._params = params
        self._joint_pub = node.create_publisher(JointState, params.joint_state_topic, 10)
        self._diag_pub = node.create_publisher(DiagnosticArray, params.diagnostics_topic, 10)
        self._tf_broadcaster = TransformBroadcaster(node) if params.publish_tf else None
        self._diagnostics_cache = DiagnosticsCache()

        joint_period = 1.0 / max(1e-3, params.joint_state_rate)
        diag_period = 1.0 / max(1e-3, params.diagnostics_rate)
        self._joint_timer = node.create_timer(joint_period, self._publish_joint_state)
        self._diag_timer = node.create_timer(diag_period, self._publish_diagnostics)

    # ------------------------------------------------------------------ timers
    def _publish_joint_state(self):
        data = self._commander.read_joint_state()
        msg = JointState()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = self._params.tf_base_frame
        msg.name = list(data.names)
        msg.position = list(data.positions)
        msg.velocity = list(data.velocities)
        msg.effort = list(data.efforts)
        self._joint_pub.publish(msg)

        if self._tf_broadcaster:
            pose = self._commander.read_end_effector_pose()
            tf_msg = TransformStamped()
            tf_msg.header.stamp = msg.header.stamp
            tf_msg.header.frame_id = self._params.tf_base_frame
            tf_msg.child_frame_id = self._params.tf_tool_frame
            tf_msg.transform.translation.x = pose.position.x
            tf_msg.transform.translation.y = pose.position.y
            tf_msg.transform.translation.z = pose.position.z
            tf_msg.transform.rotation.x = pose.orientation.x
            tf_msg.transform.rotation.y = pose.orientation.y
            tf_msg.transform.rotation.z = pose.orientation.z
            tf_msg.transform.rotation.w = pose.orientation.w
            self._tf_broadcaster.sendTransform(tf_msg)

    def _publish_diagnostics(self):
        now = self._node.get_clock().now()
        array = DiagnosticArray()
        array.header.stamp = now.to_msg()

        # Main health status
        status = DiagnosticStatus()
        status.name = 'robot_driver/health'
        status.hardware_id = self._params.diag_hardware_id
        status.level = DiagnosticStatus.OK
        status.message = 'OK'

        # Enhanced diagnostic values
        status.values = [
            KeyValue(key='zero_gravity', value=str(self._commander.get_zero_gravity())),
            KeyValue(key='can_channel', value=self._params.can_channel),
            KeyValue(key='command_timeout_s', value=f'{self._params.command_timeout_s:.1f}'),
            KeyValue(key='joint_state_rate', value=f'{self._params.joint_state_rate:.1f}'),
            KeyValue(key='xyz_only_mode', value=str(self._params.xyz_only_mode)),
        ]

        # Add joint state info
        joint_data = self._commander.read_joint_state()
        if joint_data.names:
            status.values.append(KeyValue(key='joint_count', value=str(len(joint_data.names))))
            # Add position summary (first 3 joints as example)
            for i in range(min(3, len(joint_data.names))):
                status.values.append(
                    KeyValue(
                        key=f'joint_{joint_data.names[i]}_pos',
                        value=f'{joint_data.positions[i]:.3f}'
                    )
                )

        array.status.append(status)

        # Latch check (avoid spamming diagnostics)
        last = self._diagnostics_cache.last_publish_time
        if now.nanoseconds - last > self._params.diagnostic_latch_sec * 1e9:
            self._diagnostics_cache.last_publish_time = now.nanoseconds

        self._diag_pub.publish(array)
