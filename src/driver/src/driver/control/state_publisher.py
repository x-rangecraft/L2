"""State publisher helpers for robot_driver."""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

from driver.config.parameter_schema import DriverParameters
from driver.hardware.hardware_commander import HardwareCommander


@dataclass
class DiagnosticsCache:
    last_publish_time: float = 0.0


class StatePublisher:
    def __init__(
        self,
        node,
        commander: HardwareCommander,
        params: DriverParameters,
    ):
        self._node = node
        self._commander = commander
        self._params = params
        self._joint_pub = node.create_publisher(JointState, params.joint_state_topic, 10)
        self._diag_pub = node.create_publisher(DiagnosticArray, params.diagnostics_topic, 10)
        self._tf_broadcaster = TransformBroadcaster(node) if params.publish_tf else None
        self._diagnostics_cache = DiagnosticsCache()

        self._joint_period = 1.0 / max(1e-3, params.joint_state_rate)
        self._joint_stop = threading.Event()
        self._joint_thread = threading.Thread(target=self._joint_worker, daemon=True)
        self._joint_thread.start()

        diag_period = 1.0 / max(1e-3, params.diagnostics_rate)
        self._diag_timer = node.create_timer(diag_period, self._publish_diagnostics)

    def shutdown(self) -> None:
        """Stop background state publishing thread."""
        self._joint_stop.set()
        if self._joint_thread.is_alive():
            self._joint_thread.join(timeout=1.0)

    # ------------------------------------------------------------------ threads
    def _joint_worker(self) -> None:
        while not self._joint_stop.is_set():
            try:
                self._publish_joint_state()
            except Exception as exc:  # pragma: no cover - defensive guard
                self._node.get_logger().error(f'Joint state publisher failed: {exc}')
            finally:
                # Wait for the configured period or exit early if stop requested.
                self._joint_stop.wait(self._joint_period)

    # ------------------------------------------------------------------ timers
    def _publish_joint_state(self):
        data = self._commander.read_joint_state()
        stamp = self._node.get_clock().now().to_msg()
        msg = JointState()
        msg.header.stamp = stamp
        msg.header.frame_id = self._params.tf_base_frame
        msg.name = list(data.names)
        msg.position = list(data.positions)
        msg.velocity = list(data.velocities)
        msg.effort = list(data.efforts)
        self._joint_pub.publish(msg)

        if self._tf_broadcaster:
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = self._params.tf_base_frame
            pose_msg.header.stamp = msg.header.stamp
            pose_msg.pose = self._commander.read_end_effector_pose()
            tf_msg = TransformStamped()
            tf_msg.header.stamp = msg.header.stamp
            tf_msg.header.frame_id = self._params.tf_base_frame
            tf_msg.child_frame_id = self._params.tf_tool_frame
            tf_msg.transform.translation.x = pose_msg.pose.position.x
            tf_msg.transform.translation.y = pose_msg.pose.position.y
            tf_msg.transform.translation.z = pose_msg.pose.position.z
            tf_msg.transform.rotation.x = pose_msg.pose.orientation.x
            tf_msg.transform.rotation.y = pose_msg.pose.orientation.y
            tf_msg.transform.rotation.z = pose_msg.pose.orientation.z
            tf_msg.transform.rotation.w = pose_msg.pose.orientation.w
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
