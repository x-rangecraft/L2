"""Publisher for /robot_driver/end_effector_pose based on HardwareCommander state.

This helper runs a background thread that periodically asks ``KinematicsSolver``
to compute the FK pose from the latest joint state (which is read internally
from ``HardwareCommander``) and publishes it as ``geometry_msgs/PoseStamped``.
It no longer depends on subscribing to ``/joint_states``, to avoid issues where
the self-subscription callback may not be reliably invoked.
"""
from __future__ import annotations

import threading
import time
from typing import Optional

from geometry_msgs.msg import PoseStamped

from driver.hardware.kinematics_solver import KinematicsSolver
from driver.utils.logging_utils import get_logger


class EndEffectorPosePublisher:
    def __init__(
        self,
        node,
        *,
        solver: KinematicsSolver,
        joint_state_topic: str,  # kept for backward compatibility, now unused
        pose_topic: str = '/robot_driver/end_effector_pose',
    ) -> None:
        self._node = node
        self._solver = solver
        self._logger = get_logger('end_effector_pose_publisher')
        self._pose_pub = node.create_publisher(PoseStamped, pose_topic, 10)
        self._stop = threading.Event()

        # Derive publish period from configured joint_state_rate so that the
        # pose topic roughly matches /joint_states frequency.
        joint_rate_hz: float = 30.0
        try:
            params = getattr(node, '_params', None)
            if params is not None:
                joint_rate_hz = float(getattr(params, 'joint_state_rate', joint_rate_hz))
        except Exception:
            # If anything goes wrong while probing params, fall back to default.
            pass
        # Guard against zero/negative rates and avoid a busy loop.
        self._period = 1.0 / max(1e-3, joint_rate_hz)

        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()
        self._logger.info(
            'EndEffectorPosePublisher initialized, publishing %s at ~%.2f Hz',
            pose_topic,
            joint_rate_hz,
        )

    def _worker(self) -> None:
        clock = self._node.get_clock()
        self._logger.info('EndEffectorPosePublisher worker thread started (polling HardwareCommander)')
        while not self._stop.is_set():
            try:
                start = time.time()

                # Let KinematicsSolver read the latest joint state from
                # HardwareCommander and compute FK. When positions are not
                # provided, compute_fk_pose() handles this internally.
                stamp = clock.now().to_msg()
                pose_msg, success = self._solver.compute_fk_pose(stamp=stamp)

                if not success:
                    self._logger.warning('FK computation failed, using fallback pose')

                self._pose_pub.publish(pose_msg)
            except Exception as exc:  # pragma: no cover - defensive guard
                self._logger.error('Error in end-effector pose worker: %s', exc, exc_info=True)

            # Maintain approximate target period, but allow prompt shutdown.
            elapsed = time.time() - start
            remaining = self._period - elapsed
            if remaining > 0:
                self._stop.wait(remaining)

        self._logger.info('EndEffectorPosePublisher worker thread stopped')

    def shutdown(self) -> None:
        """Stop background publishing thread."""
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)

