"""Command handling for robot_driver."""
from __future__ import annotations

import queue
import threading
import time
from typing import Dict, Iterable, Optional

from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from .hardware_commander import HardwareCommander
from .logging_utils import get_logger
from .parameter_schema import DriverParameters


class MotionController:
    def __init__(self, params: DriverParameters, commander: HardwareCommander):
        self._params = params
        self._commander = commander
        self._logger = get_logger(__name__)
        self._lock = threading.RLock()
        self._last_command_time = time.monotonic()
        self._command_queue: "queue.Queue[Optional[tuple]]" = queue.Queue()
        self._preempt_event = threading.Event()
        self._running = True

    # ------------------------------------------------------------------ getters
    @property
    def last_command_time(self) -> float:
        return self._last_command_time

    def seconds_since_last_command(self) -> float:
        return time.monotonic() - self._last_command_time

    # ------------------------------------------------------------------ handlers
    def handle_robot_command(self, msg: PoseStamped) -> None:
        """Handle Cartesian pose command. Automatically exits zero-gravity if needed."""
        self._logger.debug('Got /robot_command: frame=%s', msg.header.frame_id)

        # Auto-exit zero-gravity mode if currently enabled
        if self._commander.get_zero_gravity():
            self._logger.info('Auto-exiting zero-gravity mode for robot_command')
            self._commander.set_zero_gravity(False)
            time.sleep(0.1)  # Brief pause to let mode switch settle

        self._commander.command_cartesian_pose(msg.pose, xyz_only=self._params.xyz_only_mode)
        self._mark_activity()
        self._preempt_event.set()  # Signal any ongoing motion to preempt

    def handle_joint_command(self, msg: JointState) -> None:
        if not self._params.joint_command.enabled:
            self._logger.debug('Joint direct control disabled; ignoring command')
            return

        # Auto-exit zero-gravity mode if currently enabled
        if self._commander.get_zero_gravity():
            self._logger.info('Auto-exiting zero-gravity mode for joint_command')
            self._commander.set_zero_gravity(False)
            time.sleep(0.1)

        values = self._extract_joint_values(msg)
        if not values:
            self._logger.warning('Joint command missing required field for mode %s', self._params.joint_command.mode)
            return
        self._commander.command_joint_positions(values.keys(), values.values(), rate_limit=self._params.joint_command.rate_limit)
        self._mark_activity()
        self._preempt_event.set()

    def execute_trajectory(self, goal_handle) -> FollowJointTrajectory.Result:
        """Execute FollowJointTrajectory action with preemption support."""
        goal: FollowJointTrajectory.Goal = goal_handle.request
        joint_names = list(goal.trajectory.joint_names)
        if not joint_names:
            raise ValueError('Trajectory goal missing joint names')

        # Auto-exit zero-gravity mode if currently enabled
        if self._commander.get_zero_gravity():
            self._logger.info('Auto-exiting zero-gravity mode for trajectory')
            self._commander.set_zero_gravity(False)
            time.sleep(0.1)

        self._preempt_event.clear()
        prev_time = 0.0

        for point in goal.trajectory.points:
            # Check for preemption
            if self._preempt_event.is_set() or goal_handle.is_cancel_requested:
                self._logger.warning('Trajectory preempted')
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                result.error_string = 'Preempted by new command'
                return result

            self._apply_trajectory_point(joint_names, point)
            target_time = self._duration_to_float(point.time_from_start)
            sleep_time = max(0.0, target_time - prev_time)
            if sleep_time > 0:
                time.sleep(sleep_time)
            prev_time = target_time

        self._mark_activity()
        duration = prev_time
        self._logger.info('Executed FollowJointTrajectory with %d points in %.2fs', len(goal.trajectory.points), duration)

        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        result.error_string = ''
        return result

    def halt_and_safe(self, reason: str = '') -> None:
        self._logger.warning('Halting motion: %s', reason)
        self._commander.stop_motion(reason)
        self._commander.move_to_safe_pose()
        self._mark_activity()

    # ------------------------------------------------------------------ helpers
    def _extract_joint_values(self, msg: JointState) -> Dict[str, float]:
        mapping: Dict[str, float] = {}
        buffer: Iterable[float]
        mode = self._params.joint_command.mode
        if mode == 'position' and msg.position:
            buffer = msg.position
        elif mode == 'velocity' and msg.velocity:
            buffer = msg.velocity
        elif mode == 'effort' and msg.effort:
            buffer = msg.effort
        else:
            return mapping
        for name, value in zip(msg.name, buffer):
            mapping[name] = float(value)
        return mapping

    def _apply_trajectory_point(self, joint_names: list[str], point: JointTrajectoryPoint) -> None:
        if not point.positions:
            return
        mapping = dict(zip(joint_names, point.positions))
        self._commander.command_joint_positions(mapping.keys(), mapping.values(), rate_limit=self._params.joint_command.rate_limit)

    def _mark_activity(self) -> None:
        with self._lock:
            self._last_command_time = time.monotonic()

    @staticmethod
    def _duration_to_float(duration) -> float:
        return float(duration.sec) + float(duration.nanosec) / 1e9
