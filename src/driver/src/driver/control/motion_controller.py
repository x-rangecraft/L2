"""Command handling for robot_driver."""
from __future__ import annotations

import math
import queue
import threading
import time
from typing import Callable, Dict, Iterable, Optional

from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from driver.config.parameter_schema import DriverParameters
from driver.hardware.hardware_commander import HardwareCommander
from driver.safety.safe_pose_executor import run_safe_pose_sequence
from driver.utils.logging_utils import get_logger


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
        self._joint_ramp_thread: Optional[threading.Thread] = None
        self._joint_ramp_stop = threading.Event()

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
        # Auto-exit zero-gravity mode if currently enabled
        if self._commander.get_zero_gravity():
            self._logger.info('Auto-exiting zero-gravity mode for joint_command')
            self._commander.set_zero_gravity(False)
            time.sleep(0.1)

        values = self._extract_joint_values(msg)
        if not values:
            self._logger.warning('Joint command missing required field for mode %s', self._params.joint_command.mode)
            return
        self._preempt_active_joint_ramp()
        self._start_joint_ramp(dict(values))
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

    def halt_and_safe(self, reason: str = '') -> bool:
        self._logger.warning('Halting motion: %s', reason)
        self._commander.stop_motion(reason)
        moved = self._commander.move_to_safe_pose()
        if not moved:
            self._logger.warning('SAFE_POSE unavailable; skipping move_to_safe_pose sequence')
        self._mark_activity()
        return moved

    def safe_pose_sequence(
        self,
        *,
        reason: str = '',
        exit_zero_gravity: bool = False,
        wait_time: float = 2.0,
        tolerance: float = 0.1,
        reenable_zero_gravity: bool = False,
        on_complete: Optional[Callable[[bool, float], None]] = None,
    ) -> bool:
        """Run the shared SAFE_POSE executor and return whether it completed successfully."""

        return run_safe_pose_sequence(
            commander=self._commander,
            halt_and_safe=self.halt_and_safe,
            logger=self._logger,
            reason=reason,
            exit_zero_gravity=exit_zero_gravity,
            wait_time=wait_time,
            tolerance=tolerance,
            reenable_zero_gravity=reenable_zero_gravity,
            on_complete=on_complete,
        )

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

    # ------------------------------------------------------------------ joint ramp helpers
    def _preempt_active_joint_ramp(self) -> None:
        if self._joint_ramp_thread and self._joint_ramp_thread.is_alive():
            self._joint_ramp_stop.set()
            self._joint_ramp_thread.join(timeout=0.1)
        self._joint_ramp_thread = None

    def _start_joint_ramp(self, targets: Dict[str, float]) -> None:
        stop_event = threading.Event()
        self._joint_ramp_stop = stop_event
        thread = threading.Thread(target=self._run_joint_ramp, args=(targets, stop_event), daemon=True)
        self._joint_ramp_thread = thread
        thread.start()

    def _run_joint_ramp(self, targets: Dict[str, float], stop_event: threading.Event) -> None:
        try:
            state = self._commander.read_joint_state()
            current_map = {name: pos for name, pos in zip(state.names, state.positions)}
            deltas = {}
            for name, goal in targets.items():
                if name not in current_map:
                    self._logger.warning('Joint %s missing from current state; skipping ramp', name)
                    return
                deltas[name] = goal - current_map[name]

            if not deltas:
                return

            max_delta = max(abs(delta) for delta in deltas.values())
            if max_delta < 1e-4:
                self._commander.command_joint_positions(targets.keys(), targets.values(), rate_limit=self._params.joint_command.rate_limit)
                self._mark_activity()
                return

            max_speed = max(self._params.joint_command.rate_limit, 1e-3)
            duration = max_delta / max_speed
            frame_rate = max(1.0, min(self._params.joint_state_rate, 120.0))
            steps = max(int(math.ceil(duration * frame_rate)), 2)
            step_dt = 1.0 / frame_rate

            joint_names = list(targets.keys())
            start_values = [current_map[name] for name in joint_names]
            deltas_list = [deltas[name] for name in joint_names]

            for step in range(1, steps + 1):
                if stop_event.is_set():
                    self._logger.info('Joint command ramp preempted (%d/%d steps)', step, steps)
                    return
                alpha = step / steps
                interp = [start + delta * alpha for start, delta in zip(start_values, deltas_list)]
                self._commander.command_joint_positions(joint_names, interp, rate_limit=self._params.joint_command.rate_limit)
                self._mark_activity()
                if step < steps:
                    time.sleep(step_dt)
        except Exception as exc:
            self._logger.error('Joint command ramp failed: %s', exc)
