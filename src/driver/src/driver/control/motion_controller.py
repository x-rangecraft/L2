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
    # Approximate Cartesian workspace in base_link frame (meters), derived from
    # yam.urdf joint limits sampling. Used for basic input validation only.
    _WORKSPACE_X_MIN = -0.61
    _WORKSPACE_X_MAX = 0.60
    _WORKSPACE_Y_MIN = -0.59
    _WORKSPACE_Y_MAX = 0.61
    _WORKSPACE_Z_MIN = -0.37
    _WORKSPACE_Z_MAX = 0.72

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

        # Basic parameter validation (format + numeric range)
        if not self._validate_robot_command(msg):
            self._logger.error('Rejected /robot_command due to invalid parameters')
            return

        # Auto-exit zero-gravity mode if currently enabled
        if self._commander.get_zero_gravity():
            self._logger.info('Auto-exiting zero-gravity mode for robot_command')
            self._commander.set_zero_gravity(False)
            time.sleep(0.1)  # Brief pause to let mode switch settle

        self._commander.command_cartesian_pose(msg.pose, xyz_only=self._params.xyz_only_mode)
        self._mark_activity()
        self._preempt_event.set()  # Signal any ongoing motion to preempt

    def handle_joint_command(self, msg: JointState) -> None:
        # Basic parameter validation (format + numeric range)
        if not self._validate_joint_command(msg):
            self._logger.error('Rejected /joint_command due to invalid parameters')
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
    def _validate_robot_command(self, msg: PoseStamped) -> bool:
        """Validate PoseStamped for /robot_driver/robot_command.

        Checks:
        - position x/y/z must be finite and within configured workspace bounds;
        - orientation components must be finite and form a non-degenerate quaternion;
        - if xyz_only_mode is disabled, quaternion norm must be in a sane range.
        """
        pose = msg.pose
        p = pose.position
        q = pose.orientation

        # Position must be finite
        if not all(math.isfinite(v) for v in (p.x, p.y, p.z)):
            self._logger.error(
                'robot_command position contains non-finite values: x=%.3f, y=%.3f, z=%.3f',
                p.x,
                p.y,
                p.z,
            )
            return False

        # Position must lie within approximate workspace bounds
        if not (self._WORKSPACE_X_MIN <= p.x <= self._WORKSPACE_X_MAX):
            self._logger.error(
                'robot_command x=%.3f outside workspace [%.2f, %.2f]',
                p.x,
                self._WORKSPACE_X_MIN,
                self._WORKSPACE_X_MAX,
            )
            return False
        if not (self._WORKSPACE_Y_MIN <= p.y <= self._WORKSPACE_Y_MAX):
            self._logger.error(
                'robot_command y=%.3f outside workspace [%.2f, %.2f]',
                p.y,
                self._WORKSPACE_Y_MIN,
                self._WORKSPACE_Y_MAX,
            )
            return False
        if not (self._WORKSPACE_Z_MIN <= p.z <= self._WORKSPACE_Z_MAX):
            self._logger.error(
                'robot_command z=%.3f outside workspace [%.2f, %.2f]',
                p.z,
                self._WORKSPACE_Z_MIN,
                self._WORKSPACE_Z_MAX,
            )
            return False

        # Orientation must be finite
        q_vals = (q.x, q.y, q.z, q.w)
        if not all(math.isfinite(v) for v in q_vals):
            self._logger.error(
                'robot_command orientation contains non-finite values: '
                'x=%.6f, y=%.6f, z=%.6f, w=%.6f',
                q.x,
                q.y,
                q.z,
                q.w,
            )
            return False

        # Quaternion norm must be non-zero; allow a broad range and let commander normalize.
        norm_sq = sum(v * v for v in q_vals)
        if norm_sq < 1e-6:
            self._logger.error('robot_command orientation is near zero quaternion; rejecting')
            return False

        # When xyz_only_mode is disabled we expect a reasonably normalized quaternion.
        if not self._params.xyz_only_mode and not (0.25 <= norm_sq <= 4.0):
            self._logger.error(
                'robot_command orientation norm^2=%.6f out of expected range [0.25, 4.0]',
                norm_sq,
            )
            return False

        return True

    def _validate_joint_command(self, msg: JointState) -> bool:
        """Validate JointState for /robot_driver/joint_command.

        Checks:
        - required field for current mode (position/velocity/effort) is present;
        - name list non-empty and matches value list length;
        - all values are finite;
        - all joint names exist in current joint state.
        """
        mode = self._params.joint_command.mode
        if mode == 'position':
            buffer = list(msg.position or [])
            field_name = 'position'
        elif mode == 'velocity':
            buffer = list(msg.velocity or [])
            field_name = 'velocity'
        elif mode == 'effort':
            buffer = list(msg.effort or [])
            field_name = 'effort'
        else:
            self._logger.error('Unknown joint_command mode: %s', mode)
            return False

        if not msg.name:
            self._logger.error('joint_command missing joint names')
            return False
        if not buffer:
            self._logger.error('joint_command missing %s values for mode %s', field_name, mode)
            return False
        if len(msg.name) != len(buffer):
            self._logger.error(
                'joint_command name/value length mismatch: names=%d, %s=%d',
                len(msg.name),
                field_name,
                len(buffer),
            )
            return False

        # Values must be finite
        for name, value in zip(msg.name, buffer):
            if not math.isfinite(value):
                self._logger.error(
                    'joint_command %s for joint %s is non-finite: %r',
                    field_name,
                    name,
                    value,
                )
                return False

        # All joints must be known
        state = self._commander.read_joint_state()
        known = set(state.names)
        unknown = [name for name in msg.name if name not in known]
        if unknown:
            self._logger.error('joint_command contains unknown joints: %s', ', '.join(unknown))
            return False

        return True

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
            try:
                mapping[name] = float(value)
            except (TypeError, ValueError):
                self._logger.warning(
                    'Skipping non-numeric joint_command value for %s: %r',
                    name,
                    value,
                )
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
