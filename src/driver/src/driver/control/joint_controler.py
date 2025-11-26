"""Command handling for robot_driver."""
from __future__ import annotations

import math
import queue
import threading
import time
from typing import Callable, Dict, Iterable, Optional, List, TYPE_CHECKING

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectoryPoint
from robot_driver.action import JointCommand as JointCommandAction
from robot_driver.msg import JointContact

from driver.config.parameter_schema import DriverParameters
from driver.hardware.hardware_commander import HardwareCommander, JointStateData
from driver.safety.safe_pose_manager import SafePoseManager
from driver.utils.logging_utils import get_logger

if TYPE_CHECKING:
    from driver.control.zero_gravity_manager import ZeroGravityManager


class JointControler:
    # Approximate Cartesian workspace in base_link frame (meters), derived from
    # yam.urdf joint limits sampling. Used for basic input validation only.
    _WORKSPACE_X_MIN = -0.61
    _WORKSPACE_X_MAX = 0.60
    _WORKSPACE_Y_MIN = -0.59
    _WORKSPACE_Y_MAX = 0.61
    _WORKSPACE_Z_MIN = -0.37
    _WORKSPACE_Z_MAX = 0.72
    _CONTACT_VELOCITY_EPS = 1e-3
    _CONTACT_EFFORT_THRESHOLD = 5.0
    _MIN_SPEED_SCALE = 0.05
    _ZERO_GRAVITY_SERVICE_TIMEOUT = 5.0

    def __init__(
        self,
        node: Node,
        params: DriverParameters,
        commander: HardwareCommander,
        *,
        zero_gravity_manager: Optional["ZeroGravityManager"] = None,
        zero_gravity_service: Optional[str] = None,
        safe_pose_manager: Optional[SafePoseManager] = None,
    ):
        self._node = node
        self._params = params
        self._commander = commander
        self._zero_gravity_manager = zero_gravity_manager
        self._zero_gravity_service_name = zero_gravity_service or params.zero_gravity_service
        self._zero_gravity_client = None
        if self._zero_gravity_service_name:
            try:
                self._zero_gravity_client = node.create_client(
                    SetBool,
                    self._zero_gravity_service_name,
                    callback_group=ReentrantCallbackGroup(),
                )
            except Exception:
                self._zero_gravity_client = None
        self._logger = get_logger(__name__)
        self._lock = threading.RLock()
        self._last_command_time = time.monotonic()
        self._command_queue: "queue.Queue[Optional[tuple]]" = queue.Queue()
        self._preempt_event = threading.Event()
        self._running = True
        self._joint_ramp_thread: Optional[threading.Thread] = None
        self._joint_ramp_stop = threading.Event()
        # Dedicated worker for blocking Cartesian moves so ROS timers stay responsive.
        # A per-command stop event allows new /robot_command messages to preempt
        # the previous ramp without blocking the ROS executor.
        self._robot_command_thread: Optional[threading.Thread] = None
        self._robot_command_stop = threading.Event()
        self._safe_pose_manager = safe_pose_manager

    # ------------------------------------------------------------------ zero-gravity helpers
    def _zero_gravity_enabled(self) -> bool:
        if self._zero_gravity_manager is not None:
            return self._zero_gravity_manager.refresh_state()
        return self._commander.get_zero_gravity()

    def _toggle_zero_gravity(self, enabled: bool, label_tag: str = '') -> bool:
        """Toggle zero-gravity mode using internal manager/commander.

        NOTE: Previously this method tried to call the zero_gravity ROS service
        using spin_until_future_complete(), which could deadlock the executor
        when called from within an action execute callback. To ensure the driver
        never blocks indefinitely, we now directly use the manager or commander
        which are same-node objects and will not block.
        """
        # Use manager or commander directly to avoid blocking service calls.
        # This ensures zero-gravity toggle never causes executor deadlock.
        if self._zero_gravity_manager is not None:
            result = self._zero_gravity_manager.set_state(enabled)
            if not result:
                self._logger.warning(
                    'zero_gravity manager failed to set state=%s%s', enabled, label_tag
                )
            return result
        result = self._commander.set_zero_gravity(enabled)
        if not result:
            self._logger.warning(
                'zero_gravity commander failed to set state=%s%s', enabled, label_tag
            )
        return result

    def _exit_zero_gravity_if_needed(self, label_tag: str = '', *, force: bool = False) -> bool:
        """Exit zero-gravity if currently enabled. Returns True if it was disabled."""
        if not force and not self._zero_gravity_enabled():
            return False
        if self._toggle_zero_gravity(False, label_tag):
            time.sleep(0.1)
            return True
        return False

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

        # Heavy IK + joint ramp must not block the rclpy executor thread; otherwise
        # timers (including /joint_states publisher) are stalled. Mirror the
        # joint_command design by offloading the actual motion to a worker thread.
        # If there is an ongoing cartesian ramp, signal its stop event so the
        # internal ramp loop can exit early and let this new command take over.
        if self._robot_command_thread and self._robot_command_thread.is_alive():
            self._logger.info('Preempting previous /robot_command with new target')
            self._robot_command_stop.set()

        pose = msg.pose
        stop_event = threading.Event()
        self._robot_command_stop = stop_event

        # Record activity at dispatch time so the watchdog's "time since last
        # command" reflects command heartbeats rather than ramp completion.
        self._mark_activity()

        def _worker():
            try:
                # Auto-exit zero-gravity mode if currently enabled
                if self._zero_gravity_enabled():
                    self._logger.info('Auto-exiting zero-gravity mode for robot_command')
                    if not self._exit_zero_gravity_if_needed(' [robot_command]'):
                        self._logger.warning('Failed to exit zero-gravity; robot_command may have no effect')

                self._commander.command_cartesian_pose(
                    pose,
                    xyz_only=self._params.xyz_only_mode,
                    stop_event=stop_event,
                )
            except Exception as exc:  # pragma: no cover - defensive guard
                self._logger.error('robot_command worker failed: %s', exc)

        self._robot_command_thread = threading.Thread(target=_worker, daemon=True)
        self._robot_command_thread.start()
        self._preempt_event.set()  # Signal any ongoing motion to preempt

    def validate_robot_pose(self, msg: PoseStamped) -> bool:
        """Expose robot_command validation for action-based callers."""
        return self._validate_robot_command(msg)

    def handle_joint_command(self, msg: JointState) -> None:
        # Basic parameter validation (format + numeric range)
        if not self._validate_joint_command(msg):
            self._logger.error('Rejected /joint_command due to invalid parameters')
            return

        # Auto-exit zero-gravity mode if currently enabled
        if self._zero_gravity_enabled():
            self._logger.info('Auto-exiting zero-gravity mode for joint_command')
            if not self._exit_zero_gravity_if_needed(' [joint_command]'):
                self._logger.warning('Failed to exit zero-gravity; joint_command may be ignored')

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
        if self._zero_gravity_enabled():
            self._logger.info('Auto-exiting zero-gravity mode for trajectory')
            if not self._exit_zero_gravity_if_needed(' [trajectory]'):
                self._logger.warning('Failed to exit zero-gravity; trajectory execution may stall')

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

    def execute_joint_action(self, goal_handle) -> JointCommandAction.Result:
        """Execute /robot_driver/action/joint goal with detailed feedback/contacts."""

        goal: JointCommandAction.Goal = goal_handle.request
        joint_msg = goal.target.joint_state
        label = goal.options.label or ''
        label_tag = f' [{label}]' if label else ''

        if not self._validate_joint_command(joint_msg):
            self._logger.error('Joint action%s rejected: invalid goal', label_tag)
            raw_target = self._joint_state_from_mapping(self._extract_joint_values(joint_msg))
            return self._finalize_joint_action(
                success=False,
                code='INVALID_GOAL',
                last_error='JointCommand goal failed validation',
                target_state=raw_target,
            )

        values = self._extract_joint_values(joint_msg)
        if not values:
            self._logger.error('Joint action%s rejected: missing joint data', label_tag)
            raw_target = self._joint_state_from_mapping(values)
            return self._finalize_joint_action(
                success=False,
                code='INVALID_GOAL',
                last_error='No joint data provided for action',
                target_state=raw_target,
            )

        state_snapshot = self._commander.read_joint_state()
        current_map = {name: pos for name, pos in zip(state_snapshot.names, state_snapshot.positions)}

        try:
            targets = self._build_joint_targets(values, current_map, relative=goal.target.relative)
        except ValueError as exc:
            self._logger.error('Joint action%s rejected: %s', label_tag, exc)
            raw_target = self._joint_state_from_mapping(values)
            return self._finalize_joint_action(
                success=False,
                code='INVALID_GOAL',
                last_error=str(exc),
                target_state=raw_target,
                snapshot=state_snapshot,
            )

        target_state_msg = self._joint_state_from_mapping(targets)

        zero_gravity_was_on = self._zero_gravity_enabled()
        disabled_zero_gravity = False
        if zero_gravity_was_on:
            self._logger.info('Joint action%s: exiting zero-gravity (auto)', label_tag)
            disabled_zero_gravity = self._exit_zero_gravity_if_needed(label_tag)
            if not disabled_zero_gravity:
                self._logger.warning('Joint action%s could not exit zero-gravity; motion may not occur', label_tag)

        self._preempt_active_joint_ramp()
        self._preempt_event.set()
        time.sleep(0.01)
        self._preempt_event.clear()

        try:
            speed_scale = goal.target.speed_scale if math.isfinite(goal.target.speed_scale) else 1.0
            speed_scale = max(0.0, min(1.0, speed_scale))
            if speed_scale <= 0.0:
                self._logger.warning('Joint action%s speed_scale=%.3f; clamping to %.2f', label_tag, speed_scale, self._MIN_SPEED_SCALE)
            effective_scale = max(speed_scale, self._MIN_SPEED_SCALE)
            rate_limit = max(self._params.joint_command.rate_limit * effective_scale, 1e-3)

            joint_names = list(targets.keys())
            start_map = {name: current_map.get(name, targets[name]) for name in joint_names}
            start_values = [start_map[name] for name in joint_names]
            deltas = [targets[name] - start_map[name] for name in joint_names]

            if not deltas:
                self._logger.info('Joint action%s: no joints specified; treating as no-op', label_tag)
                return self._finalize_joint_action(True, 'NOOP', '', target_state_msg, snapshot=state_snapshot)

            max_delta = max(abs(delta) for delta in deltas)
            if max_delta < 1e-4:
                self._logger.info('Joint action%s: already at target', label_tag)
                self._mark_activity()
                return self._finalize_joint_action(True, 'ALREADY_AT_TARGET', '', target_state_msg, snapshot=state_snapshot)

            duration = max_delta / max(rate_limit, 1e-6)
            frame_rate = max(1.0, min(self._params.joint_state_rate, 120.0))
            steps = max(int(math.ceil(duration * frame_rate)), 2)
            step_dt = 1.0 / frame_rate
            total_duration = steps * step_dt

            timeout_s = self._duration_to_float(goal.options.timeout)
            deadline = time.monotonic() + timeout_s if timeout_s > 0.0 else None

            self._mark_activity()
            self._publish_joint_feedback(
                goal_handle,
                phase='queued',
                completion_hint=0.0,
                remaining_time=total_duration,
                target_state=target_state_msg,
                start_map=start_map,
                target_map=targets,
            )

            for step in range(1, steps + 1):
                if self._preempt_event.is_set():
                    err = 'Preempted by newer command'
                    self._logger.warning('Joint action%s preempted', label_tag)
                    return self._finalize_joint_action(False, 'PREEMPTED', err, target_state_msg)
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    err = 'Goal canceled by client'
                    self._logger.info('Joint action%s canceled by client', label_tag)
                    return self._finalize_joint_action(False, 'CANCELED', err, target_state_msg)
                if deadline and time.monotonic() > deadline:
                    err = f'Joint action timed out after {timeout_s:.2f}s'
                    self._logger.error('Joint action%s timed out', label_tag)
                    return self._finalize_joint_action(False, 'TIMEOUT', err, target_state_msg)

                alpha = step / steps
                interp = [start + delta * alpha for start, delta in zip(start_values, deltas)]
                self._commander.command_joint_positions(joint_names, interp, rate_limit=rate_limit)
                self._mark_activity()

                remaining = max(0.0, (steps - step) * step_dt)
                self._publish_joint_feedback(
                    goal_handle,
                    phase='ramping',
                    completion_hint=alpha,
                    remaining_time=remaining,
                    target_state=target_state_msg,
                    start_map=start_map,
                    target_map=targets,
                )

                if step < steps:
                    time.sleep(step_dt)

            final_snapshot = self._publish_joint_feedback(
                goal_handle,
                phase='settling',
                completion_hint=1.0,
                remaining_time=0.0,
                target_state=target_state_msg,
                start_map=start_map,
                target_map=targets,
            )
            self._logger.info('Joint action%s completed (%d joints, %.2fs)', label_tag, len(joint_names), total_duration)
            return self._finalize_joint_action(True, 'SUCCESS', '', target_state_msg, snapshot=final_snapshot)
        finally:
            if zero_gravity_was_on and disabled_zero_gravity:
                self._logger.info('Joint action%s: restoring zero-gravity mode', label_tag)
                if not self._toggle_zero_gravity(True, label_tag):
                    self._logger.warning('Joint action%s failed to re-enable zero-gravity', label_tag)

    def halt_and_safe(self, reason: str = '') -> bool:
        """Stop current motion and attempt to move towards SAFE_POSE."""
        self._logger.warning('Halting motion: %s', reason)
        self._commander.stop_motion(reason)
        moved = False
        manager = self._safe_pose_manager
        if manager is not None:
            moved = manager.move_to_safe_pose()
            if not moved:
                self._logger.warning('SAFE_POSE unavailable; skipping move_to_safe_pose sequence')
        else:
            self._logger.warning('SafePoseManager not configured; skipping move_to_safe_pose sequence')
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
        """Run SAFE_POSE sequence and return whether it completed successfully."""

        manager = self._safe_pose_manager
        if manager is None:
            self._logger.warning('SafePoseManager not configured; SAFE_POSE sequence skipped')
            if on_complete:
                on_complete(False, float('inf'))
            return False

        # Optionally exit zero-gravity before moving.
        if exit_zero_gravity and self._zero_gravity_enabled():
            self._logger.info('Exiting zero-gravity before SAFE_POSE sequence')
            if not self._toggle_zero_gravity(False, ' [safe_pose]'):
                self._logger.warning('Failed to exit zero-gravity; SAFE_POSE motion may not occur')

        self._logger.info(
            'SAFE_POSE sequence started, reason=%s',
            reason or '<unknown>',
        )
        moved = self.halt_and_safe(reason)
        if not moved:
            self._logger.warning('SAFE_POSE movement did not complete; skipping pose verification.')
            if on_complete:
                on_complete(False, float('inf'))
            return False

        if wait_time > 0:
            self._logger.info('Waiting %.1fs for SAFE_POSE arrival', wait_time)
            time.sleep(wait_time)

        status = manager.check_at_safe_pose(tolerance=tolerance)
        if not status.in_pose:
            self._logger.warning(
                'Robot may not be at SAFE_POSE; max |Δ|=%.3f rad (tolerance %.3f).',
                status.max_error,
                tolerance,
            )
            if on_complete:
                on_complete(False, status.max_error)
            return False

        self._logger.info('SAFE_POSE reached (max |Δ|=%.3f rad)', status.max_error)

        if reenable_zero_gravity:
            self._logger.info('Re-enabling zero-gravity after SAFE_POSE')
            if not self._toggle_zero_gravity(True, ' [safe_pose]'):
                self._logger.warning('Failed to re-enable zero-gravity after SAFE_POSE')

        if on_complete:
            on_complete(True, status.max_error)
        return True

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

    def _publish_joint_feedback(
        self,
        goal_handle,
        *,
        phase: str,
        completion_hint: float,
        remaining_time: float,
        target_state: JointState,
        start_map: Optional[Dict[str, float]] = None,
        target_map: Optional[Dict[str, float]] = None,
        last_error: str = '',
    ) -> JointStateData:
        snapshot = self._commander.read_joint_state()
        completion = completion_hint
        if start_map and target_map:
            completion = self._compute_completion_ratio(snapshot, start_map, target_map)
        feedback = JointCommandAction.Feedback()
        feedback.phase = phase
        feedback.completion_ratio = max(0.0, min(1.0, completion))
        feedback.remaining_time = self._duration_from_seconds(max(0.0, remaining_time))
        feedback.current_state = self._to_joint_state_msg(snapshot)
        feedback.target_state = target_state
        feedback.contact_state = self._build_contact_state(snapshot)
        feedback.last_error = last_error
        goal_handle.publish_feedback(feedback)
        return snapshot

    def _finalize_joint_action(
        self,
        success: bool,
        code: str,
        last_error: str,
        target_state: JointState,
        *,
        snapshot: Optional[JointStateData] = None,
    ) -> JointCommandAction.Result:
        if snapshot is None:
            snapshot = self._commander.read_joint_state()
        result = JointCommandAction.Result()
        result.success = success
        result.result_code = code
        result.last_error = last_error
        result.target_state = target_state
        result.final_state = self._to_joint_state_msg(snapshot)
        result.contact_state = self._build_contact_state(snapshot)
        return result

    def _build_joint_targets(
        self,
        values: Dict[str, float],
        current_map: Dict[str, float],
        *,
        relative: bool,
    ) -> Dict[str, float]:
        targets: Dict[str, float] = {}
        for name, value in values.items():
            if relative:
                if name not in current_map:
                    raise ValueError(f'Joint {name} missing from current state for relative goal')
                targets[name] = current_map[name] + value
            else:
                targets[name] = value
        return targets

    def _build_contact_state(self, state: JointStateData) -> List[JointContact]:
        contacts: List[JointContact] = []
        for idx, name in enumerate(state.names):
            position = state.positions[idx] if idx < len(state.positions) else 0.0
            velocity = state.velocities[idx] if idx < len(state.velocities) else 0.0
            effort = state.efforts[idx] if idx < len(state.efforts) else 0.0
            temp_mos = state.temp_mos[idx] if idx < len(state.temp_mos) else float('nan')
            temp_rotor = state.temp_rotor[idx] if idx < len(state.temp_rotor) else float('nan')
            contact = JointContact()
            contact.name = name
            contact.effort = float(effort)
            contact.velocity = float(velocity)
            contact.position = float(position)
            contact.temp_mos = float(temp_mos)
            contact.temp_rotor = float(temp_rotor)
            contact.note = ''
            contacts.append(contact)
        return contacts

    @staticmethod
    def _to_joint_state_msg(data: JointStateData) -> JointState:
        msg = JointState()
        msg.name = list(data.names)
        msg.position = list(data.positions)
        msg.velocity = list(data.velocities)
        msg.effort = list(data.efforts)
        return msg

    @staticmethod
    def _joint_state_from_mapping(mapping: Dict[str, float]) -> JointState:
        msg = JointState()
        msg.name = list(mapping.keys())
        msg.position = [float(mapping[name]) for name in msg.name]
        return msg

    @staticmethod
    def _duration_from_seconds(seconds: float) -> Duration:
        seconds = max(0.0, float(seconds))
        msg = Duration()
        msg.sec = int(seconds)
        msg.nanosec = int((seconds - msg.sec) * 1e9)
        return msg

    @staticmethod
    def _state_positions_map(state: JointStateData) -> Dict[str, float]:
        return {name: state.positions[idx] for idx, name in enumerate(state.names)}

    def _compute_completion_ratio(
        self,
        snapshot: JointStateData,
        start_map: Dict[str, float],
        target_map: Dict[str, float],
    ) -> float:
        current_map = self._state_positions_map(snapshot)
        total_span = 0.0
        remaining = 0.0
        for name, target in target_map.items():
            start_value = start_map.get(name, target)
            total_span = max(total_span, abs(target - start_value))
            remaining = max(remaining, abs(target - current_map.get(name, target)))
        if total_span < 1e-6:
            return 1.0 if remaining < 1e-6 else 0.0
        return max(0.0, min(1.0, 1.0 - (remaining / total_span)))

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
