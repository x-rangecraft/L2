"""Thin abstraction over the vendor SDK (I2RT) with graceful fallbacks."""
from __future__ import annotations

import math
import threading
import time
from copy import deepcopy
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Tuple

import numpy as np
from geometry_msgs.msg import Pose

from driver.hardware.can_interface_manager import ensure_ready
from driver.hardware.robot_description_loader import RobotDescription
from driver.utils.logging_utils import get_logger


@dataclass
class JointStateData:
    names: List[str] = field(default_factory=list)
    positions: List[float] = field(default_factory=list)
    velocities: List[float] = field(default_factory=list)
    efforts: List[float] = field(default_factory=list)


@dataclass
class CartesianTarget:
    """Cartesian pose target with position + quaternion orientation."""
    x: float
    y: float
    z: float
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    qw: float = 1.0


def _rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Convert roll/pitch/yaw (rad) into a 3x3 rotation matrix."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return rz @ ry @ rx


def _quaternion_to_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Convert quaternion to 3x3 rotation matrix."""
    # Normalize quaternion
    norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if norm < 1e-8:
        return np.eye(3)

    qx, qy, qz, qw = qx/norm, qy/norm, qz/norm, qw/norm

    # Build rotation matrix
    xx = qx * qx
    yy = qy * qy
    zz = qz * qz
    xy = qx * qy
    xz = qx * qz
    yz = qy * qz
    wx = qw * qx
    wy = qw * qy
    wz = qw * qz

    return np.array([
        [1 - 2*(yy + zz), 2*(xy - wz), 2*(xz + wy)],
        [2*(xy + wz), 1 - 2*(xx + zz), 2*(yz - wx)],
        [2*(xz - wy), 2*(yz + wx), 1 - 2*(xx + yy)]
    ])


def _pose_to_target(pose: Pose) -> CartesianTarget:
    """Convert geometry_msgs/Pose to CartesianTarget."""
    return CartesianTarget(
        x=pose.position.x,
        y=pose.position.y,
        z=pose.position.z,
        qx=pose.orientation.x,
        qy=pose.orientation.y,
        qz=pose.orientation.z,
        qw=pose.orientation.w
    )


class HardwareCommander:
    """Wraps low-level hardware access points.

    The initial implementation keeps an in-memory mirror so the ROS node can run on
    developer machines without hardware attached. When the I2RT SDK is available the
    hooks in ``_maybe_import_sdk`` can be extended to talk to the actual device.
    """

    def __init__(
        self,
        *,
        joint_limit_rate: float = 0.5,
        gripper_type: Optional[str] = None,
        robot_description: Optional[RobotDescription] = None,
    ):
        self._logger = get_logger(__name__)
        self._lock = threading.RLock()
        self._zero_gravity = False
        self._joint_limit_rate = joint_limit_rate
        self._can_channel = 'can0'
        self._gripper_type = gripper_type or "linear_4310"
        self._robot = None
        self._kinematics = None
        self._sdk_commander = self._maybe_import_sdk()
        self._robot_description = robot_description or RobotDescription()
        self._joint_state = self._initialize_joint_state()
        # Require a non-empty joint list so that downstream components
        # (JointControler, SAFE_POSE manager, etc.) can reliably validate
        # JointState/JointCommand messages. If this is empty it almost always
        # indicates a configuration issue (e.g. robot_description_file not
        # pointing to the installed robot_description.yaml), so fail fast.
        if not self._joint_state.names:
            self._logger.error(
                'HardwareCommander initialised with empty joint list; '
                'ensure robot_description_file points to a YAML that defines joints '
                '(including the gripper entry, if present).',
            )
            raise ValueError('HardwareCommander requires a non-empty joint list from RobotDescription')
        self._joint_index: Dict[str, int] = {name: idx for idx, name in enumerate(self._joint_state.names)}
        self._ee_pose = Pose()
        self._last_read_time = 0.0
        self._default_kp: Optional[np.ndarray] = None
        self._default_kd: Optional[np.ndarray] = None

    def _maybe_import_sdk(self):
        try:
            # Try importing I2RT SDK classes
            from i2rt.robots.get_robot import get_yam_robot  # type: ignore
            from i2rt.robots.kinematics import Kinematics  # type: ignore
            from i2rt.robots.utils import GripperType  # type: ignore

            self._logger.info('I2RT SDK available, will use hardware backend')
            # Don't initialize yet, wait for connect()
            return {'get_yam_robot': get_yam_robot, 'Kinematics': Kinematics, 'GripperType': GripperType}
        except Exception as e:  # pragma: no cover
            self._logger.warning('I2RT SDK not found (%s); running in dummy commander mode', e)
            return None

    def _initialize_joint_state(self) -> JointStateData:
        """Initialise the in-memory joint state mirror.

        SAFE_POSE semantics are intentionally not used here so that this class
        remains a thin abstraction over the underlying hardware.  The initial
        joint layout is derived from the robot description (if available) and
        all positions/velocities/efforts start at zero.
        """
        names: List[str] = []
        if self._robot_description:
            names = list(self._robot_description.joint_names())

        count = len(names)
        zeros = [0.0] * count
        return JointStateData(
            names=names,
            positions=zeros[:],
            velocities=zeros[:],
            efforts=zeros[:],
        )

    # ------------------------------------------------------------------ lifecycle
    def connect(self, can_channel: str, *, reset_script: Optional[str] = None) -> None:
        self._can_channel = can_channel
        ensure_ready(can_channel, reset_script=reset_script)

        if self._sdk_commander:
            try:
                GripperType = self._sdk_commander['GripperType']
                gripper_map = {
                    'linear_4310': GripperType.LINEAR_4310,
                    'linear_3507': GripperType.LINEAR_3507,
                    'crank_4310': GripperType.CRANK_4310,
                    'no_gripper': GripperType.NO_GRIPPER,
                }
                gripper_enum = gripper_map.get(self._gripper_type.lower(), GripperType.LINEAR_4310)

                get_yam_robot = self._sdk_commander['get_yam_robot']
                Kinematics = self._sdk_commander['Kinematics']

                self._robot = get_yam_robot(
                    channel=can_channel,
                    gripper_type=gripper_enum,
                    zero_gravity_mode=self._zero_gravity
                )

                # Initialize kinematics with the appropriate XML
                xml_path = gripper_enum.get_xml_path()
                self._kinematics = Kinematics(xml_path, "grasp_site")

                self._logger.info('Hardware connected via I2RT SDK on %s', can_channel)

                # Read initial state from hardware
                self._update_state_from_hardware()
                self._cache_default_gains()
            except Exception as exc:
                self._logger.error('Failed to initialize I2RT robot: %s', exc)
                self._robot = None
                self._kinematics = None
        else:
            self._logger.info('Hardware commander connected to %s (dummy mode)', can_channel)

    def disconnect(self) -> None:
        if self._robot:
            try:
                self._robot.close()
            except Exception as exc:  # pragma: no cover
                self._logger.error('Error disconnecting SDK robot: %s', exc)
            self._robot = None
            self._kinematics = None
        self._logger.info('Hardware commander disconnected')

    # ------------------------------------------------------------------ commands
    def set_zero_gravity(self, enabled: bool) -> bool:
        # Short lock: check state and snapshot pointers/buffers
        with self._lock:
            if self._zero_gravity == enabled:
                self._logger.debug('Zero-gravity already %s; skip', enabled)
                return True
            robot = self._robot
            default_kp = self._default_kp.copy() if self._default_kp is not None else None
            default_kd = self._default_kd.copy() if self._default_kd is not None else None

        if not robot:
            self._logger.info('Zero-gravity toggled to %s (simulation mode)', enabled)
            with self._lock:
                self._zero_gravity = enabled
            return True

        try:
            if enabled:
                if not self._ensure_gain_snapshot():
                    self._logger.warning('Unable to snapshot joint gains; zero-gravity aborted')
                    return False
                if hasattr(robot, 'zero_torque_mode'):
                    robot.zero_torque_mode()
                else:
                    self._logger.warning('SDK robot missing zero_torque_mode; zero-gravity unavailable')
                    return False
            else:
                if default_kp is not None and default_kd is not None:
                    robot.update_kp_kd(default_kp, default_kd)
                current = robot.get_joint_pos()
                robot.command_joint_pos(current)

            with self._lock:
                self._zero_gravity = enabled
            self._logger.info('Zero-gravity mode -> %s', enabled)
            return True
        except Exception as exc:
            self._logger.error('Failed to set zero_gravity: %s', exc)
            return False

    def command_joint_positions(self, names: Iterable[str], values: Iterable[float], *, rate_limit: Optional[float] = None) -> None:
        mapping = dict(zip(names, values))
        robot = None
        current = None
        joint_index: Optional[Dict[str, int]] = None

        # Step 1: update local cache under lock (short critical section)
        with self._lock:
            self._apply_joint_delta(mapping, rate_limit or self._joint_limit_rate)
            self._logger.debug('Updated joints: %s', mapping)

            # Snapshot data needed for SDK call so the lock can be released before blocking
            robot = self._robot
            if robot:
                try:
                    current = np.array(robot.get_joint_pos())
                    joint_index = dict(self._joint_index)
                except Exception as exc:  # pragma: no cover
                    self._logger.warning('SDK get_joint_pos failed: %s', exc)
                    robot = None

        # Step 2: perform potentially blocking SDK call without holding the lock
        if robot is not None and current is not None and joint_index is not None:
            try:
                for name, value in mapping.items():
                    idx = joint_index.get(name)
                    if idx is None or idx >= len(current):
                        self._logger.warning('Unknown joint %s', name)
                        continue
                    current[idx] = value
                robot.command_joint_pos(current)
            except Exception as exc:
                self._logger.warning('SDK command_joint_pos failed: %s', exc)

    def command_cartesian_pose(
        self,
        pose: Pose,
        *,
        xyz_only: bool = False,
        stop_event: Optional[threading.Event] = None,
    ) -> None:
        """Command a Cartesian pose. This method will solve IK and ramp joints smoothly."""
        # Minimal lock: store target pose and snapshot handles
        with self._lock:
            self._logger.info('Received cartesian command xyz_only=%s', xyz_only)
            self._ee_pose = deepcopy(pose)
            robot = self._robot
            kinematics = self._kinematics

        if not robot or not kinematics:
            self._logger.debug('No SDK/kinematics available, storing target only')
            return

        try:
            # Solve IK (reads robot state internally)
            target = _pose_to_target(pose)
            success, joint_solution = self.solve_ik(target, xyz_only=xyz_only)

            if not success or joint_solution is None:
                self._logger.warning('IK failed for target pose')
                return

            # Clamp and ramp similar to SAFE_POSE to avoid step changes
            joint_solution = self._clamp_joint_array(joint_solution)
            ramp_ok, duration, steps, max_delta, preempted = self._ramp_from_current_to_target(
                joint_solution,
                label='cartesian_pose',
                min_duration=0.25,
                max_duration=5.0,
                stop_event=stop_event,
            )
            if ramp_ok:
                self._logger.debug(
                    'Commanded cartesian pose via IK (ramp %.2fs, %d steps, max Δ=%.3f rad)',
                    duration,
                    steps,
                    max_delta,
                )
            elif preempted:
                # Motion was intentionally preempted by a newer command; do not
                # force a final step move to the old target.
                self._logger.info('Cartesian pose ramp preempted; skipping fallback step move')
            else:
                self._logger.warning('Cartesian pose ramp degraded; fall back to step move')
                robot.command_joint_pos(joint_solution)
        except Exception as exc:
            self._logger.error('Failed to command cartesian pose: %s', exc)


    def _ramp_from_current_to_target(
        self,
        target: np.ndarray,
        *,
        label: str,
        min_duration: float,
        max_duration: float,
        stop_event: Optional[threading.Event] = None,
    ) -> Tuple[bool, float, int, float, bool]:
        robot = self._robot
        if not robot:
            return False, 0.0, 0, 0.0, False
        try:
            current = np.array(robot.get_joint_pos(), dtype=float)
        except Exception as exc:
            self._logger.warning('Failed to read current pose for %s: %s', label, exc)
            return False, 0.0, 0, 0.0, False

        return self._execute_joint_ramp(
            current,
            np.array(target, dtype=float),
            label=label,
            min_duration=min_duration,
            max_duration=max_duration,
            stop_event=stop_event,
        )

    def _execute_joint_ramp(
        self,
        current: np.ndarray,
        target: np.ndarray,
        *,
        label: str,
        min_duration: float,
        max_duration: float,
        target_hz: float = 40.0,
        stop_event: Optional[threading.Event] = None,
    ) -> Tuple[bool, float, int, float, bool]:
        robot = self._robot
        if robot is None:
            self._logger.debug('Joint ramp %s skipped: robot unavailable', label)
            return False, 0.0, 0, 0.0, False

        current = np.array(current, dtype=float)
        target = np.array(target, dtype=float)
        if current.shape != target.shape:
            size = min(current.size, target.size)
            current = current[:size]
            target = target[:size]

        delta = target - current
        max_delta = float(np.max(np.abs(delta))) if delta.size else 0.0
        if max_delta < 1e-4:
            return True, 0.0, 0, max_delta, False

        nominal_speed = max(self._joint_limit_rate, 0.2)
        computed_duration = max_delta / nominal_speed
        ramp_duration = float(min(max(computed_duration, min_duration), max_duration))
        ramp_steps = max(int(ramp_duration * target_hz), 10)
        dt = ramp_duration / ramp_steps if ramp_steps else 0.02

        preempted = False
        for step in range(1, ramp_steps + 1):
            if stop_event is not None and stop_event.is_set():
                self._logger.info('%s ramp preempted (%d/%d steps)', label, step - 1, ramp_steps)
                preempted = True
                break
            alpha = step / ramp_steps
            cmd = current + delta * alpha
            try:
                robot.command_joint_pos(cmd)
            except Exception as exc:
                self._logger.warning('%s ramp step failed (%s); aborting ramp', label, exc)
                return False, ramp_duration, step, max_delta, False
            if step < ramp_steps:
                time.sleep(dt)

        if preempted:
            # Do not overwrite cached state with full target; let subsequent
            # telemetry refresh reflect the partially executed motion.
            return False, ramp_duration, step - 1, max_delta, True

        with self._lock:
            limit = min(len(self._joint_state.positions), len(target))
            for i in range(limit):
                self._joint_state.positions[i] = float(target[i])
            for i in range(len(self._joint_state.velocities)):
                self._joint_state.velocities[i] = 0.0
            for i in range(len(self._joint_state.efforts)):
                self._joint_state.efforts[i] = 0.0

        return True, ramp_duration, ramp_steps, max_delta, False

    # ------------------------------------------------------------------ helpers
    def _cache_default_gains(self) -> None:
        if not self._robot:
            return
        try:
            info = self._robot.get_robot_info()
        except Exception as exc:
            self._logger.warning('Failed to query robot gains: %s', exc)
            return

        kp = info.get('kp')
        kd = info.get('kd')
        if kp is None or kd is None:
            self._logger.warning('Robot info missing kp/kd; zero-gravity accuracy may degrade')
            return

        self._default_kp = np.array(kp, copy=True)
        self._default_kd = np.array(kd, copy=True)
        self._logger.debug('Cached default joint gains for zero-gravity toggling')

    def _ensure_gain_snapshot(self) -> bool:
        if self._default_kp is None or self._default_kd is None:
            self._cache_default_gains()
        return self._default_kp is not None and self._default_kd is not None

    def stop_motion(self, reason: str = '') -> None:
        with self._lock:
            self._logger.warning('Stop motion requested: %s', reason)
            if self._robot:
                try:
                    if hasattr(self._robot, 'stop'):
                        self._robot.stop()
                except Exception as exc:
                    self._logger.warning('Failed to stop robot: %s', exc)

    # ------------------------------------------------------------------ telemetry
    def read_joint_state(self) -> JointStateData:
        """Read joint state, updating from hardware if available."""
        # Try to update from hardware first
        self._update_state_from_hardware()

        with self._lock:
            return JointStateData(
                names=list(self._joint_state.names),
                positions=list(self._joint_state.positions),
                velocities=list(self._joint_state.velocities),
                efforts=list(self._joint_state.efforts),
            )

    def set_joint_state(self, data: JointStateData) -> None:
        with self._lock:
            self._joint_state = data
            self._joint_index = {name: idx for idx, name in enumerate(self._joint_state.names)}

    def get_zero_gravity(self) -> bool:
        return self._zero_gravity

    def read_end_effector_pose(self) -> Pose:
        with self._lock:
            return deepcopy(self._ee_pose)

    # ------------------------------------------------------------------ helpers
    def _apply_joint_delta(self, mapping: Dict[str, float], rate_limit: float) -> None:
        timestamp = time.time()
        for name, target in list(mapping.items()):
            bounded_target = self._clamp_joint_target(name, target)
            if bounded_target != target:
                mapping[name] = bounded_target

            idx = self._joint_index.get(name)
            if idx is None:
                self._logger.warning('Unknown joint %s; ignoring', name)
                continue

            current = self._joint_state.positions[idx]
            delta = bounded_target - current
            limit = rate_limit
            if abs(delta) > limit:
                delta = limit if delta > 0 else -limit
            new_value = current + delta
            self._joint_state.positions[idx] = new_value
            self._joint_state.velocities[idx] = delta
            self._joint_state.efforts[idx] = 0.0
            self._logger.debug('Joint %s: %.3f -> %.3f (dt=%.3f)', name, current, new_value, timestamp)

    def _clamp_joint_target(self, name: str, target: float) -> float:
        if not self._robot_description:
            return target
        limit = self._robot_description.joint_limit(name)
        if not limit:
            return target

        clamped = target
        if limit.min_position is not None and clamped < limit.min_position:
            clamped = limit.min_position
        if limit.max_position is not None and clamped > limit.max_position:
            clamped = limit.max_position

        if clamped != target:
            self._logger.debug(
                'Joint %s target %.3f clamped to %.3f (limits [%s, %s])',
                name,
                target,
                clamped,
                limit.min_position,
                limit.max_position,
            )
        return clamped

    def _clamp_joint_array(self, joint_values: np.ndarray) -> np.ndarray:
        if not self._robot_description or not self._joint_index:
            return joint_values

        clamped = np.asarray(joint_values, dtype=float).copy()
        changed = False
        for name, idx in self._joint_index.items():
            if idx >= len(clamped):
                continue
            limited_value = self._clamp_joint_target(name, float(clamped[idx]))
            if limited_value != clamped[idx]:
                clamped[idx] = limited_value
                changed = True
        return clamped if changed else joint_values

    def _update_state_from_hardware(self) -> None:
        """Update internal joint state from hardware SDK if available."""
        if not self._robot:
            return

        # Rate limit reads to avoid flooding
        now = time.time()
        if now - self._last_read_time < 0.01:  # 100 Hz max
            return

        try:
            with self._lock:
                joint_pos = self._robot.get_joint_pos()
                # Update positions (assume joint_pos matches our joint order)
                for i in range(min(len(joint_pos), len(self._joint_state.positions))):
                    self._joint_state.positions[i] = float(joint_pos[i])

                # Try to get velocities if available
                if hasattr(self._robot, 'get_joint_vel'):
                    joint_vel = self._robot.get_joint_vel()
                    for i in range(min(len(joint_vel), len(self._joint_state.velocities))):
                        self._joint_state.velocities[i] = float(joint_vel[i])

                # Try to get efforts/currents if available
                if hasattr(self._robot, 'get_joint_torque'):
                    joint_torque = self._robot.get_joint_torque()
                    for i in range(min(len(joint_torque), len(self._joint_state.efforts))):
                        self._joint_state.efforts[i] = float(joint_torque[i])

                self._last_read_time = now
        except Exception as exc:
            self._logger.debug('Failed to read hardware state: %s', exc)

    def solve_ik(self, target: CartesianTarget, *, xyz_only: bool = False, max_iters: int = 500) -> Tuple[bool, Optional[np.ndarray]]:
        """Solve inverse kinematics for a Cartesian target.

        Returns:
            (success, joint_solution) where joint_solution is a numpy array of joint positions
        """
        if not self._robot or not self._kinematics:
            self._logger.warning('IK solving requires hardware SDK')
            return False, None

        try:
            with self._lock:
                current_joints = np.array(self._robot.get_joint_pos()[:6])

                # Build target pose matrix
                if xyz_only:
                    # Preserve current orientation, only change position
                    current_pose = self._kinematics.fk(current_joints, "grasp_site")
                    target_pose = np.array(current_pose, copy=True)
                    target_pose[:3, 3] = [target.x, target.y, target.z]
                else:
                    # Full pose with orientation
                    target_pose = np.eye(4)
                    target_pose[:3, 3] = [target.x, target.y, target.z]
                    target_pose[:3, :3] = _quaternion_to_matrix(target.qx, target.qy, target.qz, target.qw)

                # Solve IK
                success, joint_sol = self._kinematics.ik(
                    target_pose=target_pose,
                    site_name="grasp_site",
                    init_q=current_joints,
                    max_iters=max_iters,
                    verbose=False,
                )

                if success:
                    # Build full joint vector (including gripper)
                    full_joints = np.array(self._robot.get_joint_pos())
                    full_joints[:6] = joint_sol
                    return True, full_joints
                else:
                    return False, None

        except Exception as exc:
            self._logger.error('IK solving failed: %s', exc)
            return False, None
