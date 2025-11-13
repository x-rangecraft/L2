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

from .can_interface_manager import ensure_ready
from .logging_utils import get_logger
from .safe_pose_loader import SafePose


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

    def __init__(self, safe_pose: SafePose, *, joint_limit_rate: float = 0.5, gripper_type: Optional[str] = None):
        self._logger = get_logger(__name__)
        self._lock = threading.RLock()
        self._safe_pose = safe_pose
        self._zero_gravity = False
        self._joint_limit_rate = joint_limit_rate
        self._can_channel = 'can0'
        self._gripper_type = gripper_type or "linear_4310"
        self._robot = None
        self._kinematics = None
        self._sdk_commander = self._maybe_import_sdk()
        self._joint_state = self._initialize_joint_state()
        self._ee_pose = Pose()
        self._last_read_time = 0.0

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
        names = list(self._safe_pose.joint_names)
        count = len(names)
        positions = list(self._safe_pose.positions)
        zeros = [0.0] * count
        return JointStateData(names=names, positions=positions, velocities=zeros[:], efforts=zeros[:])

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
    def set_zero_gravity(self, enabled: bool) -> None:
        with self._lock:
            self._zero_gravity = enabled
            if self._robot:
                try:
                    # I2RT SDK may have zero gravity control
                    if hasattr(self._robot, 'set_zero_gravity'):
                        self._robot.set_zero_gravity(enabled)
                    else:
                        self._logger.warning('Robot lacks set_zero_gravity method')
                except Exception as exc:
                    self._logger.warning('Failed to set zero_gravity: %s', exc)
            self._logger.info('Zero-gravity mode -> %s', enabled)

    def command_joint_positions(self, names: Iterable[str], values: Iterable[float], *, rate_limit: Optional[float] = None) -> None:
        mapping = dict(zip(names, values))
        with self._lock:
            self._apply_joint_delta(mapping, rate_limit or self._joint_limit_rate)
            self._logger.debug('Updated joints: %s', mapping)
            if self._robot:
                try:
                    # Build full joint array
                    current = np.array(self._robot.get_joint_pos())
                    for name, value in mapping.items():
                        try:
                            idx = self._joint_state.names.index(name)
                            if idx < len(current):
                                current[idx] = value
                        except ValueError:
                            self._logger.warning('Unknown joint %s', name)
                    self._robot.command_joint_pos(current)
                except Exception as exc:
                    self._logger.warning('SDK command_joint_pos failed: %s', exc)

    def command_cartesian_pose(self, pose: Pose, *, xyz_only: bool = False) -> None:
        """Command a Cartesian pose. This method will solve IK and command joint positions."""
        with self._lock:
            self._logger.info('Received cartesian command xyz_only=%s', xyz_only)
            self._ee_pose = deepcopy(pose)

            if not self._robot or not self._kinematics:
                self._logger.debug('No SDK/kinematics available, storing target only')
                return

            try:
                # Solve IK
                target = _pose_to_target(pose)
                success, joint_solution = self.solve_ik(target, xyz_only=xyz_only)

                if not success or joint_solution is None:
                    self._logger.warning('IK failed for target pose')
                    return

                # Command the solved joint positions
                self._robot.command_joint_pos(joint_solution)
                self._logger.debug('Commanded cartesian pose via IK')
            except Exception as exc:
                self._logger.error('Failed to command cartesian pose: %s', exc)

    def move_to_safe_pose(self) -> bool:
        """Move to safe pose and return True if successful."""
        self.command_joint_positions(self._safe_pose.joint_names, self._safe_pose.positions, rate_limit=self._joint_limit_rate)
        self._logger.info('Moved to SAFE_POSE')
        # TODO: Add position verification
        return True

    def check_at_safe_pose(self, tolerance: float = 0.05) -> bool:
        """Check if robot is at safe pose within tolerance (radians)."""
        with self._lock:
            for i, name in enumerate(self._safe_pose.joint_names):
                try:
                    idx = self._joint_state.names.index(name)
                    current = self._joint_state.positions[idx]
                    target = self._safe_pose.positions[i]
                    if abs(current - target) > tolerance:
                        return False
                except (ValueError, IndexError):
                    return False
            return True

    def stop_motion(self, reason: str = '') -> None:
        with self._lock:
            self._logger.warning('Stop motion requested: %s', reason)
            if self._robot:
                try:
                    if hasattr(self._robot, 'stop'):
                        self._robot.stop()
                except Exception as exc:
                    self._logger.warning('Failed to stop robot: %s', exc)

    def reset_can(self) -> None:
        ensure_ready(self._can_channel)
        if self._robot:
            try:
                if hasattr(self._robot, 'reset_bus'):
                    self._robot.reset_bus()
            except Exception as exc:
                self._logger.debug('Failed to reset CAN bus: %s', exc)

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

    def get_zero_gravity(self) -> bool:
        return self._zero_gravity

    def read_end_effector_pose(self) -> Pose:
        with self._lock:
            return deepcopy(self._ee_pose)

    # ------------------------------------------------------------------ helpers
    def _apply_joint_delta(self, mapping: Dict[str, float], rate_limit: float) -> None:
        idx = {name: i for i, name in enumerate(self._joint_state.names)}
        timestamp = time.time()
        for name, target in mapping.items():
            if name not in idx:
                self._logger.warning('Unknown joint %s; ignoring', name)
                continue
            i = idx[name]
            current = self._joint_state.positions[i]
            delta = target - current
            limit = rate_limit
            if abs(delta) > limit:
                delta = limit if delta > 0 else -limit
            new_value = current + delta
            self._joint_state.positions[i] = new_value
            self._joint_state.velocities[i] = delta
            self._joint_state.efforts[i] = 0.0
            self._logger.debug('Joint %s: %.3f -> %.3f (dt=%.3f)', name, current, new_value, timestamp)

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
                    verbose=False
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
