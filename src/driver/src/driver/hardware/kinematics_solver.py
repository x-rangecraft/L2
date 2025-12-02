"""Unified IK/FK solver wrapper for robot_driver."""
from __future__ import annotations

from typing import Optional, Sequence, Tuple
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState

from driver.hardware.hardware_commander import CartesianTarget, HardwareCommander, JointStateData
from driver.utils.logging_utils import get_logger


class KinematicsSolver:
    """Wrapper around HardwareCommander kinematics helpers."""

    def __init__(self, commander: HardwareCommander, base_frame: str) -> None:
        self._commander = commander
        self._base_frame = base_frame
        self._logger = get_logger('kinematics_solver')
        self._joint_names = commander.get_joint_names()
        self._joint_index = {name: idx for idx, name in enumerate(self._joint_names)}

    # ------------------------------------------------------------------ public API
    def solve_ik(
        self,
        pose: Pose,
        *,
        xyz_only: bool = False,
        max_iters: int = 500,
        seed_joints: Optional[JointState] = None,
    ) -> Tuple[bool, Optional[JointState], str]:
        """Solve IK for a target pose. Returns (success, JointState, error).
        
        Args:
            pose: Target end-effector pose
            xyz_only: If True, only constrain position, preserve current orientation
            max_iters: Maximum iterations (0 means use default 500)
            seed_joints: Optional initial joint state (None means use current hardware state)
        """
        target = CartesianTarget(
            x=float(pose.position.x),
            y=float(pose.position.y),
            z=float(pose.position.z),
            qx=float(pose.orientation.x),
            qy=float(pose.orientation.y),
            qz=float(pose.orientation.z),
            qw=float(pose.orientation.w),
        )
        
        # Convert seed_joints JointState to numpy array if provided
        seed_array = None
        if seed_joints is not None and seed_joints.name and seed_joints.position:
            seed_array = self._positions_from_data(seed_joints.name, seed_joints.position)
            if seed_array is None:
                self._logger.warning('Failed to convert seed_joints, using current hardware state')
        
        # Use default max_iters if 0 is provided
        effective_max_iters = max_iters if max_iters > 0 else 500
        
        # Convert seed_array to numpy if available
        seed_np = np.array(seed_array) if seed_array is not None else None
        
        success, solution = self._commander.solve_ik(
            target,
            xyz_only=xyz_only,
            max_iters=effective_max_iters,
            seed_joints=seed_np,
        )
        if not success or solution is None:
            return False, None, 'IK solver unavailable or failed'

        joint_state = JointState()
        names = self._refresh_joint_names()
        count = min(len(names), solution.size)
        if count == 0:
            return False, None, 'Joint list empty'
        joint_state.name = names[:count]
        joint_state.position = [float(solution[i]) for i in range(count)]
        return True, joint_state, ''

    def compute_fk_pose(
        self,
        *,
        stamp=None,
        joint_state: Optional[JointStateData] = None,
        joint_msg: Optional[JointState] = None,
    ) -> Tuple[PoseStamped, bool]:
        """Compute FK pose; returns (PoseStamped, solver_success)."""
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = self._base_frame
        if stamp is not None:
            pose_msg.header.stamp = stamp

        positions = None
        if joint_state is not None:
            positions = self._positions_from_data(joint_state.names, joint_state.positions)
        elif joint_msg is not None:
            positions = self._positions_from_data(joint_msg.name, joint_msg.position)

        if positions is None:
            joint_state = self._commander.read_joint_state()
            positions = self._positions_from_data(joint_state.names, joint_state.positions)

        pose = None
        if positions is not None:
            pose = self._commander.compute_fk_pose(positions)

        if pose is None:
            pose_msg.pose = self._commander.read_end_effector_pose()
            return pose_msg, False

        pose_msg.pose = pose
        return pose_msg, True

    # ------------------------------------------------------------------ helpers
    def _positions_from_data(self, names: Sequence[str], values: Sequence[float]) -> Optional[Sequence[float]]:
        if not names or not values:
            return None
        names_list = self._refresh_joint_names()
        index = self._joint_index
        buffer = [0.0] * len(names_list)
        found = 0
        for name, value in zip(names, values):
            idx = index.get(name)
            if idx is None:
                continue
            buffer[idx] = float(value)
            found += 1
        return buffer if found else None

    def _refresh_joint_names(self) -> Sequence[str]:
        current = self._commander.get_joint_names()
        if len(current) != len(self._joint_names):
            self._joint_names = current
            self._joint_index = {name: idx for idx, name in enumerate(self._joint_names)}
        return self._joint_names
