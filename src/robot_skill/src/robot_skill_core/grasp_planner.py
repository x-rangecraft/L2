"""Grasp planner using Open3D OBB for optimal grasp pose calculation."""
from __future__ import annotations

import copy
from typing import List, Tuple

import numpy as np
import open3d as o3d
from geometry_msgs.msg import Point, PoseStamped
from scipy.spatial.transform import Rotation

from robot_skill_core.skill_loader import SkillStep


class GraspPlanner:
    """
    Grasp planner that computes optimal grasp poses using Open3D Oriented Bounding Box (OBB).

    The planner analyzes the object's boundary points to determine:
    1. The object's center position
    2. The object's principal axes and dimensions
    3. The best grasp direction (narrowest graspable axis)
    """

    def __init__(
        self,
        observe_step: SkillStep,
        approach_distance: float = 0.08,
        gripper_width_margin: float = 0.02,
    ) -> None:
        """
        Initialize the grasp planner.

        Args:
            observe_step: SkillStep for moving to observe position (from general_skill.yaml)
            approach_distance: Height offset for pre-grasp pose (default 8cm)
            gripper_width_margin: Extra margin added to gripper width (default 2cm)
        """
        self._observe_step = observe_step
        self._approach_distance = approach_distance
        self._gripper_width_margin = gripper_width_margin

    def compute_grasp_pose_from_boundary(
        self,
        boundary_points: List[Point],
        max_gripper_width: float,
    ) -> Tuple[PoseStamped, float]:
        """
        Compute optimal grasp pose from boundary points using OBB.

        Args:
            boundary_points: List of geometry_msgs/Point representing object boundary
            max_gripper_width: Maximum gripper opening width in meters

        Returns:
            Tuple of (grasp_pose, gripper_width)

        Raises:
            ValueError: If object is too large to grasp
        """
        # 1. Convert to numpy array
        points = np.array([[p.x, p.y, p.z] for p in boundary_points])

        if len(points) < 4:
            raise ValueError(f'Not enough boundary points: {len(points)} (need at least 4)')

        # 2. Create Open3D point cloud and compute OBB
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        obb = pcd.get_oriented_bounding_box()

        center = np.array(obb.center)  # Grasp center [x, y, z]
        R = np.array(obb.R)  # Rotation matrix (three principal axes)
        extent = np.array(obb.extent)  # Dimensions along each axis [dx, dy, dz]

        # 3. Select best grasp axis (width <= max_gripper_width, prefer narrowest)
        graspable_axes = [
            (i, extent[i]) for i in range(3)
            if extent[i] <= max_gripper_width
        ]

        if not graspable_axes:
            raise ValueError(
                f'Object too large to grasp: min dimension {min(extent):.3f}m > '
                f'max gripper width {max_gripper_width:.3f}m'
            )

        # Choose the narrowest graspable axis for most stable grasp
        best_axis_idx, object_width = min(graspable_axes, key=lambda x: x[1])
        gripper_width = min(object_width + self._gripper_width_margin, max_gripper_width)

        # 4. Build grasp pose
        grasp_pose = self._build_grasp_pose(center, R, best_axis_idx)

        return grasp_pose, gripper_width

    def _build_grasp_pose(
        self,
        center: np.ndarray,
        R: np.ndarray,
        grasp_axis: int,
    ) -> PoseStamped:
        """
        Build grasp pose from OBB data.

        The grasp pose is constructed such that:
        - Position: OBB center
        - Orientation: End-effector Z-axis pointing down, gripper closes along grasp_axis

        Args:
            center: OBB center position [x, y, z]
            R: OBB rotation matrix (3x3)
            grasp_axis: Index of the axis to use for gripper closing direction

        Returns:
            PoseStamped with computed grasp pose
        """
        pose = PoseStamped()
        pose.header.frame_id = 'base_link'
        pose.pose.position.x = float(center[0])
        pose.pose.position.y = float(center[1])
        pose.pose.position.z = float(center[2])

        # Get gripper closing direction from OBB axis
        grasp_dir = R[:, grasp_axis]

        # Build end-effector coordinate frame:
        # - Z-axis: pointing down (0, 0, -1)
        # - X-axis: gripper closing direction (projected onto horizontal plane)
        z_axis = np.array([0.0, 0.0, -1.0])  # End-effector Z-axis points down

        # Project grasp direction onto horizontal plane and normalize
        x_axis = grasp_dir.copy()
        x_axis[2] = 0  # Remove vertical component
        x_norm = np.linalg.norm(x_axis)
        if x_norm < 1e-6:
            # Grasp direction is nearly vertical, use default X direction
            x_axis = np.array([1.0, 0.0, 0.0])
        else:
            x_axis = x_axis / x_norm

        # Compute Y-axis to complete right-handed coordinate system
        y_axis = np.cross(z_axis, x_axis)
        y_norm = np.linalg.norm(y_axis)
        if y_norm < 1e-6:
            # Degenerate case, use default orientation
            y_axis = np.array([0.0, 1.0, 0.0])
        else:
            y_axis = y_axis / y_norm

        # Re-orthogonalize X-axis
        x_axis = np.cross(y_axis, z_axis)
        x_axis = x_axis / np.linalg.norm(x_axis)

        # Build rotation matrix and convert to quaternion
        rot_matrix = np.column_stack([x_axis, y_axis, z_axis])
        quat = Rotation.from_matrix(rot_matrix).as_quat()  # [x, y, z, w]

        pose.pose.orientation.x = float(quat[0])
        pose.pose.orientation.y = float(quat[1])
        pose.pose.orientation.z = float(quat[2])
        pose.pose.orientation.w = float(quat[3])

        return pose

    def compute_pre_grasp_pose(self, grasp_pose: PoseStamped) -> PoseStamped:
        """
        Compute pre-grasp pose (approach position above the grasp pose).

        Args:
            grasp_pose: The target grasp pose

        Returns:
            PoseStamped with position elevated by approach_distance
        """
        pre_grasp = copy.deepcopy(grasp_pose)
        pre_grasp.pose.position.z += self._approach_distance
        return pre_grasp

    def build_grasp_record_steps(
        self,
        boundary_points: List[Point],
        max_gripper_width: float,
    ) -> List[SkillStep]:
        """
        Build complete grasp-observe-place step sequence.

        Args:
            boundary_points: Object boundary points for grasp planning
            max_gripper_width: Maximum gripper opening width

        Returns:
            List of SkillStep for the complete operation

        Raises:
            ValueError: If object cannot be grasped
        """
        # Compute grasp pose and gripper width
        grasp_pose, gripper_width = self.compute_grasp_pose_from_boundary(
            boundary_points, max_gripper_width
        )
        pre_grasp_pose = self.compute_pre_grasp_pose(grasp_pose)

        # Build step sequence
        return [
            SkillStep(
                id='safe_start',
                type='safety_pose',
                desc='回到安全位姿',
                params={},
            ),
            SkillStep(
                id='pre_grasp',
                type='cartesian_move',
                desc='移到预抓取位',
                params={'target_pose': pre_grasp_pose, 'speed_scale': 1.0},
            ),
            SkillStep(
                id='open_gripper',
                type='gripper',
                desc='张开夹爪',
                params={'command': 0, 'width_m': gripper_width},
            ),
            SkillStep(
                id='grasp',
                type='cartesian_move',
                desc='移到抓取位',
                params={'target_pose': grasp_pose, 'speed_scale': 0.5},
            ),
            SkillStep(
                id='close_gripper',
                type='gripper',
                desc='闭合夹爪',
                params={'command': 1},
            ),
            SkillStep(
                id='observe',
                type=self._observe_step.type,
                desc=self._observe_step.desc,
                params=self._observe_step.params,
            ),
            SkillStep(
                id='sweep',
                type='joint_move',
                desc='Joint6扫描',
                params={'joint_index': 5, 'positions': [-2.14, 2.14, -2.14]},
            ),
            SkillStep(
                id='place',
                type='cartesian_move',
                desc='放回原位',
                params={'target_pose': grasp_pose, 'speed_scale': 0.5},
            ),
            SkillStep(
                id='release',
                type='gripper',
                desc='张开夹爪',
                params={'command': 0},
            ),
            SkillStep(
                id='safe_end',
                type='safety_pose',
                desc='回到安全位姿',
                params={},
            ),
        ]


__all__ = ['GraspPlanner']

