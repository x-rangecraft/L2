"""Simple grasp executor based on geometric center and bbox (no CGN)."""
from __future__ import annotations

import copy
import math
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from rclpy.node import Node as RclpyNode
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from robot_driver.srv import SolveIK

from robot_skill_core.constants import (
    DEFAULT_APPROACH_DISTANCE,
    DEFAULT_GRIPPER_WIDTH_MARGIN,
    DEFAULT_MAX_GRIPPER_WIDTH,
    DEFAULT_MIN_GRASP_HEIGHT,
    JOINT_NAME_ORDER,
)
from robot_skill_core.skill_loader import SkillStep
from robot_skill_core.grasp_executor import GraspExecutor


ROBOT_BASE_FRAME = "base_link"

IK_SERVICE = "/robot_driver/service/solve_ik"
JOINT_STATES_TOPIC = "/joint_states"
END_EFFECTOR_POSE_TOPIC = "/robot_driver/end_effector_pose"

# Prepare joint positions (step2) - shared constant
PREPARE_JOINT_POSITIONS = [
    -0.026131074998088977,   # joint1
    0.7471198596169977,      # joint2
    1.2533379110399032,      # joint3
    -0.5460822461280248,     # joint4
    -0.0005722133211296665,  # joint5
    0.03452353704127553,     # joint6
]


class SimpleGraspExecutor:
    """Geometric grasp planner using object center & bbox in base_link frame.

    This class is a lightweight alternative to Contact-GraspNet based planning.
    It reuses :class:`GraspExecutor` for step sequence generation.
    """

    def __init__(
        self,
        node: RclpyNode,
        observe_step: SkillStep,
        approach_distance: float = DEFAULT_APPROACH_DISTANCE,
        gripper_width_margin: float = DEFAULT_GRIPPER_WIDTH_MARGIN,
        max_gripper_width: float = DEFAULT_MAX_GRIPPER_WIDTH,
    ) -> None:
        self._node = node
        self._approach_distance = approach_distance
        self._gripper_width_margin = gripper_width_margin
        self._max_gripper_width = max_gripper_width

        # IK client
        self._ik_client = node.create_client(SolveIK, IK_SERVICE)
        
        # Subscribe to joint_states and end_effector_pose topics
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        self._current_joint_state: Optional[JointState] = None
        self._current_end_effector_pose: Optional[PoseStamped] = None
        
        self._joint_state_sub = node.create_subscription(
            JointState,
            JOINT_STATES_TOPIC,
            self._joint_state_callback,
            qos_profile
        )
        
        self._end_effector_pose_sub = node.create_subscription(
            PoseStamped,
            END_EFFECTOR_POSE_TOPIC,
            self._end_effector_pose_callback,
            qos_profile
        )

        # Reuse GraspExecutor for step building
        self._delegate = GraspExecutor(
            node=node,
            observe_step=observe_step,
            approach_distance=approach_distance,
            gripper_width_margin=gripper_width_margin,
            max_gripper_width=max_gripper_width,
        )

    # ------------------------------------------------------------------ #
    # Core API
    # ------------------------------------------------------------------ #
    async def compute_grasp_plan(
        self,
        center_base: Point,
    ) -> Tuple[Optional[PoseStamped], Optional[PoseStamped], float]:
        """Compute pre-grasp & grasp poses and gripper width.

        Args:
            center_base: object center in base_link frame.
        """
        self._node.get_logger().info(
            f"[SimpleGraspExecutor] Starting grasp plan computation for center: "
            f"({center_base.x:.3f}, {center_base.y:.3f}, {center_base.z:.3f})"
        )
        
        # Step 1: Get current end-effector pose via FK
        self._node.get_logger().info("[SimpleGraspExecutor] Step 1: Getting current end-effector pose (FK)...")
        current_pose = await self._get_current_end_effector_pose()
        if current_pose is None:
            self._node.get_logger().error(
                "[SimpleGraspExecutor] ❌ FAILED: Failed to get current end-effector pose via FK service. "
                "Check if FK service is available and robot is connected."
            )
            return None, None, 0.0
        
        self._node.get_logger().info(
            f"[SimpleGraspExecutor] ✓ Current end-effector pose: "
            f"pos=({current_pose.position.x:.3f}, {current_pose.position.y:.3f}, {current_pose.position.z:.3f})"
        )

        # Step 2: Build grasp pose
        self._node.get_logger().info("[SimpleGraspExecutor] Step 2: Building grasp pose...")
        grasp_pose = self._build_grasp_pose(center_base, current_pose)
        self._node.get_logger().info(
            f"[SimpleGraspExecutor] ✓ Grasp pose: "
            f"pos=({grasp_pose.position.x:.3f}, {grasp_pose.position.y:.3f}, {grasp_pose.position.z:.3f}), "
            f"quat=({grasp_pose.orientation.x:.4f}, {grasp_pose.orientation.y:.4f}, "
            f"{grasp_pose.orientation.z:.4f}, {grasp_pose.orientation.w:.4f})"
        )
        
        # Step 3: Compute pre-grasp pose based on step3 end-effector position
        # Pre-grasp position is 0.08m away from center_base along the line from center_base to step3 end-effector
        self._node.get_logger().info("[SimpleGraspExecutor] Step 3: Computing pre-grasp pose from step3 position...")
        pre_pose_stamped = self._compute_pre_grasp_pose_from_step3(center_base)
        if pre_pose_stamped is None:
            self._node.get_logger().error(
                "[SimpleGraspExecutor] ❌ FAILED: Failed to compute pre-grasp pose from step3 position"
            )
            return None, None, 0.0
        
        self._node.get_logger().info(
            f"[SimpleGraspExecutor] ✓ Pre-grasp pose: "
            f"pos=({pre_pose_stamped.pose.position.x:.3f}, {pre_pose_stamped.pose.position.y:.3f}, "
            f"{pre_pose_stamped.pose.position.z:.3f})"
        )

        # Step 4: Verify pre-grasp IK
        self._node.get_logger().info("[SimpleGraspExecutor] Step 4: Verifying pre-grasp IK...")
        pre_grasp_ik_success = await self._verify_ik(pre_pose_stamped.pose)
        if not pre_grasp_ik_success:
            self._node.get_logger().error(
                f"[SimpleGraspExecutor] ❌ FAILED: Pre-grasp IK verification failed. "
                f"Pose: pos=({pre_pose_stamped.pose.position.x:.3f}, "
                f"{pre_pose_stamped.pose.position.y:.3f}, {pre_pose_stamped.pose.position.z:.3f}), "
                f"quat=({pre_pose_stamped.pose.orientation.x:.4f}, "
                f"{pre_pose_stamped.pose.orientation.y:.4f}, {pre_pose_stamped.pose.orientation.z:.4f}, "
                f"{pre_pose_stamped.pose.orientation.w:.4f})"
            )
            return None, None, 0.0
        self._node.get_logger().info("[SimpleGraspExecutor] ✓ Pre-grasp IK verification passed")

        # Step 5: Verify grasp IK
        self._node.get_logger().info("[SimpleGraspExecutor] Step 5: Verifying grasp IK...")
        grasp_ik_success = await self._verify_ik(grasp_pose)
        if not grasp_ik_success:
            self._node.get_logger().error(
                f"[SimpleGraspExecutor] ❌ FAILED: Grasp IK verification failed. "
                f"Pose: pos=({grasp_pose.position.x:.3f}, {grasp_pose.position.y:.3f}, "
                f"{grasp_pose.position.z:.3f}), quat=({grasp_pose.orientation.x:.4f}, "
                f"{grasp_pose.orientation.y:.4f}, {grasp_pose.orientation.z:.4f}, "
                f"{grasp_pose.orientation.w:.4f})"
            )
            return None, None, 0.0
        self._node.get_logger().info("[SimpleGraspExecutor] ✓ Grasp IK verification passed")

        # Step 6: Compute gripper width
        gripper_width = min(0.05 + self._gripper_width_margin, self._max_gripper_width)
        self._node.get_logger().info(
            f"[SimpleGraspExecutor] ✓ Gripper width: {gripper_width:.3f}m"
        )

        grasp_stamped = PoseStamped()
        grasp_stamped.header.frame_id = ROBOT_BASE_FRAME
        grasp_stamped.header.stamp = self._node.get_clock().now().to_msg()
        grasp_stamped.pose = grasp_pose

        self._node.get_logger().info("[SimpleGraspExecutor] ✓ Grasp plan computation completed successfully")
        return grasp_stamped, pre_pose_stamped, gripper_width

    def build_grasp_steps(
        self,
        grasp_pose: PoseStamped,
        pre_grasp_pose: PoseStamped,
        gripper_width: float,
        center_base: Optional[Point] = None,
    ) -> list[SkillStep]:
        """Build step sequence for grasp_record_easy with additional preparation steps.
        
        Args:
            grasp_pose: Target grasp pose
            pre_grasp_pose: Pre-grasp pose
            gripper_width: Gripper opening width
            center_base: Object center in base_link frame (for computing joint angles)
        """
        steps = []
        
        # Step 1: safe_start - 回到安全位姿
        steps.append(SkillStep(
            id='safe_start',
            type='safety_pose',
            desc='回到安全位姿',
            params={},
        ))
        
        # Step 2: move_to_prepare - 移动到准备夹取位置（使用关节角度）
        steps.append(SkillStep(
            id='move_to_prepare',
            type='joint_move',
            desc='移动到准备夹取位置',
            params={
                'all_positions': PREPARE_JOINT_POSITIONS.copy(),
                'speed_scale': 1.0,
            },
        ))
        
        # Step 3: adjust_joints_toward_target - 调整Joint1和Joint4朝向质心
        # Compute step3 joint angles (reuse the computation from _compute_pre_grasp_pose_from_step3)
        step3_joint_angles = None
        if center_base is not None:
            step3_joint_angles = self._compute_step3_joint_angles(center_base)
            
            if step3_joint_angles is not None and len(step3_joint_angles) >= 6:
                steps.append(SkillStep(
                    id='adjust_joints_toward_target',
                    type='joint_move',
                    desc='调整Joint1和Joint4朝向质心',
                    params={
                        'all_positions': step3_joint_angles.copy(),
                        'speed_scale': 1.0,
                    },
                ))
            else:
                self._node.get_logger().warning(
                    "[SimpleGraspExecutor] Failed to compute step3 joint angles, "
                    "skipping joint adjustment step. Pre-grasp pose may be incorrect."
                )
        else:
            self._node.get_logger().warning(
                "[SimpleGraspExecutor] center_base not provided, skipping joint adjustment"
            )
        
        # Step 4-13: Delegate remaining steps to GraspExecutor
        remaining_steps = self._delegate.build_grasp_steps(
            grasp_pose=grasp_pose,
            pre_grasp_pose=pre_grasp_pose,
            gripper_width=gripper_width,
        )
        
        # Skip the first step (safe_start) from remaining_steps since we already added it
        steps.extend(remaining_steps[1:])
        
        return steps

    # ------------------------------------------------------------------ #
    # Topic callbacks
    # ------------------------------------------------------------------ #
    def _joint_state_callback(self, msg: JointState) -> None:
        """Callback for /joint_states topic."""
        self._current_joint_state = msg
    
    def _end_effector_pose_callback(self, msg: PoseStamped) -> None:
        """Callback for /robot_driver/end_effector_pose topic."""
        self._current_end_effector_pose = msg
    
    def _get_latest_joint_state(self, timeout_sec: float = 2.0) -> Optional[JointState]:
        """Get latest joint state from topic."""
        if self._current_joint_state is not None:
            return self._current_joint_state
        
        # Wait for first message
        import time
        start_time = time.time()
        while self._current_joint_state is None and (time.time() - start_time) < timeout_sec:
            rclpy.spin_once(self._node, timeout_sec=0.1)
        
        if self._current_joint_state is None:
            self._node.get_logger().error(
                f"❌ No joint state received from {JOINT_STATES_TOPIC} within {timeout_sec}s"
            )
            return None
        
        return self._current_joint_state
    
    def _get_latest_end_effector_pose(self, timeout_sec: float = 2.0) -> Optional[Pose]:
        """Get latest end-effector pose from topic."""
        if self._current_end_effector_pose is not None:
            return self._current_end_effector_pose.pose
        
        # Wait for first message
        import time
        start_time = time.time()
        while self._current_end_effector_pose is None and (time.time() - start_time) < timeout_sec:
            rclpy.spin_once(self._node, timeout_sec=0.1)
        
        if self._current_end_effector_pose is None:
            self._node.get_logger().error(
                f"❌ No end-effector pose received from {END_EFFECTOR_POSE_TOPIC} within {timeout_sec}s"
            )
            return None
        
        return self._current_end_effector_pose.pose
    
    def _get_joint_position_by_name(self, joint_name: str, joint_state: JointState) -> Optional[float]:
        """Get joint position by name from JointState."""
        try:
            index = joint_state.name.index(joint_name)
            return joint_state.position[index]
        except (ValueError, IndexError):
            return None
    
    # ------------------------------------------------------------------ #
    # Internal helpers
    # ------------------------------------------------------------------ #
    async def _get_current_end_effector_pose(self) -> Optional[Pose]:
        """Get current end-effector pose from /robot_driver/end_effector_pose topic."""
        pose = self._get_latest_end_effector_pose(timeout_sec=2.0)
        if pose is not None:
            self._node.get_logger().info(
                f"Current end-effector pose from topic: pos=({pose.position.x:.3f}, "
                f"{pose.position.y:.3f}, {pose.position.z:.3f})"
            )
        return pose

    def _build_grasp_pose(self, center_base: Point, current_pose: Pose, use_current_orientation: bool = False) -> Pose:
        """Build grasp pose with orientation pointing from base origin to target.
        
        The gripper Z-axis (approach direction) will point from base_link origin
        toward the target object center.
        
        If the target z is too low (below DEFAULT_MIN_GRASP_HEIGHT), it will be
        adjusted to the minimum height to avoid workspace limits.
        
        Args:
            center_base: Target object center in base_link frame
            current_pose: Current end-effector pose (used for orientation if use_current_orientation=True)
            use_current_orientation: If True, use current_pose's orientation instead of computing new one
        """
        grasp_pose = Pose()
        # Copy position fields individually to ensure type safety
        grasp_pose.position.x = center_base.x
        grasp_pose.position.y = center_base.y
        # Ensure minimum height to avoid workspace limits
        grasp_pose.position.z = max(center_base.z, DEFAULT_MIN_GRASP_HEIGHT)
        
        if center_base.z < DEFAULT_MIN_GRASP_HEIGHT:
            self._node.get_logger().warning(
                f"[SimpleGraspExecutor] Target z ({center_base.z:.3f}m) is too low, "
                f"adjusting to minimum height {DEFAULT_MIN_GRASP_HEIGHT:.3f}m"
            )
        
        # If use_current_orientation is True, use the orientation from current_pose (which is from step3)
        if use_current_orientation:
            self._node.get_logger().info(
                "[SimpleGraspExecutor] Using current pose orientation (from step3) for grasp pose"
            )
            grasp_pose.orientation.x = current_pose.orientation.x
            grasp_pose.orientation.y = current_pose.orientation.y
            grasp_pose.orientation.z = current_pose.orientation.z
            grasp_pose.orientation.w = current_pose.orientation.w
            return grasp_pose
        
        # Compute direction vector from base origin to target
        dx = center_base.x
        dy = center_base.y
        dz = center_base.z
        
        # Normalize direction vector
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)
        if dist < 1e-6:
            # If target is at origin, use default orientation
            self._node.get_logger().warning(
                "[SimpleGraspExecutor] Target too close to base origin, using default orientation"
            )
            grasp_pose.orientation.x = 0.0
            grasp_pose.orientation.y = 0.7071067811865476  # sin(45°)
            grasp_pose.orientation.z = 0.0
            grasp_pose.orientation.w = 0.7071067811865476  # cos(45°)
            return grasp_pose
        
        # Normalize direction vector
        dx /= dist
        dy /= dist
        dz /= dist
        
        # Compute orientation: Z-axis points toward target (from base origin)
        # We need to construct a rotation matrix where:
        # - Z-axis (approach direction) points toward target: (dx, dy, dz)
        # - X-axis and Y-axis form a right-handed coordinate system
        
        # Method: Use look-at style orientation
        # Z-axis = direction to target (normalized)
        # X-axis = perpendicular to Z-axis and world Z-axis
        # Y-axis = Z-axis × X-axis
        
        # Compute X-axis: perpendicular to both Z-axis and world Z-axis
        # If Z-axis is nearly parallel to world Z-axis, use world Y-axis as reference
        if abs(dz) > 0.99:
            # Nearly vertical, use world Y-axis as reference
            x_axis = [0.0, 1.0, 0.0] if dz > 0 else [0.0, -1.0, 0.0]
        else:
            # X-axis is perpendicular to Z-axis and world Z-axis
            world_z = [0.0, 0.0, 1.0]
            # Cross product: world_z × z_axis
            x_axis = [
                world_z[1] * dz - world_z[2] * dy,
                world_z[2] * dx - world_z[0] * dz,
                world_z[0] * dy - world_z[1] * dx
            ]
            # Normalize X-axis
            x_norm = math.sqrt(x_axis[0]**2 + x_axis[1]**2 + x_axis[2]**2)
            if x_norm > 1e-6:
                x_axis = [x_axis[0]/x_norm, x_axis[1]/x_norm, x_axis[2]/x_norm]
            else:
                # Fallback: use world Y-axis
                x_axis = [0.0, 1.0, 0.0]
        
        # Y-axis = Z-axis × X-axis (right-handed coordinate system)
        y_axis = [
            dz * x_axis[1] - dy * x_axis[2],
            dx * x_axis[2] - dz * x_axis[0],
            dy * x_axis[0] - dx * x_axis[1]
        ]
        # Normalize Y-axis
        y_norm = math.sqrt(y_axis[0]**2 + y_axis[1]**2 + y_axis[2]**2)
        if y_norm > 1e-6:
            y_axis = [y_axis[0]/y_norm, y_axis[1]/y_norm, y_axis[2]/y_norm]
        else:
            # Fallback: recompute Y-axis as X-axis × Z-axis
            y_axis = [
                x_axis[1] * dz - x_axis[2] * dy,
                x_axis[2] * dx - x_axis[0] * dz,
                x_axis[0] * dy - x_axis[1] * dx
            ]
            y_norm = math.sqrt(y_axis[0]**2 + y_axis[1]**2 + y_axis[2]**2)
            if y_norm > 1e-6:
                y_axis = [y_axis[0]/y_norm, y_axis[1]/y_norm, y_axis[2]/y_norm]
            else:
                # Last resort: use world Y-axis
                y_axis = [0.0, 1.0, 0.0]
        
        # Build rotation matrix from axes
        # Rotation matrix columns are X, Y, Z axes
        R = [
            [x_axis[0], y_axis[0], dx],
            [x_axis[1], y_axis[1], dy],
            [x_axis[2], y_axis[2], dz]
        ]
        
        # Convert rotation matrix to quaternion
        trace = R[0][0] + R[1][1] + R[2][2]
        if trace > 0.0:
            s = math.sqrt(trace + 1.0) * 2.0
            qw = 0.25 * s
            qx = (R[2][1] - R[1][2]) / s
            qy = (R[0][2] - R[2][0]) / s
            qz = (R[1][0] - R[0][1]) / s
        else:
            if R[0][0] > R[1][1] and R[0][0] > R[2][2]:
                s = math.sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]) * 2.0
                qx = 0.25 * s
                qy = (R[0][1] + R[1][0]) / s
                qz = (R[0][2] + R[2][0]) / s
                qw = (R[2][1] - R[1][2]) / s
            elif R[1][1] > R[2][2]:
                s = math.sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]) * 2.0
                qx = (R[0][1] + R[1][0]) / s
                qy = 0.25 * s
                qz = (R[1][2] + R[2][1]) / s
                qw = (R[0][2] - R[2][0]) / s
            else:
                s = math.sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]) * 2.0
                qx = (R[0][2] + R[2][0]) / s
                qy = (R[1][2] + R[2][1]) / s
                qz = 0.25 * s
                qw = (R[1][0] - R[0][1]) / s
        
        # Normalize quaternion to ensure it's unit quaternion
        q_norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
        if q_norm > 1e-6:
            qx /= q_norm
            qy /= q_norm
            qz /= q_norm
            qw /= q_norm
        else:
            # Fallback: identity quaternion
            self._node.get_logger().warning(
                "[SimpleGraspExecutor] Quaternion norm too small, using identity"
            )
            qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
        
        grasp_pose.orientation.x = float(qx)
        grasp_pose.orientation.y = float(qy)
        grasp_pose.orientation.z = float(qz)
        grasp_pose.orientation.w = float(qw)
        
        self._node.get_logger().info(
            f"[SimpleGraspExecutor] Built grasp pose: "
            f"pos=({grasp_pose.position.x:.3f}, {grasp_pose.position.y:.3f}, {grasp_pose.position.z:.3f}), "
            f"direction=({dx:.3f}, {dy:.3f}, {dz:.3f}), "
            f"quat=({grasp_pose.orientation.x:.4f}, {grasp_pose.orientation.y:.4f}, "
            f"{grasp_pose.orientation.z:.4f}, {grasp_pose.orientation.w:.4f})"
        )
        
        return grasp_pose

    def _compute_orientation_toward_target(
        self, from_position: Point, to_position: Point
    ) -> Quaternion:
        """Compute orientation pointing from from_position to to_position.
        
        Returns:
            Quaternion message object
        """
        # Compute direction vector
        dx = to_position.x - from_position.x
        dy = to_position.y - from_position.y
        dz = to_position.z - from_position.z
        
        # Normalize
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)
        if dist < 1e-6:
            # Use default orientation if positions are too close
            quat = Quaternion()
            quat.x = 0.0
            quat.y = 0.7071067811865476
            quat.z = 0.0
            quat.w = 0.7071067811865476
            return quat
        
        dx /= dist
        dy /= dist
        dz /= dist
        
        # Build rotation matrix (same logic as _build_grasp_pose)
        if abs(dz) > 0.99:
            x_axis = [0.0, 1.0, 0.0] if dz > 0 else [0.0, -1.0, 0.0]
        else:
            world_z = [0.0, 0.0, 1.0]
            x_axis = [
                world_z[1] * dz - world_z[2] * dy,
                world_z[2] * dx - world_z[0] * dz,
                world_z[0] * dy - world_z[1] * dx
            ]
            x_norm = math.sqrt(x_axis[0]**2 + x_axis[1]**2 + x_axis[2]**2)
            if x_norm > 1e-6:
                x_axis = [x_axis[0]/x_norm, x_axis[1]/x_norm, x_axis[2]/x_norm]
            else:
                x_axis = [0.0, 1.0, 0.0]
        
        y_axis = [
            dz * x_axis[1] - dy * x_axis[2],
            dx * x_axis[2] - dz * x_axis[0],
            dy * x_axis[0] - dx * x_axis[1]
        ]
        
        R = [
            [x_axis[0], y_axis[0], dx],
            [x_axis[1], y_axis[1], dy],
            [x_axis[2], y_axis[2], dz]
        ]
        
        # Convert to quaternion
        trace = R[0][0] + R[1][1] + R[2][2]
        if trace > 0.0:
            s = math.sqrt(trace + 1.0) * 2.0
            qw = 0.25 * s
            qx = (R[2][1] - R[1][2]) / s
            qy = (R[0][2] - R[2][0]) / s
            qz = (R[1][0] - R[0][1]) / s
        else:
            if R[0][0] > R[1][1] and R[0][0] > R[2][2]:
                s = math.sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]) * 2.0
                qx = 0.25 * s
                qy = (R[0][1] + R[1][0]) / s
                qz = (R[0][2] + R[2][0]) / s
                qw = (R[2][1] - R[1][2]) / s
            elif R[1][1] > R[2][2]:
                s = math.sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]) * 2.0
                qx = (R[0][1] + R[1][0]) / s
                qy = 0.25 * s
                qz = (R[1][2] + R[2][1]) / s
                qw = (R[0][2] - R[2][0]) / s
            else:
                s = math.sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]) * 2.0
                qx = (R[0][2] + R[2][0]) / s
                qy = (R[1][2] + R[2][1]) / s
                qz = 0.25 * s
                qw = (R[1][0] - R[0][1]) / s
        
        # Normalize quaternion to ensure it's a valid unit quaternion
        quat_norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
        if quat_norm < 1e-6:
            # Fallback to default orientation if quaternion is invalid
            self._node.get_logger().warning(
                "[SimpleGraspExecutor] Computed quaternion norm too small, using default orientation"
            )
            quat = Quaternion()
            quat.x = 0.0
            quat.y = 0.7071067811865476
            quat.z = 0.0
            quat.w = 0.7071067811865476
            return quat
        
        quat = Quaternion()
        quat.x = float(qx / quat_norm)
        quat.y = float(qy / quat_norm)
        quat.z = float(qz / quat_norm)
        quat.w = float(qw / quat_norm)
        return quat
    
    def _compute_step3_joint_angles(self, center_base: Point) -> Optional[list[float]]:
        """Compute step3 joint angles that orient Joint1 and Joint4 toward target center.
        
        Joint1: Rotate in xy plane to point toward center (based on current Joint1 angle)
        Joint4: Rotate in xz plane to point toward center (based on current Joint4 angle)
        Other joints: Keep prepare positions unchanged.
        
        This is a shared method used by both _compute_pre_grasp_pose_from_step3 and build_grasp_steps
        to ensure consistency.
        
        Args:
            center_base: Object center in base_link frame
            
        Returns:
            List of 6 joint angles for step3, or None if computation fails
        """
        self._node.get_logger().info(
            f"[SimpleGraspExecutor] Computing step3 joint angles for center: "
            f"({center_base.x:.3f}, {center_base.y:.3f}, {center_base.z:.3f})"
        )
        
        # Get current prepare joint positions
        current_joint1 = PREPARE_JOINT_POSITIONS[0]
        current_joint4 = PREPARE_JOINT_POSITIONS[3]
        
        # Get current end-effector position (after step2 execution) from topic
        # This is needed for Joint4 calculation
        self._node.get_logger().info(
            "[SimpleGraspExecutor] Step3: Getting current end-effector pose from topic (after step2)..."
        )
        prepare_pose_msg = self._get_latest_end_effector_pose(timeout_sec=2.0)
        if prepare_pose_msg is None:
            self._node.get_logger().error(
                "[SimpleGraspExecutor] ❌ FAILED: Failed to get current end-effector pose from topic. "
                "Check if /robot_driver/end_effector_pose topic is available."
            )
            return None
        
        # Create a Point from the pose position for easier use
        from geometry_msgs.msg import Point
        prepare_position = Point()
        prepare_position.x = prepare_pose_msg.position.x
        prepare_position.y = prepare_pose_msg.position.y
        prepare_position.z = prepare_pose_msg.position.z
        
        self._node.get_logger().info(
            f"[SimpleGraspExecutor] Step3: ✓ Current end-effector pose from topic: "
            f"pos=({prepare_position.x:.3f}, {prepare_position.y:.3f}, {prepare_position.z:.3f})"
        )
        
        # Compute Joint1 angle: rotate in xy plane to point toward center
        # Joint1 controls rotation around z-axis (xy plane)
        # Target angle: atan2(center_base.y, center_base.x) - angle from base origin to center in xy plane
        target_joint1 = math.atan2(center_base.y, center_base.x)
        self._node.get_logger().info(
            f"[SimpleGraspExecutor] Step3: Joint1 - current={current_joint1:.4f}, "
            f"target={target_joint1:.4f} (xy plane angle to center)"
        )
        
        # Compute Joint4 angle: rotate in xz plane to point toward center
        # Joint4 controls rotation in xz plane
        # Direction from end-effector to center
        dx = center_base.x - prepare_position.x
        dz = center_base.z - prepare_position.z
        # Target angle in xz plane: atan2(dz, dx)
        target_joint4 = math.atan2(dz, dx)
        self._node.get_logger().info(
            f"[SimpleGraspExecutor] Step3: Joint4 - current={current_joint4:.4f}, "
            f"target={target_joint4:.4f} (xz plane angle from end-effector to center), "
            f"direction=({dx:.3f}, {dz:.3f})"
        )
        
        # Build step3 joint angles: update Joint1 and Joint4, keep others unchanged
        step3_joint_angles = list(PREPARE_JOINT_POSITIONS)
        step3_joint_angles[0] = target_joint1  # Joint1
        step3_joint_angles[3] = target_joint4  # Joint4
        
        self._node.get_logger().info(
            f"[SimpleGraspExecutor] ✓ Step3 joint angles computed: {step3_joint_angles}"
        )
        
        return step3_joint_angles
    
    def _compute_pre_grasp_pose_from_step3(self, center_base: Point) -> Optional[PoseStamped]:
        """Compute pre-grasp pose: 0.08m away from center_base along the line from center_base to step3 end-effector.
        
        Step3 is the joint adjustment step that orients Joint1 and Joint4 toward the target center.
        We compute the end-effector position after step3, then find the point 0.08m away from center
        along the line from center_base to step3 end-effector position.
        
        Args:
            center_base: Object center in base_link frame
            
        Returns:
            PoseStamped for pre-grasp position, or None if computation fails
        """
        self._node.get_logger().info(
            f"[SimpleGraspExecutor] _compute_pre_grasp_pose_from_step3: Starting computation for center: "
            f"({center_base.x:.3f}, {center_base.y:.3f}, {center_base.z:.3f})"
        )
        
        # Get current end-effector position (after step3 execution) from topic
        self._node.get_logger().info(
            "[SimpleGraspExecutor] _compute_pre_grasp_pose_from_step3: Getting current end-effector pose from topic (after step3)..."
        )
        step3_pose_msg = self._get_latest_end_effector_pose(timeout_sec=2.0)
        if step3_pose_msg is None:
            self._node.get_logger().error(
                "[SimpleGraspExecutor] ❌ FAILED: Failed to get current end-effector pose from topic after step3. "
                "Check if /robot_driver/end_effector_pose topic is available."
            )
            return None
        
        # Create a Point from the pose position for easier use
        from geometry_msgs.msg import Point
        step3_position = Point()
        step3_position.x = step3_pose_msg.position.x
        step3_position.y = step3_pose_msg.position.y
        step3_position.z = step3_pose_msg.position.z
        
        # Create a Pose for step3 (we need the orientation too)
        step3_pose = Pose()
        step3_pose.position.x = step3_position.x
        step3_pose.position.y = step3_position.y
        step3_pose.position.z = step3_position.z
        step3_pose.orientation.x = step3_pose_msg.orientation.x
        step3_pose.orientation.y = step3_pose_msg.orientation.y
        step3_pose.orientation.z = step3_pose_msg.orientation.z
        step3_pose.orientation.w = step3_pose_msg.orientation.w
        
        self._node.get_logger().info(
            f"[SimpleGraspExecutor] _compute_pre_grasp_pose_from_step3: ✓ Step3 end-effector pose: "
            f"pos=({step3_pose.position.x:.3f}, {step3_pose.position.y:.3f}, {step3_pose.position.z:.3f})"
        )
        
        # Compute direction vector from step3 end-effector to center_base
        # Direction: from end-effector toward center (末端到质心)
        dx = center_base.x - step3_pose.position.x
        dy = center_base.y - step3_pose.position.y
        dz = center_base.z - step3_pose.position.z
        
        # Normalize direction vector
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)
        if dist < 1e-6:
            self._node.get_logger().error(
                "[SimpleGraspExecutor] Step3 end-effector too close to center_base"
            )
            return None
        
        dx /= dist
        dy /= dist
        dz /= dist
        
        # Pre-grasp position: 0.08m away from center_base along the direction vector
        # (from center_base toward step3 end-effector, i.e., center_base - 0.08m × direction)
        # Since direction is from end-effector to center, we go from center toward end-effector
        pre_grasp_pose = Pose()
        pre_grasp_pose.position.x = center_base.x - self._approach_distance * dx
        pre_grasp_pose.position.y = center_base.y - self._approach_distance * dy
        pre_grasp_pose.position.z = center_base.z - self._approach_distance * dz
        
        # Use the same orientation as step3 (pointing toward center)
        # Copy orientation fields individually to ensure type safety
        pre_grasp_pose.orientation.x = step3_pose.orientation.x
        pre_grasp_pose.orientation.y = step3_pose.orientation.y
        pre_grasp_pose.orientation.z = step3_pose.orientation.z
        pre_grasp_pose.orientation.w = step3_pose.orientation.w
        
        pre_grasp_stamped = PoseStamped()
        pre_grasp_stamped.header.frame_id = ROBOT_BASE_FRAME
        pre_grasp_stamped.header.stamp = self._node.get_clock().now().to_msg()
        pre_grasp_stamped.pose = pre_grasp_pose
        
        self._node.get_logger().info(
            f"[SimpleGraspExecutor] Computed pre-grasp pose: "
            f"step3_pos=({step3_pose.position.x:.3f}, {step3_pose.position.y:.3f}, {step3_pose.position.z:.3f}), "
            f"center=({center_base.x:.3f}, {center_base.y:.3f}, {center_base.z:.3f}), "
            f"pre_grasp_pos=({pre_grasp_pose.position.x:.3f}, {pre_grasp_pose.position.y:.3f}, {pre_grasp_pose.position.z:.3f})"
        )
        
        return pre_grasp_stamped

    def _compute_joint_angles_for_pose(self, pose: Pose) -> Optional[list[float]]:
        """Compute joint angles for a given pose using IK (synchronous, blocking).
        
        Returns joint angles in JOINT_NAME_ORDER (joint1-joint6) or None if IK fails.
        """
        if not self._ik_client.service_is_ready():
            if not self._ik_client.wait_for_service(timeout_sec=2.0):
                self._node.get_logger().error(
                    f"❌ IK service {IK_SERVICE} not available (timeout after 2s) for joint angle computation"
                )
                return None
        
        request = SolveIK.Request()  # type: ignore[attr-defined]
        request.target_pose = pose
        request.xyz_only = False
        request.max_iters = 100
        
        try:
            future = self._ik_client.call_async(request)
            rclpy.spin_until_future_complete(self._node, future, timeout_sec=2.0)
            if not future.done():
                self._node.get_logger().error(
                    f"❌ IK service call timeout for pose: "
                    f"pos=({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f}), "
                    f"quat=({pose.orientation.x:.4f}, {pose.orientation.y:.4f}, "
                    f"{pose.orientation.z:.4f}, {pose.orientation.w:.4f})"
                )
                return None
            
            response = future.result()
            if not response.success:
                error_msg = getattr(response, 'error_message', 'Unknown error')
                quat_norm = math.sqrt(
                    pose.orientation.x**2 + pose.orientation.y**2 + 
                    pose.orientation.z**2 + pose.orientation.w**2
                )
                self._node.get_logger().error(
                    f"❌ IK solve failed for pose: "
                    f"pos=({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f}), "
                    f"quat=({pose.orientation.x:.4f}, {pose.orientation.y:.4f}, "
                    f"{pose.orientation.z:.4f}, {pose.orientation.w:.4f}), "
                    f"quat_norm={quat_norm:.6f}, error_message={error_msg}"
                )
                return None
            
            # Extract joint positions from response
            if hasattr(response, 'joint_state') and response.joint_state:
                joint_state = response.joint_state
                # Create a mapping from joint name to position
                joint_map = dict(zip(joint_state.name, joint_state.position))
                # Extract joints in JOINT_NAME_ORDER (first 6 joints: joint1-joint6)
                joint_angles = []
                for joint_name in JOINT_NAME_ORDER[:6]:
                    if joint_name in joint_map:
                        joint_angles.append(float(joint_map[joint_name]))
                    else:
                        self._node.get_logger().warning(
                            f"Joint {joint_name} not found in IK response"
                        )
                        return None
                return joint_angles if len(joint_angles) == 6 else None
            return None
        except Exception as exc:  # pylint: disable=broad-except
            self._node.get_logger().error(f"IK solve exception in _compute_joint_angles_for_pose: {exc}")
            return None

    async def _verify_ik(self, pose: Pose) -> bool:
        """Best-effort IK check using /robot_driver/service/solve_ik."""
        if not self._ik_client.service_is_ready():
            if not self._ik_client.wait_for_service(timeout_sec=2.0):
                self._node.get_logger().error(
                    f"❌ IK service {IK_SERVICE} not available (timeout after 2s)"
                )
                return False

        request = SolveIK.Request()  # type: ignore[attr-defined]
        request.target_pose = pose
        request.xyz_only = False
        request.max_iters = 100

        try:
            future = self._ik_client.call_async(request)
            rclpy.spin_until_future_complete(self._node, future, timeout_sec=2.0)
            if not future.done():
                self._node.get_logger().error(
                    f"❌ IK service {IK_SERVICE} call timeout (2s) for pose "
                    f"({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f})"
                )
                return False

            response = future.result()
            if not response.success:
                error_msg = getattr(response, 'error_message', 'Unknown error')
                self._node.get_logger().error(
                    f"❌ IK solve failed for pose "
                    f"pos=({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f}), "
                    f"quat=({pose.orientation.x:.4f}, {pose.orientation.y:.4f}, "
                    f"{pose.orientation.z:.4f}, {pose.orientation.w:.4f}), "
                    f"error: {error_msg}"
                )
            return bool(response.success)
        except Exception as exc:  # pylint: disable=broad-except
            self._node.get_logger().error(
                f"❌ IK solve exception for pose "
                f"({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f}): {exc}"
            )
            return False


__all__ = ["SimpleGraspExecutor"]


