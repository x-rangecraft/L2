"""Simple grasp executor based on geometric center and bbox (no CGN)."""
from __future__ import annotations

import copy
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Point, Pose, PoseStamped
from rclpy.node import Node as RclpyNode

from robot_driver.srv import SolveFK, SolveIK

from robot_skill_core.constants import (
    DEFAULT_APPROACH_DISTANCE,
    DEFAULT_GRIPPER_WIDTH_MARGIN,
    DEFAULT_MAX_GRIPPER_WIDTH,
)
from robot_skill_core.skill_loader import SkillStep
from robot_skill_core.grasp_executor import GraspExecutor


ROBOT_BASE_FRAME = "base_link"

FK_SERVICE = "/robot_driver/service/solve_fk"
IK_SERVICE = "/robot_driver/service/solve_ik"


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

        # FK / IK clients
        self._fk_client = node.create_client(SolveFK, FK_SERVICE)
        self._ik_client = node.create_client(SolveIK, IK_SERVICE)

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
        current_pose = await self._get_current_end_effector_pose()
        if current_pose is None:
            self._node.get_logger().error(
                "[SimpleGraspExecutor] Failed to get current end-effector pose"
            )
            return None, None, 0.0

        grasp_pose = self._build_grasp_pose(center_base, current_pose)
        pre_pose_stamped = self._delegate._compute_pre_grasp_pose(grasp_pose)  # noqa: SLF001

        # IK validation (best-effort, keep simple)
        if not await self._verify_ik(pre_pose_stamped.pose):
            self._node.get_logger().warning(
                "[SimpleGraspExecutor] Pre-grasp IK failed, aborting plan"
            )
            return None, None, 0.0

        if not await self._verify_ik(grasp_pose):
            self._node.get_logger().warning(
                "[SimpleGraspExecutor] Grasp IK failed, aborting plan"
            )
            return None, None, 0.0

        # For now use a conservative constant width (mid-range), can be tuned.
        gripper_width = min(0.05 + self._gripper_width_margin, self._max_gripper_width)

        grasp_stamped = PoseStamped()
        grasp_stamped.header.frame_id = ROBOT_BASE_FRAME
        grasp_stamped.header.stamp = self._node.get_clock().now().to_msg()
        grasp_stamped.pose = grasp_pose

        return grasp_stamped, pre_pose_stamped, gripper_width

    def build_grasp_steps(
        self,
        grasp_pose: PoseStamped,
        pre_grasp_pose: PoseStamped,
        gripper_width: float,
    ) -> list[SkillStep]:
        """Delegate to :class:`GraspExecutor` for step sequence creation."""
        return self._delegate.build_grasp_steps(
            grasp_pose=grasp_pose,
            pre_grasp_pose=pre_grasp_pose,
            gripper_width=gripper_width,
        )

    # ------------------------------------------------------------------ #
    # Internal helpers
    # ------------------------------------------------------------------ #
    async def _get_current_end_effector_pose(self) -> Optional[Pose]:
        """Query current end-effector pose via FK service."""
        if not self._fk_client.service_is_ready():
            if not self._fk_client.wait_for_service(timeout_sec=2.0):
                self._node.get_logger().error(
                    f"FK service {FK_SERVICE} not available"
                )
                return None

        request = SolveFK.Request()  # type: ignore[attr-defined]
        # Empty joint_state -> use current hardware state.

        try:
            future = self._fk_client.call_async(request)
            rclpy.spin_until_future_complete(self._node, future, timeout_sec=2.0)
            if not future.done():
                self._node.get_logger().error(
                    f"FK service {FK_SERVICE} call timeout"
                )
                return None

            response = future.result()
            if not response.success:
                self._node.get_logger().error(
                    f"FK solve failed: {response.error_message}"
                )
                return None

            return response.end_effector_pose.pose
        except Exception as exc:  # pylint: disable=broad-except
            self._node.get_logger().error(f"FK solve exception: {exc}")
            return None

    def _build_grasp_pose(self, center_base: Point, current_pose: Pose) -> Pose:
        """Build grasp pose using line from current ee to object center."""
        grasp_pose = copy.deepcopy(current_pose)
        grasp_pose.position = center_base
        # Keep current orientation for now, simple but usually stable.
        return grasp_pose

    async def _verify_ik(self, pose: Pose) -> bool:
        """Best-effort IK check using /robot_driver/service/solve_ik."""
        if not self._ik_client.service_is_ready():
            if not self._ik_client.wait_for_service(timeout_sec=2.0):
                self._node.get_logger().warning(
                    f"IK service {IK_SERVICE} not available"
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
                self._node.get_logger().warning(
                    f"IK service {IK_SERVICE} call timeout"
                )
                return False

            response = future.result()
            if not response.success:
                self._node.get_logger().info(
                    "IK solve failed for pose "
                    f"({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f})"
                )
            return bool(response.success)
        except Exception as exc:  # pylint: disable=broad-except
            self._node.get_logger().error(f"IK solve exception: {exc}")
            return False


__all__ = ["SimpleGraspExecutor"]


