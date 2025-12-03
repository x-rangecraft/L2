"""Grasp executor for TF transformation and IK validation."""
from __future__ import annotations

import copy
from typing import List, Optional, Tuple, Callable

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.node import Node as RclpyNode
from builtin_interfaces.msg import Time as TimeMsg

from tf_tools.srv import TransformPoses
from robot_driver.srv import SolveIK
from perception.msg import GraspCandidate

from robot_skill_core.constants import (
    DEFAULT_APPROACH_DISTANCE,
    DEFAULT_GRIPPER_WIDTH_MARGIN,
    DEFAULT_MAX_GRIPPER_WIDTH,
)
from robot_skill_core.skill_loader import SkillStep


# 坐标系配置
CAMERA_FRAME = 'camera_color_optical_frame'
ROBOT_BASE_FRAME = 'base_link'

# 服务名称
TF_TRANSFORM_POSES_SERVICE = '/tf_tools/service/transform_poses'
IK_SOLVE_SERVICE = '/robot_driver/service/solve_ik'

# 服务超时配置
SERVICE_WAIT_TIMEOUT = 5.0  # 等待服务可用的超时时间（秒）
SERVICE_CALL_TIMEOUT = 2.0  # IK服务调用超时时间（秒）- 缩短以提高响应速度


class GraspExecutor:
    """
    Grasp executor that handles TF transformation and IK validation.
    
    This class replaces GraspPlanner and uses pre-computed grasp candidates
    from Contact-GraspNet instead of computing grasp poses from OBB.
    """

    def __init__(
        self,
        node: RclpyNode,
        observe_step: SkillStep,
        approach_distance: float = DEFAULT_APPROACH_DISTANCE,
        gripper_width_margin: float = DEFAULT_GRIPPER_WIDTH_MARGIN,
        max_gripper_width: float = DEFAULT_MAX_GRIPPER_WIDTH,
    ) -> None:
        """
        Initialize the grasp executor.

        Args:
            node: ROS2 node instance for creating service clients
            observe_step: SkillStep for moving to observe position (from general_skill.yaml)
            approach_distance: Height offset for pre-grasp pose (default 8cm)
            gripper_width_margin: Extra margin added to gripper width (default 2cm)
            max_gripper_width: Maximum gripper opening width (default 94mm)
        """
        self._node = node
        self._observe_step = observe_step
        self._approach_distance = approach_distance
        self._gripper_width_margin = gripper_width_margin
        self._max_gripper_width = max_gripper_width

        # 创建服务客户端
        self._tf_poses_client = node.create_client(
            TransformPoses, TF_TRANSFORM_POSES_SERVICE
        )
        self._ik_client = node.create_client(
            SolveIK, IK_SOLVE_SERVICE
        )

    # ================================================================
    # 公开方法
    # ================================================================

    async def find_executable_candidate(
        self,
        candidates: List[GraspCandidate],
        source_frame: str = CAMERA_FRAME,
        on_progress: Optional[Callable[[int, int, float], None]] = None,
    ) -> Tuple[int, Optional[PoseStamped], Optional[PoseStamped], float]:
        """
        Find the first candidate that passes IK validation.

        Process:
        1. If source_frame != base_link, batch transform all candidate poses to base_link
        2. For each transformed pose, verify IK for both pre-grasp and grasp poses
        3. Return the first candidate that passes both IK checks

        Args:
            candidates: List of GraspCandidate from Contact-GraspNet
            source_frame: Source coordinate frame (if already base_link, skip TF transform)
            on_progress: Optional callback function(current_index, total, progress) to update progress

        Returns:
            Tuple of (candidate_index, grasp_pose, pre_grasp_pose, gripper_width)
            If all candidates fail, returns (-1, None, None, 0.0)
        """
        if not candidates:
            self._node.get_logger().warning('No grasp candidates provided')
            return (-1, None, None, 0.0)

        # Phase 1: TF 转换（若已是 base_link 则跳过）
        if source_frame == ROBOT_BASE_FRAME:
            self._node.get_logger().info(
                f'Source frame is already {ROBOT_BASE_FRAME}, skipping TF transform'
            )
            # 直接构建 Pose 列表
            transformed_poses = []
            for candidate in candidates:
                pose = Pose()
                pose.position = candidate.position
                pose.orientation = candidate.orientation
                transformed_poses.append(pose)
        else:
            self._node.get_logger().info(
                f'Transforming {len(candidates)} candidates from {source_frame} to {ROBOT_BASE_FRAME}'
            )
            transformed_poses = await self._transform_candidates_to_base(
                candidates, source_frame
            )
            if transformed_poses is None:
                self._node.get_logger().error('Failed to transform candidate poses')
                return (-1, None, None, 0.0)

        # Phase 2: 逐个 IK 验证
        total_candidates = len(transformed_poses)
        for i, (pose_base, candidate) in enumerate(zip(transformed_poses, candidates)):
            if pose_base is None:
                continue

            # 更新进度反馈
            if on_progress:
                progress = 0.1 + 0.1 * (float(i) / float(total_candidates))  # 10%-20% 进度范围
                on_progress(i, total_candidates, progress)

            # 计算预抓取位姿
            pre_grasp_pose = self._compute_pre_grasp_pose(pose_base)

            # 验证预抓取位姿 IK
            self._node.get_logger().info(
                f'Testing candidate {i+1}/{total_candidates} '
                f'(confidence={candidate.confidence:.3f}, '
                f'pos=({pose_base.position.x:.3f}, {pose_base.position.y:.3f}, {pose_base.position.z:.3f})): '
                f'pre-grasp IK...'
            )
            if not await self._verify_ik(pre_grasp_pose.pose):
                self._node.get_logger().info(
                    f'Candidate {i+1}: pre-grasp IK failed'
                )
                continue

            # 验证抓取位姿 IK
            self._node.get_logger().info(
                f'Candidate {i+1}: pre-grasp IK passed, testing grasp IK...'
            )
            if not await self._verify_ik(pose_base):
                self._node.get_logger().info(
                    f'Candidate {i+1}: grasp IK failed'
                )
                continue

            # 两者都成功，计算夹爪宽度
            gripper_width = min(
                candidate.width + self._gripper_width_margin,
                self._max_gripper_width
            )

            # 构建 PoseStamped
            grasp_pose = PoseStamped()
            grasp_pose.header.frame_id = ROBOT_BASE_FRAME
            grasp_pose.header.stamp = self._node.get_clock().now().to_msg()
            grasp_pose.pose = pose_base

            self._node.get_logger().info(
                f'Candidate {i} passed IK validation (confidence={candidate.confidence:.3f})'
            )
            return (i, grasp_pose, pre_grasp_pose, gripper_width)

        self._node.get_logger().warning(
            f'All {len(candidates)} candidates failed IK validation'
        )
        return (-1, None, None, 0.0)

    def build_grasp_steps(
        self,
        grasp_pose: PoseStamped,
        pre_grasp_pose: PoseStamped,
        gripper_width: float,
    ) -> List[SkillStep]:
        """
        Build complete grasp-observe-place step sequence.

        Args:
            grasp_pose: Target grasp pose in base_link frame
            pre_grasp_pose: Pre-grasp pose (above grasp pose)
            gripper_width: Gripper opening width

        Returns:
            List of SkillStep for the complete operation
        """
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

    # ================================================================
    # 内部方法：TF 转换
    # ================================================================

    async def _transform_candidates_to_base(
        self,
        candidates: List[GraspCandidate],
        source_frame: str,
    ) -> Optional[List[Pose]]:
        """
        Batch transform all candidate poses from camera frame to base_link.

        Args:
            candidates: List of GraspCandidate
            source_frame: Source coordinate frame

        Returns:
            List of transformed Pose in base_link frame, or None if failed
        """
        # 等待服务可用
        if not self._tf_poses_client.service_is_ready():
            self._node.get_logger().warning(
                f'TF service {TF_TRANSFORM_POSES_SERVICE} not available, waiting...'
            )
            if not self._tf_poses_client.wait_for_service(timeout_sec=SERVICE_WAIT_TIMEOUT):
                self._node.get_logger().error(
                    f'TF service {TF_TRANSFORM_POSES_SERVICE} not available after {SERVICE_WAIT_TIMEOUT}s'
                )
                return None

        # 构建请求：将所有候选的 position + orientation 转换为 Pose[]
        poses_in = []
        for candidate in candidates:
            pose = Pose()
            pose.position = candidate.position
            pose.orientation = candidate.orientation
            poses_in.append(pose)

        request = TransformPoses.Request()
        request.source_frame = source_frame
        request.target_frame = ROBOT_BASE_FRAME
        request.stamp = TimeMsg()  # 使用最新 TF（sec=0, nanosec=0）
        request.poses_in = poses_in

        try:
            future = self._tf_poses_client.call_async(request)
            rclpy.spin_until_future_complete(
                self._node, future, timeout_sec=SERVICE_CALL_TIMEOUT
            )

            if not future.done():
                self._node.get_logger().error('TF transform_poses service call timed out')
                return None

            response = future.result()
            if not response.success:
                self._node.get_logger().error(f'TF transform failed: {response.message}')
                return None

            return list(response.poses_out)

        except Exception as e:
            self._node.get_logger().error(f'TF transform exception: {e}')
            return None

    # ================================================================
    # 内部方法：IK 验证
    # ================================================================

    async def _verify_ik(self, pose: Pose) -> bool:
        """
        Verify if a pose can be solved by inverse kinematics.

        Args:
            pose: Target pose to verify

        Returns:
            True if IK solution exists, False otherwise
        """
        # 等待服务可用
        if not self._ik_client.service_is_ready():
            self._node.get_logger().warning(
                f'IK service {IK_SOLVE_SERVICE} not available, waiting...'
            )
            if not self._ik_client.wait_for_service(timeout_sec=SERVICE_WAIT_TIMEOUT):
                self._node.get_logger().error(
                    f'IK service {IK_SOLVE_SERVICE} not available after {SERVICE_WAIT_TIMEOUT}s'
                )
                return False

        request = SolveIK.Request()
        request.target_pose = pose
        request.xyz_only = False
        request.max_iters = 100  # 减少迭代次数，如果100次还解不出来，可能位姿不合理
        # seed_joints 留空，使用当前硬件状态

        try:
            future = self._ik_client.call_async(request)
            rclpy.spin_until_future_complete(
                self._node, future, timeout_sec=SERVICE_CALL_TIMEOUT
            )

            if not future.done():
                self._node.get_logger().warning(
                    f'IK solve service call timed out after {SERVICE_CALL_TIMEOUT}s '
                    f'(pose=({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f}))'
                )
                return False

            response = future.result()
            if not response.success:
                self._node.get_logger().debug(
                    f'IK solve failed for pose=({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f})'
                )
            return response.success

        except Exception as e:
            self._node.get_logger().error(f'IK solve exception: {e}')
            return False

    # ================================================================
    # 内部方法：位姿计算
    # ================================================================

    def _compute_pre_grasp_pose(self, grasp_pose: Pose) -> PoseStamped:
        """
        Compute pre-grasp pose (approach position above the grasp pose).

        Args:
            grasp_pose: The target grasp pose

        Returns:
            PoseStamped with position elevated by approach_distance
        """
        pre_grasp = PoseStamped()
        pre_grasp.header.frame_id = ROBOT_BASE_FRAME
        pre_grasp.header.stamp = self._node.get_clock().now().to_msg()
        pre_grasp.pose = copy.deepcopy(grasp_pose)
        pre_grasp.pose.position.z += self._approach_distance
        return pre_grasp


__all__ = ['GraspExecutor']

