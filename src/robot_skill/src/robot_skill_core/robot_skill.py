"""Robot skill coordinator node with skill_sequence and grasp_record actions."""
from __future__ import annotations

import os
from typing import List

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node as RclpyNode
from geometry_msgs.msg import Point
from builtin_interfaces.msg import Time as TimeMsg

from robot_skill.action import GraspRecord, SkillSequence
from tf_tools.srv import TransformPoints
from robot_skill_core.constants import (
    DEFAULT_APPROACH_DISTANCE,
    DEFAULT_GRIPPER_WIDTH_MARGIN,
    DEFAULT_MAX_GRIPPER_WIDTH,
    GRASP_RECORD_ACTION_TOPIC,
    SKILL_ACTION_TOPIC,
    SUPPORTED_STEP_TYPES,
    JOINT_NAME_ORDER,
)

# TF 坐标系配置
CAMERA_FRAME = 'camera_color_optical_frame'
ROBOT_BASE_FRAME = 'base_link'
TF_TRANSFORM_SERVICE = '/tf_tools/service/transform_points'
from robot_skill_core.grasp_planner import GraspPlanner
from robot_skill_core.skill_loader import SkillLibrary, SkillStep
from robot_skill_core.step_executor import StepExecutor


class RobotSkill(RclpyNode):
    """
    Robot skill coordinator node.

    Exposes two Action servers:
    - /robot_skill/action/skill_sequence: Execute YAML-defined skill sequences
    - /robot_skill/action/grasp_record: Dynamic grasp-observe-place operation
    """

    def __init__(self) -> None:
        super().__init__('robot_skill')
        self._declare_parameters()

        # Load skill library
        skill_sets_dir = self.get_parameter('skill_sets_dir').get_parameter_value().string_value
        if not skill_sets_dir:
            package_share = get_package_share_directory('robot_skill')
            skill_sets_dir = os.path.join(package_share, 'skill_sets')
            self.get_logger().info(f'Using default skill_sets_dir={skill_sets_dir}')

        self._skill_library = SkillLibrary(skill_sets_dir)
        self._self_check_skill_sets()

        # Shared step executor
        self._step_executor = StepExecutor(self)

        # Grasp planner for dynamic grasp operations
        # Get observe step from common actions (single source of truth)
        observe_step = self._skill_library.get_common_action('move_record_observe')
        self._grasp_planner = GraspPlanner(
            observe_step=observe_step,
            approach_distance=DEFAULT_APPROACH_DISTANCE,
            gripper_width_margin=DEFAULT_GRIPPER_WIDTH_MARGIN,
        )

        # Action Server 1: skill_sequence (YAML-defined skills)
        self._skill_server = ActionServer(
            self,
            SkillSequence,
            SKILL_ACTION_TOPIC,
            goal_callback=self._skill_goal_callback,
            cancel_callback=self._skill_cancel_callback,
            execute_callback=self._execute_skill_sequence,
        )

        # Action Server 2: grasp_record (dynamic grasp operation)
        self._grasp_server = ActionServer(
            self,
            GraspRecord,
            GRASP_RECORD_ACTION_TOPIC,
            goal_callback=self._grasp_goal_callback,
            cancel_callback=self._grasp_cancel_callback,
            execute_callback=self._execute_grasp_record,
        )

        # TF 转换服务客户端
        self._tf_client = self.create_client(TransformPoints, TF_TRANSFORM_SERVICE)

        self.get_logger().info('robot_skill action servers ready')

    # ------------------------------------------------------------------ #
    # Parameter handling
    # ------------------------------------------------------------------ #
    def _declare_parameters(self) -> None:
        package_share = get_package_share_directory('robot_skill')
        default_skill_dir = os.path.join(package_share, 'skill_sets')
        self.declare_parameter('skill_sets_dir', default_skill_dir)

    # ------------------------------------------------------------------ #
    # Configuration self-check
    # ------------------------------------------------------------------ #
    def _self_check_skill_sets(self) -> None:
        errors: List[str] = []
        common_actions = list(self._skill_library.iter_common_actions())
        for action in common_actions:
            try:
                self._validate_step_schema(action)
            except ValueError as exc:
                errors.append(f"common action '{action.id}': {exc}")

        skill_ids = sorted(self._skill_library.list_skills())
        for skill_id in skill_ids:
            try:
                skill_def = self._skill_library.load_skill(skill_id)
            except Exception as exc:  # pylint: disable=broad-except
                errors.append(f"skill '{skill_id}' failed to load: {exc}")
                continue
            for step in skill_def.steps:
                try:
                    self._validate_step_schema(step)
                except ValueError as exc:
                    errors.append(f"skill '{skill_id}' step '{step.id}': {exc}")

        if errors:
            for message in errors:
                self.get_logger().error(f'Skill self-check error: {message}')
            raise RuntimeError('skill_sets validation failed; see errors above')

        self.get_logger().info(
            'Skill set self-check passed: '
            f"common_actions={len(common_actions)}, skills={len(skill_ids)}"
        )

    def _validate_step_schema(self, step: SkillStep) -> None:
        if step.type not in SUPPORTED_STEP_TYPES:
            raise ValueError(f"unsupported type '{step.type}'")

        params = step.params or {}
        if step.type == 'safety_pose':
            return
        if step.type == 'cartesian_move':
            pose_dict = params.get('target_pose')
            if not isinstance(pose_dict, dict):
                raise ValueError("cartesian_move requires 'target_pose'")
            position = pose_dict.get('position')
            orientation = pose_dict.get('orientation')
            if not isinstance(position, dict) or not isinstance(orientation, dict):
                raise ValueError("cartesian_move requires position and orientation dicts")
            for axis in ('x', 'y', 'z'):
                if axis not in position:
                    raise ValueError(f"cartesian_move missing position.{axis}")
            for axis in ('x', 'y', 'z', 'w'):
                if axis not in orientation:
                    raise ValueError(f"cartesian_move missing orientation.{axis}")
            return
        if step.type == 'gripper':
            if 'command' not in params:
                raise ValueError("gripper requires 'command'")
            return
        if step.type == 'joint_move':
            # Mode 1: all_positions - set all 6 joints at once
            all_positions = params.get('all_positions')
            if all_positions is not None:
                if not isinstance(all_positions, (list, tuple)):
                    raise ValueError("joint_move all_positions must be a list")
                if len(all_positions) != len(JOINT_NAME_ORDER):
                    raise ValueError(
                        f"joint_move all_positions must have {len(JOINT_NAME_ORDER)} values"
                    )
                return
            # Mode 2: single joint with positions list
            positions = params.get('positions')
            if not isinstance(positions, (list, tuple)) or not positions:
                raise ValueError("joint_move requires 'positions' or 'all_positions'")
            if params.get('joint_name') is None:
                index = params.get('joint_index')
                if index is None:
                    raise ValueError("joint_move requires 'joint_name' or 'joint_index'")
                try:
                    index_val = int(index)
                except (TypeError, ValueError) as exc:
                    raise ValueError('joint_move joint_index must be an integer') from exc
                if index_val < 0 or index_val >= len(JOINT_NAME_ORDER):
                    raise ValueError('joint_move joint_index out of range (0-5)')
            return
        raise ValueError(f"unsupported type '{step.type}'")

    # ------------------------------------------------------------------ #
    # SkillSequence Action callbacks
    # ------------------------------------------------------------------ #
    def _skill_goal_callback(self, goal_request: SkillSequence.Goal) -> GoalResponse:
        if not goal_request.skill_id:
            self.get_logger().warning('Rejecting goal without skill_id')
            return GoalResponse.REJECT
        if not self._skill_library.has_skill(goal_request.skill_id):
            self.get_logger().warning(f"Skill '{goal_request.skill_id}' not found")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _skill_cancel_callback(self, goal_handle: ServerGoalHandle) -> CancelResponse:
        self.get_logger().info(f'Cancel requested for skill {goal_handle.request.skill_id}')
        return CancelResponse.ACCEPT

    async def _execute_skill_sequence(
        self, goal_handle: ServerGoalHandle
    ) -> SkillSequence.Result:
        """Execute a YAML-defined skill sequence."""
        skill_id = goal_handle.request.skill_id
        context_id = goal_handle.request.context_id

        try:
            skill_def = self._skill_library.load_skill(skill_id)
        except Exception as exc:
            self.get_logger().error(f'Failed to load skill {skill_id}: {exc}')
            result = SkillSequence.Result()
            result.success = False
            result.error_code = 'LOAD_FAILED'
            result.message = str(exc)
            goal_handle.abort()
            return result

        total_steps = len(skill_def.steps)
        self.get_logger().info(
            f"Executing skill '{skill_id}' ({total_steps} steps), context={context_id}"
        )

        def on_progress(index: int, step: SkillStep) -> None:
            progress = float(index) / float(total_steps) if total_steps else 0.0
            feedback = SkillSequence.Feedback()
            feedback.step_index = index
            feedback.step_id = step.id
            feedback.step_desc = step.desc
            feedback.progress = progress
            feedback.detail = f'Executing {step.type}'
            goal_handle.publish_feedback(feedback)

        success, code, message, steps_executed = await self._step_executor.execute_steps(
            skill_def.steps,
            on_progress=on_progress,
            check_cancel=lambda: goal_handle.is_cancel_requested,
        )

        result = SkillSequence.Result()
        result.success = success
        result.error_code = code
        result.message = message
        result.steps_executed = steps_executed

        if success:
            goal_handle.succeed()
        elif code == 'CANCELED':
            goal_handle.canceled()
        else:
            goal_handle.abort()

        return result

    # ------------------------------------------------------------------ #
    # GraspRecord Action callbacks
    # ------------------------------------------------------------------ #
    def _grasp_goal_callback(self, goal_request: GraspRecord.Goal) -> GoalResponse:
        if not goal_request.boundary_points:
            self.get_logger().warning('Rejecting grasp goal without boundary_points')
            return GoalResponse.REJECT
        if goal_request.max_gripper_width <= 0:
            self.get_logger().warning('Rejecting grasp goal with invalid max_gripper_width')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _grasp_cancel_callback(self, goal_handle: ServerGoalHandle) -> CancelResponse:
        self.get_logger().info(f'Cancel requested for grasp_record {goal_handle.request.context_id}')
        return CancelResponse.ACCEPT

    async def _transform_points_to_base(
        self, points: List[Point]
    ) -> List[Point] | None:
        """
        Transform points from camera frame to base_link frame.
        
        Args:
            points: List of points in camera frame
            
        Returns:
            List of points in base_link frame, or None if transform failed
        """
        if not self._tf_client.service_is_ready():
            self.get_logger().warning(
                f'TF service {TF_TRANSFORM_SERVICE} not available, waiting...'
            )
            if not self._tf_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error(
                    f'TF service {TF_TRANSFORM_SERVICE} not available after 5s'
                )
                return None

        # Build request
        request = TransformPoints.Request()
        request.source_frame = CAMERA_FRAME
        request.target_frame = ROBOT_BASE_FRAME
        request.stamp = TimeMsg()  # Use latest transform (sec=0, nanosec=0)
        request.points_in = points

        try:
            future = self._tf_client.call_async(request)
            # Wait for result with timeout
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if not future.done():
                self.get_logger().error('TF transform service call timed out')
                return None
                
            response = future.result()
            if not response.success:
                self.get_logger().error(f'TF transform failed: {response.message}')
                return None
                
            return list(response.points_out)
            
        except Exception as e:
            self.get_logger().error(f'TF transform exception: {e}')
            return None

    async def _execute_grasp_record(
        self, goal_handle: ServerGoalHandle
    ) -> GraspRecord.Result:
        """Execute dynamic grasp-observe-place operation."""
        req = goal_handle.request
        context_id = req.context_id
        max_gripper_width = req.max_gripper_width or DEFAULT_MAX_GRIPPER_WIDTH

        self.get_logger().info(
            f"Executing grasp_record: {len(req.boundary_points)} points, "
            f"max_width={max_gripper_width:.3f}m, context={context_id}"
        )

        # Phase 0: Transform points from camera frame to base_link
        planning_feedback = GraspRecord.Feedback()
        planning_feedback.phase = 'planning'
        planning_feedback.step_index = 0
        planning_feedback.step_desc = 'Transforming coordinates'
        planning_feedback.progress = 0.0
        goal_handle.publish_feedback(planning_feedback)

        transformed_points = await self._transform_points_to_base(
            list(req.boundary_points)
        )
        if transformed_points is None:
            result = GraspRecord.Result()
            result.success = False
            result.error_code = 'TF_FAILED'
            result.message = 'Failed to transform points from camera to base_link'
            result.steps_executed = 0
            goal_handle.abort()
            return result

        self.get_logger().info(
            f"Points transformed: {len(transformed_points)} points in base_link frame"
        )

        # Phase 1: Planning
        planning_feedback.step_desc = 'Computing grasp pose'
        planning_feedback.progress = 0.1
        goal_handle.publish_feedback(planning_feedback)

        try:
            steps = self._grasp_planner.build_grasp_record_steps(
                transformed_points,
                max_gripper_width,
            )
        except ValueError as exc:
            self.get_logger().error(f'Grasp planning failed: {exc}')
            result = GraspRecord.Result()
            result.success = False
            result.error_code = 'PLANNING_FAILED'
            result.message = str(exc)
            result.steps_executed = 0
            goal_handle.abort()
            return result

        total_steps = len(steps)
        self.get_logger().info(f'Grasp plan generated: {total_steps} steps')

        # Phase 2: Executing
        def on_progress(index: int, step: SkillStep) -> None:
            progress = float(index) / float(total_steps) if total_steps else 0.0
            feedback = GraspRecord.Feedback()
            feedback.phase = 'executing'
            feedback.step_index = index
            feedback.step_desc = step.desc
            feedback.progress = progress
            goal_handle.publish_feedback(feedback)

        success, code, message, steps_executed = await self._step_executor.execute_steps(
            steps,
            on_progress=on_progress,
            check_cancel=lambda: goal_handle.is_cancel_requested,
        )

        result = GraspRecord.Result()
        result.success = success
        result.error_code = code
        result.message = message
        result.steps_executed = steps_executed

        if success:
            goal_handle.succeed()
        elif code == 'CANCELED':
            goal_handle.canceled()
        else:
            goal_handle.abort()

        return result


def main() -> None:
    rclpy.init()
    node = RobotSkill()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('robot_skill shutdown requested by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


__all__ = ['RobotSkill', 'main']

