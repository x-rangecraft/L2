"""Robot skill coordinator node with skill_sequence and grasp_record actions."""
from __future__ import annotations

import os
from typing import List

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node as RclpyNode

from robot_skill.action import GraspRecord, SkillSequence
from robot_skill_core.constants import (
    DEFAULT_APPROACH_DISTANCE,
    DEFAULT_GRIPPER_WIDTH_MARGIN,
    DEFAULT_MAX_GRIPPER_WIDTH,
    GRASP_RECORD_ACTION_TOPIC,
    SKILL_ACTION_TOPIC,
    SUPPORTED_STEP_TYPES,
    JOINT_NAME_ORDER,
)
from robot_skill_core.grasp_executor import GraspExecutor
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

        # Grasp executor for dynamic grasp operations
        # Get observe step from common actions (single source of truth)
        observe_step = self._skill_library.get_common_action('move_record_observe')
        self._grasp_executor = GraspExecutor(
            node=self,
            observe_step=observe_step,
            approach_distance=DEFAULT_APPROACH_DISTANCE,
            gripper_width_margin=DEFAULT_GRIPPER_WIDTH_MARGIN,
            max_gripper_width=DEFAULT_MAX_GRIPPER_WIDTH,
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
        if not goal_request.grasp_candidates:
            self.get_logger().warning('Rejecting grasp goal without grasp_candidates')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _grasp_cancel_callback(self, goal_handle: ServerGoalHandle) -> CancelResponse:
        self.get_logger().info(f'Cancel requested for grasp_record {goal_handle.request.context_id}')
        return CancelResponse.ACCEPT

    async def _execute_grasp_record(
        self, goal_handle: ServerGoalHandle
    ) -> GraspRecord.Result:
        """Execute dynamic grasp-observe-place operation using pre-computed grasp candidates."""
        req = goal_handle.request
        context_id = req.context_id
        candidates = list(req.grasp_candidates)
        source_frame = req.source_frame_id if req.source_frame_id else 'camera_color_optical_frame'

        self.get_logger().info(
            f"Executing grasp_record: {len(candidates)} candidates, "
            f"source_frame={source_frame}, context={context_id}"
        )

        # Phase 1: TF 转换 + IK 验证
        tf_feedback = GraspRecord.Feedback()
        tf_feedback.phase = 'tf_converting'
        tf_feedback.step_index = 0
        tf_feedback.step_desc = 'Transforming candidate poses to base_link'
        tf_feedback.progress = 0.0
        tf_feedback.current_candidate_index = 0
        goal_handle.publish_feedback(tf_feedback)

        # 使用 GraspExecutor 查找可执行的候选
        def on_ik_progress(current_index: int, total: int, progress: float) -> None:
            """IK验证进度回调"""
            ik_feedback = GraspRecord.Feedback()
            ik_feedback.phase = 'ik_testing'
            ik_feedback.step_index = 0
            ik_feedback.step_desc = f'Validating IK solutions ({current_index+1}/{total})'
            ik_feedback.progress = progress
            ik_feedback.current_candidate_index = current_index
            goal_handle.publish_feedback(ik_feedback)

        candidate_idx, grasp_pose, pre_grasp_pose, gripper_width = \
            await self._grasp_executor.find_executable_candidate(
                candidates, source_frame, on_progress=on_ik_progress
            )

        if candidate_idx < 0:
            self.get_logger().error(
                f'All {len(candidates)} candidates failed IK validation'
            )
            result = GraspRecord.Result()
            result.success = False
            result.error_code = 'ALL_IK_FAILED'
            result.message = f'尝试了 {len(candidates)} 个候选，均无法解算 IK'
            result.steps_executed = 0
            result.candidate_used = -1
            goal_handle.abort()
            return result

        self.get_logger().info(
            f'Using candidate {candidate_idx} (confidence={candidates[candidate_idx].confidence:.3f}), '
            f'gripper_width={gripper_width:.3f}m'
        )

        # Phase 2: 生成步骤序列
        steps = self._grasp_executor.build_grasp_steps(
            grasp_pose, pre_grasp_pose, gripper_width
        )
        total_steps = len(steps)
        self.get_logger().info(f'Grasp plan generated: {total_steps} steps')

        # Phase 3: 执行步骤
        def on_progress(index: int, step: SkillStep) -> None:
            progress = 0.2 + 0.8 * float(index) / float(total_steps) if total_steps else 0.2
            feedback = GraspRecord.Feedback()
            feedback.phase = 'executing'
            feedback.step_index = index
            feedback.step_desc = step.desc
            feedback.progress = progress
            feedback.current_candidate_index = candidate_idx
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
        result.candidate_used = candidate_idx

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
