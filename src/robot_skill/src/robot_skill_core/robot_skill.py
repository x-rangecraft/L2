"""Robot skill coordinator node with skill_sequence and grasp_record actions."""
from __future__ import annotations

import os
from typing import List

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node as RclpyNode

from robot_skill.action import GraspRecord, GraspRecordEasy, SkillSequence
from robot_skill_core.constants import (
    DEFAULT_APPROACH_DISTANCE,
    DEFAULT_GRIPPER_WIDTH_MARGIN,
    DEFAULT_MAX_GRIPPER_WIDTH,
    GRASP_RECORD_ACTION_TOPIC,
    SKILL_ACTION_TOPIC,
    SUPPORTED_STEP_TYPES,
    JOINT_NAME_ORDER,
    MIN_CENTER_Z,
)
from robot_skill_core.grasp_executor import GraspExecutor
from robot_skill_core.simple_grasp_executor import SimpleGraspExecutor
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

        # Simple grasp executor (geometry-based, no CGN)
        self._simple_grasp_executor = SimpleGraspExecutor(
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

        # Action Server 3: grasp_record_easy (geometry-only grasp)
        self._grasp_easy_server = ActionServer(
            self,
            GraspRecordEasy,
            "/robot_skill/action/grasp_record_easy",
            goal_callback=self._grasp_easy_goal_callback,
            cancel_callback=self._grasp_easy_cancel_callback,
            execute_callback=self._execute_grasp_record_easy,
        )

        self.get_logger().info(
            "robot_skill action servers ready "
            "(skill_sequence, grasp_record, grasp_record_easy)"
        )

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

    # ------------------------------------------------------------------ #
    # GraspRecordEasy Action callbacks
    # ------------------------------------------------------------------ #
    def _grasp_easy_goal_callback(
        self, goal_request: GraspRecordEasy.Goal
    ) -> GoalResponse:
        if goal_request.point_cloud is None:
            self.get_logger().warning("Rejecting grasp_easy goal without point_cloud")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _grasp_easy_cancel_callback(
        self, goal_handle: ServerGoalHandle
    ) -> CancelResponse:
        self.get_logger().info(
            f"Cancel requested for grasp_record_easy {goal_handle.request.context_id}"
        )
        return CancelResponse.ACCEPT

    async def _execute_grasp_record_easy(
        self, goal_handle: ServerGoalHandle
    ) -> GraspRecordEasy.Result:
        """Execute simplified grasp using geometric center in base_link frame."""
        req = goal_handle.request
        context_id = req.context_id

        self.get_logger().info(
            "Executing grasp_record_easy: "
            f"point_cloud points (width={req.point_cloud.width}, height={req.point_cloud.height}), "
            f"context={context_id}"
        )

        # Execute steps one by one, computing next step after each execution
        steps_executed = 0
        
        # Helper function to go to safety pose on error
        async def go_to_safety_and_return(error_code: str, error_message: str) -> GraspRecordEasy.Result:
            self.get_logger().error(f"Error: {error_message}, going to safety pose...")
            try:
                safety_step = SkillStep(
                    id='safe_on_error',
                    type='safety_pose',
                    desc='回到安全位姿（错误恢复）',
                    params={},
                )
                await self._step_executor.execute_step(safety_step)
            except Exception as exc:
                self.get_logger().error(f"Failed to go to safety pose: {exc}")
            result = GraspRecordEasy.Result()
            result.success = False
            result.error_code = error_code
            result.message = error_message
            result.steps_executed = steps_executed
            goal_handle.abort()
            return result
        
        # Calibrate center_base.z: ensure minimum height
        original_z = req.center_base.z
        if req.center_base.z < MIN_CENTER_Z:
            self.get_logger().info(
                f"[robot_skill] Calibrating center_base.z: {req.center_base.z:.3f}m -> {MIN_CENTER_Z:.3f}m"
            )
            req.center_base.z = MIN_CENTER_Z
        
        # Wrap entire execution in try-except to catch any unexpected errors
        try:
            # Step 1: Go to safety pose
            feedback = GraspRecordEasy.Feedback()
            feedback.phase = "executing"
            feedback.step_index = 0
            feedback.step_desc = "回到安全位姿"
            feedback.progress = 0.0
            goal_handle.publish_feedback(feedback)
            
            step1 = SkillStep(
                id='safe_start',
                type='safety_pose',
                desc='回到安全位姿',
                params={},
            )
            success, code, message = await self._step_executor.execute_step(step1)
            if not success:
                return await go_to_safety_and_return(code or 'STEP_FAILED', f"Step1 failed: {message}")
            steps_executed += 1
            
            # Step 2: Move to prepare position
            feedback.step_index = 1
            feedback.step_desc = "移动到准备夹取位置"
            feedback.progress = 0.1
            goal_handle.publish_feedback(feedback)
            
            from robot_skill_core.simple_grasp_executor import PREPARE_JOINT_POSITIONS
            step2 = SkillStep(
                id='move_to_prepare',
                type='joint_move',
                desc='移动到准备夹取位置',
                params={
                    'all_positions': list(PREPARE_JOINT_POSITIONS),
                    'speed_scale': 1.0,
                },
            )
            success, code, message = await self._step_executor.execute_step(step2)
            if not success:
                return await go_to_safety_and_return(code or 'STEP_FAILED', f"Step2 failed: {message}")
            steps_executed += 1
            
            # Step 3: Compute and execute joint adjustment
            feedback.step_index = 2
            feedback.step_desc = "计算并调整关节朝向质心"
            feedback.progress = 0.2
            goal_handle.publish_feedback(feedback)
            
            step3_joint_angles = self._simple_grasp_executor._compute_step3_joint_angles(req.center_base)
            if step3_joint_angles is None or len(step3_joint_angles) < 6:
                return await go_to_safety_and_return(
                    'COMPUTATION_FAILED',
                    "Step3关节角度计算失败，无法调整朝向质心"
                )
            
            step3 = SkillStep(
                id='adjust_joints_toward_target',
                type='joint_move',
                desc='调整Joint1和Joint4朝向质心',
                params={
                    'all_positions': step3_joint_angles,
                    'speed_scale': 1.0,
                },
            )
            success, code, message = await self._step_executor.execute_step(step3)
            if not success:
                return await go_to_safety_and_return(code or 'STEP_FAILED', f"Step3 failed: {message}")
            steps_executed += 1
            
            # Step 4: Compute pre-grasp pose and execute
            feedback.step_index = 3
            feedback.step_desc = "计算并移动到预抓取位置"
            feedback.progress = 0.3
            goal_handle.publish_feedback(feedback)
            
            pre_grasp_pose = self._simple_grasp_executor._compute_pre_grasp_pose_from_step3(req.center_base)
            if pre_grasp_pose is None:
                return await go_to_safety_and_return(
                    'COMPUTATION_FAILED',
                    "预抓取位置计算失败"
                )
            
            step4 = SkillStep(
                id='pre_grasp',
                type='cartesian_move',
                desc='移到预抓取位',
                params={'target_pose': pre_grasp_pose, 'speed_scale': 1.0},
            )
            success, code, message = await self._step_executor.execute_step(step4)
            if not success:
                return await go_to_safety_and_return(code or 'STEP_FAILED', f"Step4 failed: {message}")
            steps_executed += 1
            
            # Step 5-12: Compute remaining steps and execute
            # First compute grasp pose and gripper width
            # Use the pose from step4 (pre_grasp), which already uses step3's orientation
            current_pose = await self._simple_grasp_executor._get_current_end_effector_pose()
            if current_pose is None:
                return await go_to_safety_and_return(
                    'COMPUTATION_FAILED',
                    "无法获取当前末端位姿"
                )
            
            # Use step3's orientation (from current_pose which is after step4, and step4 uses step3's orientation)
            grasp_pose_obj = self._simple_grasp_executor._build_grasp_pose(
                req.center_base, 
                current_pose, 
                use_current_orientation=True  # Use step3's orientation instead of computing new one
            )
            gripper_width = min(0.05 + self._simple_grasp_executor._gripper_width_margin, 
                               self._simple_grasp_executor._max_gripper_width)
            
            # Log the final grasp pose that will be used for IK
            self.get_logger().info(
                f"[robot_skill] Final grasp pose for IK: "
                f"pos=({grasp_pose_obj.position.x:.3f}, {grasp_pose_obj.position.y:.3f}, "
                f"{grasp_pose_obj.position.z:.3f}), "
                f"quat=({grasp_pose_obj.orientation.x:.4f}, {grasp_pose_obj.orientation.y:.4f}, "
                f"{grasp_pose_obj.orientation.z:.4f}, {grasp_pose_obj.orientation.w:.4f})"
            )
            
            grasp_stamped = PoseStamped()
            grasp_stamped.header.frame_id = "base_link"
            grasp_stamped.header.stamp = self.get_clock().now().to_msg()
            grasp_stamped.pose = grasp_pose_obj
            
            # Build remaining steps using GraspExecutor
            remaining_steps = self._simple_grasp_executor._delegate.build_grasp_steps(
                grasp_pose=grasp_stamped,
                pre_grasp_pose=pre_grasp_pose,
                gripper_width=gripper_width,
            )
            # Skip the first step (safe_start) since we already executed it
            remaining_steps = remaining_steps[1:]
            
            # Execute remaining steps
            for idx, step in enumerate(remaining_steps):
                if goal_handle.is_cancel_requested:
                    return await go_to_safety_and_return('CANCELED', 'Execution canceled by user')
                
                feedback.step_index = 4 + idx
                feedback.step_desc = step.desc
                feedback.progress = 0.4 + 0.6 * float(idx) / float(len(remaining_steps)) if remaining_steps else 0.4
                goal_handle.publish_feedback(feedback)
                
                success, code, message = await self._step_executor.execute_step(step)
                if not success:
                    return await go_to_safety_and_return(code or 'STEP_FAILED', f"Step{4+idx} ({step.id}) failed: {message}")
                steps_executed += 1
            
            success = True
            code = 'SUCCESS'
            message = 'All steps completed successfully'

            result = GraspRecordEasy.Result()
            result.success = success
            result.error_code = code
            result.message = message
            result.steps_executed = steps_executed

            if success:
                goal_handle.succeed()
            elif code == "CANCELED":
                goal_handle.canceled()
            else:
                goal_handle.abort()

            return result
        
        except Exception as exc:
            # Catch any unexpected exceptions and go to safety
            import traceback
            error_trace = traceback.format_exc()
            self.get_logger().error(f"Unexpected error in grasp_record_easy: {exc}\n{error_trace}")
            return await go_to_safety_and_return(
                'UNEXPECTED_ERROR',
                f"Unexpected error: {str(exc)}"
            )

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
