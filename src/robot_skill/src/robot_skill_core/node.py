from __future__ import annotations

import asyncio
import math
import os
import traceback
from typing import Tuple

import rclpy
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node as RclpyNode
from sensor_msgs.msg import JointState

from robot_driver.action import GripperCommand, JointCommand, RobotCommand, SafetyPose
from robot_driver.msg import JointCommandOptions, JointCommandTarget
from robot_skill.action import SkillSequence
from robot_skill_core.skill_loader import SkillLibrary, SkillStep

JOINT_NAME_ORDER = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
SKILL_ACTION_TOPIC = '/robot_skill/action/skill_sequence'


class RobotSkillNode(RclpyNode):
    """Skill executor node exposing /robot_skill/action/skill_sequence."""

    def __init__(self) -> None:
        super().__init__('robot_skill')
        self._declare_parameters()

        skill_sets_dir = self.get_parameter('skill_sets_dir').get_parameter_value().string_value
        if not skill_sets_dir:
            package_share = get_package_share_directory('robot_skill')
            skill_sets_dir = os.path.join(package_share, 'skill_sets')
            self.get_logger().info(f'Using default skill_sets_dir={skill_sets_dir}')

        self._skill_library = SkillLibrary(skill_sets_dir)
        self._move_timeout_default = (
            self.get_parameter('move_timeout').get_parameter_value().double_value
        )
        self._gripper_timeout_default = (
            self.get_parameter('gripper_timeout').get_parameter_value().double_value
        )

        self._robot_client = ActionClient(self, RobotCommand, '/robot_driver/action/robot')
        self._joint_client = ActionClient(self, JointCommand, '/robot_driver/action/joint')
        self._gripper_client = ActionClient(self, GripperCommand, '/robot_driver/action/gripper')
        self._safety_client = ActionClient(self, SafetyPose, '/robot_driver/action/safety_pose')

        self._skill_server = ActionServer(
            self,
            SkillSequence,
            SKILL_ACTION_TOPIC,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            execute_callback=self._execute_skill_sequence,
        )

        self.get_logger().info('robot_skill action server ready')

    # ------------------------------------------------------------------ #
    # Parameter + goal handling
    # ------------------------------------------------------------------ #
    def _declare_parameters(self) -> None:
        package_share = get_package_share_directory('robot_skill')
        default_skill_dir = os.path.join(package_share, 'skill_sets')
        self.declare_parameter('skill_sets_dir', default_skill_dir)
        self.declare_parameter('move_timeout', 10.0)
        self.declare_parameter('gripper_timeout', 5.0)

    def _goal_callback(self, goal_request: SkillSequence.Goal) -> GoalResponse:
        if not goal_request.skill_id:
            self.get_logger().warning('Rejecting goal without skill_id')
            return GoalResponse.REJECT
        if not self._skill_library.has_skill(goal_request.skill_id):
            self.get_logger().warning(f"Skill '{goal_request.skill_id}' not found")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle: ServerGoalHandle) -> CancelResponse:
        self.get_logger().info(f'Cancel requested for skill {goal_handle.request.skill_id}')
        return CancelResponse.ACCEPT

    # ------------------------------------------------------------------ #
    # Execution pipeline
    # ------------------------------------------------------------------ #
    async def _execute_skill_sequence(
        self, goal_handle: ServerGoalHandle
    ) -> SkillSequence.Result:
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
        steps_executed = 0

        for index, step in enumerate(skill_def.steps):
            if goal_handle.is_cancel_requested:
                self.get_logger().warning(f'Skill {skill_id} canceled at step {step.id}')
                goal_handle.canceled()
                result = SkillSequence.Result()
                result.success = False
                result.error_code = 'CANCELED'
                result.message = f'Canceled at step {step.id}'
                result.steps_executed = steps_executed
                return result

            progress = float(index) / float(total_steps) if total_steps else 0.0
            feedback = SkillSequence.Feedback()
            feedback.step_index = index
            feedback.step_id = step.id
            feedback.step_desc = step.desc
            feedback.progress = progress
            feedback.detail = f'Executing {step.type}'
            goal_handle.publish_feedback(feedback)

            success, code, message = await self._execute_step(step)
            if not success:
                self.get_logger().error(
                    f"Step '{step.id}' failed (code={code}): {message}"
                )
                result = SkillSequence.Result()
                result.success = False
                result.error_code = code or 'FAILED'
                result.message = message
                result.steps_executed = steps_executed
                goal_handle.abort()
                return result

            steps_executed = index + 1

        result = SkillSequence.Result()
        result.success = True
        result.error_code = 'SUCCESS'
        result.message = 'Skill completed successfully'
        result.steps_executed = steps_executed
        goal_handle.succeed()
        return result

    async def _execute_step(self, step: SkillStep) -> Tuple[bool, str, str]:
        try:
            if step.type == 'safety_pose':
                return await self._execute_safety_pose(step)
            if step.type == 'cartesian_move':
                return await self._execute_cartesian_move(step)
            if step.type == 'gripper':
                return await self._execute_gripper(step)
            if step.type == 'joint_move':
                return await self._execute_joint_move(step)
            self.get_logger().error(f"Unknown step type '{step.type}'")
            return False, 'UNKNOWN_STEP', f'Unsupported step type {step.type}'
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().error(
                f'Exception during step {step.id}: {exc}\n{traceback.format_exc()}'
            )
            return False, 'EXCEPTION', str(exc)

    # ------------------------------------------------------------------ #
    # Step executors
    # ------------------------------------------------------------------ #
    async def _execute_safety_pose(self, step: SkillStep) -> Tuple[bool, str, str]:
        goal = SafetyPose.Goal()
        goal.enable_zero_gravity_after = bool(
            step.params.get('enable_zero_gravity_after', False)
        )
        return await self._send_goal(self._safety_client, goal, f'{step.id}/safety_pose')

    async def _execute_cartesian_move(self, step: SkillStep) -> Tuple[bool, str, str]:
        pose_dict = step.params.get('target_pose')
        if not pose_dict:
            raise ValueError(f"Step '{step.id}' missing target_pose params")
        frame_id = step.params.get('frame_id', 'base_link')
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(pose_dict['position']['x'])
        pose.pose.position.y = float(pose_dict['position']['y'])
        pose.pose.position.z = float(pose_dict['position']['z'])
        pose.pose.orientation.x = float(pose_dict['orientation']['x'])
        pose.pose.orientation.y = float(pose_dict['orientation']['y'])
        pose.pose.orientation.z = float(pose_dict['orientation']['z'])
        pose.pose.orientation.w = float(pose_dict['orientation']['w'])

        goal = RobotCommand.Goal()
        goal.target = pose
        goal.speed_scale = float(step.params.get('speed_scale', 0.3))
        timeout = float(step.params.get('timeout', self._move_timeout_default))
        goal.timeout = self._seconds_to_duration(timeout)
        goal.label = step.params.get('label', step.id)
        return await self._send_goal(self._robot_client, goal, f'{step.id}/cartesian')

    async def _execute_gripper(self, step: SkillStep) -> Tuple[bool, str, str]:
        goal = GripperCommand.Goal()
        goal.command = int(step.params.get('command', 0))
        goal.width_m = float(step.params.get('width_m', 0.0))
        goal.speed_scale = float(step.params.get('speed_scale', 0.5))
        goal.stop_on_contact = bool(step.params.get('stop_on_contact', True))
        timeout = float(step.params.get('timeout', self._gripper_timeout_default))
        goal.timeout = self._seconds_to_duration(timeout)
        goal.label = step.params.get('label', step.id)
        return await self._send_goal(self._gripper_client, goal, f'{step.id}/gripper')

    async def _execute_joint_move(self, step: SkillStep) -> Tuple[bool, str, str]:
        params = step.params
        positions = params.get('positions')
        if not positions:
            raise ValueError(f"Step '{step.id}' missing positions list")
        joint_name = params.get('joint_name')
        if joint_name is None:
            index = params.get('joint_index')
            if index is None or index < 0 or index >= len(JOINT_NAME_ORDER):
                raise ValueError(f"Invalid joint index in step '{step.id}'")
            joint_name = JOINT_NAME_ORDER[int(index)]
        speed_scale = float(params.get('speed_scale', 0.4))
        timeout = float(params.get('timeout', self._move_timeout_default))
        label = params.get('label', step.id)
        relative = bool(params.get('relative', False))

        for pos in positions:
            goal = JointCommand.Goal()
            joint_target = JointCommandTarget()
            joint_state = JointState()
            joint_state.name = [joint_name]
            joint_state.position = [float(pos)]
            joint_target.joint_state = joint_state
            joint_target.relative = relative
            joint_target.speed_scale = speed_scale
            goal.target = joint_target
            options = JointCommandOptions()
            options.timeout = self._seconds_to_duration(timeout)
            options.label = f'{label}_{pos:.2f}'
            goal.options = options

            success, code, message = await self._send_goal(
                self._joint_client, goal, f'{step.id}/joint'
            )
            if not success:
                return success, code, message
        return True, 'SUCCESS', ''

    # ------------------------------------------------------------------ #
    # Helpers
    # ------------------------------------------------------------------ #
    async def _send_goal(
        self, client: ActionClient, goal_msg, log_prefix: str
    ) -> Tuple[bool, str, str]:
        await self._wait_for_server(client, log_prefix)
        send_future = client.send_goal_async(goal_msg)
        goal_handle = await send_future
        if not goal_handle.accepted:
            return False, 'GOAL_REJECTED', f'{log_prefix} goal rejected'
        result_future = goal_handle.get_result_async()
        result = await result_future
        result_msg = result.result
        success = getattr(result_msg, 'success', True)
        result_code = getattr(result_msg, 'result_code', 'SUCCESS')
        last_error = getattr(result_msg, 'last_error', '')
        return success, result_code, last_error

    async def _wait_for_server(self, client: ActionClient, name: str) -> None:
        while not client.wait_for_server(timeout_sec=0.5):
            self.get_logger().debug(f'Waiting for action server {name}...')
            await asyncio.sleep(0.1)

    @staticmethod
    def _seconds_to_duration(seconds: float) -> Duration:
        seconds = max(0.0, float(seconds))
        duration = Duration()
        duration.sec = int(math.floor(seconds))
        duration.nanosec = int((seconds - duration.sec) * 1e9)
        return duration


def main() -> None:
    rclpy.init()
    node = RobotSkillNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('robot_skill shutdown requested by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


__all__ = ['RobotSkillNode', 'main']
