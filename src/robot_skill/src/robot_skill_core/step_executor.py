"""Step executor for robot skill sequences."""
from __future__ import annotations

import asyncio
import math
import traceback
from typing import Any, Callable, List, Optional, Tuple

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node as RclpyNode
from sensor_msgs.msg import JointState

from robot_driver.action import GripperCommand, JointCommand, RobotCommand, SafetyPose
from robot_driver.msg import JointCommandOptions, JointCommandTarget
from robot_skill_core.constants import (
    DEFAULT_ACTION_TIMEOUT_SEC,
    DEFAULT_SPEED_SCALE,
    JOINT_NAME_ORDER,
)
from robot_skill_core.skill_loader import SkillStep


class StepExecutor:
    """Executes SkillStep sequences by calling robot_driver actions."""

    def __init__(self, node: RclpyNode) -> None:
        self._node = node
        self._robot_client = ActionClient(node, RobotCommand, '/robot_driver/action/robot')
        self._joint_client = ActionClient(node, JointCommand, '/robot_driver/action/joint')
        self._gripper_client = ActionClient(node, GripperCommand, '/robot_driver/action/gripper')
        self._safety_client = ActionClient(node, SafetyPose, '/robot_driver/action/safety_pose')

    # ------------------------------------------------------------------ #
    # Public API
    # ------------------------------------------------------------------ #
    async def execute_steps(
        self,
        steps: List[SkillStep],
        on_progress: Optional[Callable[[int, SkillStep], None]] = None,
        check_cancel: Optional[Callable[[], bool]] = None,
    ) -> Tuple[bool, str, str, int]:
        """
        Execute a sequence of steps.

        Args:
            steps: List of SkillStep to execute
            on_progress: Optional callback(step_index, step) called before each step
            check_cancel: Optional callback() returning True if execution should cancel

        Returns:
            Tuple of (success, error_code, message, steps_executed)
        """
        steps_executed = 0

        for index, step in enumerate(steps):
            # Check for cancellation
            if check_cancel and check_cancel():
                self._node.get_logger().warning(f'Execution canceled at step {step.id}')
                return False, 'CANCELED', f'Canceled at step {step.id}', steps_executed

            # Report progress
            if on_progress:
                on_progress(index, step)

            # Execute the step
            success, code, message = await self.execute_step(step)
            if not success:
                self._node.get_logger().error(
                    f"Step '{step.id}' failed (code={code}): {message}"
                )
                return False, code or 'FAILED', message, steps_executed

            steps_executed = index + 1

        return True, 'SUCCESS', 'All steps completed successfully', steps_executed

    async def execute_step(self, step: SkillStep) -> Tuple[bool, str, str]:
        """
        Execute a single step.

        Args:
            step: The SkillStep to execute

        Returns:
            Tuple of (success, error_code, message)
        """
        try:
            if step.type == 'safety_pose':
                return await self._execute_safety_pose(step)
            if step.type == 'cartesian_move':
                return await self._execute_cartesian_move(step)
            if step.type == 'gripper':
                return await self._execute_gripper(step)
            if step.type == 'joint_move':
                return await self._execute_joint_move(step)
            self._node.get_logger().error(f"Unknown step type '{step.type}'")
            return False, 'UNKNOWN_STEP', f'Unsupported step type {step.type}'
        except Exception as exc:  # pylint: disable=broad-except
            self._node.get_logger().error(
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
        target_pose = step.params.get('target_pose')
        if target_pose is None:
            raise ValueError(f"Step '{step.id}' missing target_pose params")

        # Support both dict format (from YAML) and PoseStamped format (from dynamic planning)
        if isinstance(target_pose, PoseStamped):
            pose = target_pose
            pose.header.stamp = self._node.get_clock().now().to_msg()
        elif isinstance(target_pose, dict):
            frame_id = step.params.get('frame_id', 'base_link')
            pose = PoseStamped()
            pose.header.frame_id = frame_id
            pose.header.stamp = self._node.get_clock().now().to_msg()
            pose.pose.position.x = float(target_pose['position']['x'])
            pose.pose.position.y = float(target_pose['position']['y'])
            pose.pose.position.z = float(target_pose['position']['z'])
            pose.pose.orientation.x = float(target_pose['orientation']['x'])
            pose.pose.orientation.y = float(target_pose['orientation']['y'])
            pose.pose.orientation.z = float(target_pose['orientation']['z'])
            pose.pose.orientation.w = float(target_pose['orientation']['w'])
        else:
            raise ValueError(f"Step '{step.id}' has invalid target_pose type: {type(target_pose)}")

        goal = RobotCommand.Goal()
        goal.target = pose
        goal.speed_scale = float(step.params.get('speed_scale', DEFAULT_SPEED_SCALE))
        goal.timeout = self._seconds_to_duration(DEFAULT_ACTION_TIMEOUT_SEC)
        goal.label = step.params.get('label', step.id)
        return await self._send_goal(self._robot_client, goal, f'{step.id}/cartesian')

    async def _execute_gripper(self, step: SkillStep) -> Tuple[bool, str, str]:
        goal = GripperCommand.Goal()
        goal.command = int(step.params.get('command', 0))
        goal.width_m = float(step.params.get('width_m', 0.0))
        goal.speed_scale = float(step.params.get('speed_scale', DEFAULT_SPEED_SCALE))
        goal.stop_on_contact = bool(step.params.get('stop_on_contact', True))
        goal.timeout = self._seconds_to_duration(DEFAULT_ACTION_TIMEOUT_SEC)
        goal.label = step.params.get('label', step.id)
        return await self._send_goal(self._gripper_client, goal, f'{step.id}/gripper')

    async def _execute_joint_move(self, step: SkillStep) -> Tuple[bool, str, str]:
        params = step.params
        speed_scale = float(params.get('speed_scale', DEFAULT_SPEED_SCALE))
        timeout = DEFAULT_ACTION_TIMEOUT_SEC
        label = params.get('label', step.id)
        relative = bool(params.get('relative', False))

        # Mode 1: all_positions - set all 6 joints at once
        all_positions = params.get('all_positions')
        if all_positions is not None:
            if len(all_positions) != len(JOINT_NAME_ORDER):
                raise ValueError(
                    f"Step '{step.id}' all_positions must have {len(JOINT_NAME_ORDER)} values"
                )
            goal = JointCommand.Goal()
            joint_target = JointCommandTarget()
            joint_state = JointState()
            joint_state.name = list(JOINT_NAME_ORDER)
            joint_state.position = [float(p) for p in all_positions]
            joint_target.joint_state = joint_state
            joint_target.relative = relative
            joint_target.speed_scale = speed_scale
            goal.target = joint_target
            options = JointCommandOptions()
            options.timeout = self._seconds_to_duration(timeout)
            options.label = label
            goal.options = options
            return await self._send_goal(self._joint_client, goal, f'{step.id}/joint_all')

        # Mode 2: single joint with positions list (original behavior)
        positions = params.get('positions')
        if not positions:
            raise ValueError(f"Step '{step.id}' missing positions list or all_positions")
        joint_name = params.get('joint_name')
        if joint_name is None:
            index = params.get('joint_index')
            if index is None or index < 0 or index >= len(JOINT_NAME_ORDER):
                raise ValueError(f"Invalid joint index in step '{step.id}'")
            joint_name = JOINT_NAME_ORDER[int(index)]

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
        self, client: ActionClient, goal_msg: Any, log_prefix: str
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
            self._node.get_logger().debug(f'Waiting for action server {name}...')
            await asyncio.sleep(0.1)

    @staticmethod
    def _seconds_to_duration(seconds: float) -> Duration:
        seconds = max(0.0, float(seconds))
        duration = Duration()
        duration.sec = int(math.floor(seconds))
        duration.nanosec = int((seconds - duration.sec) * 1e9)
        return duration


__all__ = ['StepExecutor']

