"""GripperAction: wraps JointCommand to expose /robot_driver/action/gripper."""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Dict, Any

from rclpy.action import ActionServer, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from robot_driver.action import JointCommand, GripperCommand
from robot_driver.msg import JointContact

from driver.utils.logging_utils import get_logger


@dataclass
class _GripperConfig:
    max_width_m: float = 0.094
    open_width_default_m: float = 0.06
    no_object_margin_ratio: float = 0.1


class GripperAction:
    """Dedicated /robot_driver/action/gripper wrapper.

    This class owns the GripperCommand ActionServer and internally uses the
    existing JointCommand ActionServer (via the provided ActionClient) to
    actually drive the 'gripper' joint. It also interprets JointContact
    feedback to estimate whether an object has been grasped.
    """

    def __init__(
        self,
        node: Node,
        joint_client: ActionClient,
    ) -> None:
        self._node = node
        self._joint_client = joint_client
        self._logger = get_logger('gripper_action')

        # Read gripper-related parameters from the node. These are treated as
        # device-level configuration; Action callers only see width_m in meters.
        self._cfg = _GripperConfig(
            max_width_m=float(
                node.declare_parameter('gripper.max_width_m', 0.094).value
            ),
            open_width_default_m=float(
                node.declare_parameter('gripper.open_width_default_m', 0.06).value
            ),
            no_object_margin_ratio=float(
                node.declare_parameter('gripper.no_object_margin_ratio', 0.1).value
            ),
        )
        if self._cfg.max_width_m <= 0.0:
            self._logger.warning(
                'gripper.max_width_m <= 0 (%.4f); fallback to 0.094m',
                self._cfg.max_width_m,
            )
            self._cfg.max_width_m = 0.094

        # Optional local contact heuristic for internal GRIP logic; this is
        # not exposed via JointContact.in_contact any more, but still useful
        # for deciding when to stop squeezing.
        self._contact_effort_threshold = float(
            node.declare_parameter('gripper.contact_effort_threshold', 5.0).value
        )
        self._contact_velocity_eps = 1e-3

        self._cb_group = ReentrantCallbackGroup()
        self._action = ActionServer(
            node,
            GripperCommand,
            '/robot_driver/action/gripper',
            execute_callback=self._execute,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self._cb_group,
        )

    # ------------------------------------------------------------------ public API
    def destroy(self) -> None:
        try:
            self._action.destroy()
        except Exception:
            # Defensive: destroy should never raise during node shutdown.
            pass

    # ------------------------------------------------------------------ helpers
    def _clamp_width(self, width_m: float) -> float:
        width_m = float(width_m)
        if not math.isfinite(width_m):
            return 0.0
        return max(0.0, min(width_m, self._cfg.max_width_m))

    # ------------------------------------------------------------------ callbacks
    def _goal_cb(self, _goal_request) -> int:
        from rclpy.action import GoalResponse

        return GoalResponse.ACCEPT

    def _cancel_cb(self, _goal_handle) -> int:
        from rclpy.action import CancelResponse

        # GripperAction does not maintain its own motion thread; cancel simply
        # forwards to the underlying JointCommand goal (if any) via its client
        # cancel APIs in the execute coroutine.
        return CancelResponse.ACCEPT

    async def _execute(self, goal_handle) -> GripperCommand.Result:
        goal: GripperCommand.Goal = goal_handle.request
        cmd = int(goal.command)

        result = GripperCommand.Result()
        result.success = False
        result.result_code = ''
        result.last_error = ''
        result.object_attached = False
        result.in_contact = False
        result.final_width_m = 0.0
        result.contact_position = float('nan')
        result.contact_effort = 0.0

        label = goal.label or ''
        label_tag = f' [{label}]' if label else ''

        # Basic speed sanity.
        speed_scale = goal.speed_scale if math.isfinite(goal.speed_scale) else 1.0
        speed_scale = max(0.0, min(1.0, speed_scale))

        # Ensure JointCommand server is available.
        if not self._joint_client.wait_for_server(timeout_sec=2.0):
            msg = 'JointCommand action server unavailable for gripper'
            self._logger.error(msg + '%s', label_tag)
            result.success = False
            result.result_code = 'JOINT_SERVER_UNAVAILABLE'
            result.last_error = msg
            goal_handle.abort()
            return result

        try:
            if cmd == GripperCommand.Goal.COMMAND_OPEN:
                return await self._handle_open(goal_handle, goal, speed_scale, label_tag)
            if cmd == GripperCommand.Goal.COMMAND_MOVE:
                return await self._handle_move(goal_handle, goal, speed_scale, label_tag)
            if cmd == GripperCommand.Goal.COMMAND_GRIP:
                return await self._handle_grip(goal_handle, goal, speed_scale, label_tag)

            msg = f'Unknown gripper command: {cmd}'
            self._logger.error(msg + '%s', label_tag)
            result.success = False
            result.result_code = 'INVALID_COMMAND'
            result.last_error = msg
            goal_handle.abort()
            return result
        except Exception as exc:  # pragma: no cover - defensive guard
            msg = f'Gripper action execution failed: {exc}'
            self._logger.error(msg + '%s', label_tag)
            result.success = False
            result.result_code = 'EXCEPTION'
            result.last_error = msg
            goal_handle.abort()
            return result

    # ------------------------------------------------------------------ sub-handlers
    async def _handle_open(
        self,
        goal_handle,
        goal: GripperCommand.Goal,
        speed_scale: float,
        label_tag: str,
    ) -> GripperCommand.Result:
        """COMMAND_OPEN: move to a target width, ignore object-attached semantics."""
        result = GripperCommand.Result()

        target_width = goal.width_m if goal.width_m > 0.0 else self._cfg.open_width_default_m
        target_width = self._clamp_width(target_width)

        self._logger.info('Gripper OPEN to %.3fm%s', target_width, label_tag)

        # Build JointCommand goal that only touches the 'gripper' joint.
        joint_goal = JointCommand.Goal()
        joint_goal.target.joint_state.name = ['gripper']
        joint_goal.target.joint_state.position = [target_width]
        joint_goal.target.relative = False
        joint_goal.target.speed_scale = float(speed_scale)
        joint_goal.options.timeout = goal.timeout
        joint_goal.options.exit_zero_gravity = False
        joint_goal.options.label = goal.label or 'gripper_open'

        fb_msg = GripperCommand.Feedback()
        fb_msg.phase = 'queued'
        fb_msg.completion_ratio = 0.0
        fb_msg.current_width_m = target_width
        fb_msg.in_contact = False
        goal_handle.publish_feedback(fb_msg)

        # We do not need fine-grained feedback here; just send and wait.
        send_future = self._joint_client.send_goal_async(joint_goal)
        joint_goal_handle = await send_future
        if not joint_goal_handle.accepted:
            msg = 'JointCommand rejected gripper OPEN goal'
            self._logger.error(msg + '%s', label_tag)
            result.success = False
            result.result_code = 'JOINT_REJECTED'
            result.last_error = msg
            goal_handle.abort()
            return result

        joint_result_future = joint_goal_handle.get_result_async()
        joint_result = await joint_result_future
        jc_result: JointCommand.Result = joint_result.result

        # Extract final gripper width from JointCommand result.
        final_width, final_in_contact, contact = self._extract_gripper_from_result(jc_result)

        result.success = bool(jc_result.success)
        if not jc_result.success:
            result.result_code = jc_result.result_code or 'JOINT_FAILED'
            result.last_error = jc_result.last_error or ''
            result.object_attached = False
            result.in_contact = final_in_contact
        else:
            result.result_code = 'SUCCESS'
            result.last_error = ''
            result.object_attached = False
            result.in_contact = final_in_contact

        result.final_width_m = final_width
        if contact is not None:
            result.contact_position = float(contact.position)
            result.contact_effort = float(contact.effort)
        else:
            result.contact_position = float('nan')
            result.contact_effort = 0.0

        fb_msg.phase = 'complete'
        fb_msg.completion_ratio = 1.0
        fb_msg.current_width_m = final_width
        fb_msg.in_contact = final_in_contact
        goal_handle.publish_feedback(fb_msg)

        if result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    async def _handle_move(
        self,
        goal_handle,
        goal: GripperCommand.Goal,
        speed_scale: float,
        label_tag: str,
    ) -> GripperCommand.Result:
        """COMMAND_MOVE: move to an explicit target width, no grasp semantics."""
        result = GripperCommand.Result()

        if goal.width_m <= 0.0:
            msg = 'Gripper MOVE requires width_m > 0'
            self._logger.error(msg + '%s', label_tag)
            result.success = False
            result.result_code = 'INVALID_GOAL'
            result.last_error = msg
            result.object_attached = False
            result.in_contact = False
            result.final_width_m = 0.0
            result.contact_position = float('nan')
            result.contact_effort = 0.0
            goal_handle.abort()
            return result

        target_width = self._clamp_width(goal.width_m)
        self._logger.info('Gripper MOVE to %.3fm%s', target_width, label_tag)

        joint_goal = JointCommand.Goal()
        joint_goal.target.joint_state.name = ['gripper']
        joint_goal.target.joint_state.position = [target_width]
        joint_goal.target.relative = False
        joint_goal.target.speed_scale = float(speed_scale)
        joint_goal.options.timeout = goal.timeout
        joint_goal.options.exit_zero_gravity = False
        joint_goal.options.label = goal.label or 'gripper_move'

        fb_msg = GripperCommand.Feedback()
        fb_msg.phase = 'queued'
        fb_msg.completion_ratio = 0.0
        fb_msg.current_width_m = target_width
        fb_msg.in_contact = False
        goal_handle.publish_feedback(fb_msg)

        send_future = self._joint_client.send_goal_async(joint_goal)
        joint_goal_handle = await send_future
        if not joint_goal_handle.accepted:
            msg = 'JointCommand rejected gripper MOVE goal'
            self._logger.error(msg + '%s', label_tag)
            result.success = False
            result.result_code = 'JOINT_REJECTED'
            result.last_error = msg
            result.object_attached = False
            result.in_contact = False
            result.final_width_m = 0.0
            result.contact_position = float('nan')
            result.contact_effort = 0.0
            goal_handle.abort()
            return result

        joint_result = await joint_goal_handle.get_result_async()
        jc_result: JointCommand.Result = joint_result.result

        final_width, final_in_contact, contact = self._extract_gripper_from_result(jc_result)

        result.success = bool(jc_result.success)
        if not jc_result.success:
            result.result_code = jc_result.result_code or 'JOINT_FAILED'
            result.last_error = jc_result.last_error or ''
        else:
            result.result_code = 'SUCCESS'
            result.last_error = ''

        result.object_attached = False
        result.in_contact = final_in_contact
        result.final_width_m = final_width
        if contact is not None:
            result.contact_position = float(contact.position)
            result.contact_effort = float(contact.effort)
        else:
            result.contact_position = float('nan')
            result.contact_effort = 0.0

        fb_msg.phase = 'complete'
        fb_msg.completion_ratio = 1.0
        fb_msg.current_width_m = final_width
        fb_msg.in_contact = final_in_contact
        goal_handle.publish_feedback(fb_msg)

        if result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    async def _handle_grip(
        self,
        goal_handle,
        goal: GripperCommand.Goal,
        speed_scale: float,
        label_tag: str,
    ) -> GripperCommand.Result:
        """COMMAND_GRIP: close until grasped or fully closed, with object detection."""
        result = GripperCommand.Result()

        # Minimum width to close to; default 0.0 (fully closed).
        target_width = goal.width_m if goal.width_m > 0.0 else 0.0
        target_width = self._clamp_width(target_width)

        self._logger.info(
            'Gripper GRIP towards %.3fm (stop_on_contact=%s)%s',
            target_width,
            bool(goal.stop_on_contact),
            label_tag,
        )

        margin = max(0.0, min(self._cfg.no_object_margin_ratio, 1.0)) * self._cfg.max_width_m

        joint_goal = JointCommand.Goal()
        joint_goal.target.joint_state.name = ['gripper']
        joint_goal.target.joint_state.position = [target_width]
        joint_goal.target.relative = False
        joint_goal.target.speed_scale = float(speed_scale)
        joint_goal.options.timeout = goal.timeout
        joint_goal.options.exit_zero_gravity = False
        joint_goal.options.label = goal.label or 'gripper_grip'

        # Shared state between feedback callback and awaiter.
        shared: Dict[str, Any] = {
            'goal_handle': None,      # type: Optional[Any]
            'object_attached': False,
            'contact_width': float('nan'),
            'contact_effort': 0.0,
            'cancel_requested': False,
            'last_width': 0.0,
            'last_in_contact': False,
        }

        fb_msg = GripperCommand.Feedback()
        fb_msg.phase = 'queued'
        fb_msg.completion_ratio = 0.0
        fb_msg.current_width_m = self._cfg.max_width_m
        fb_msg.in_contact = False
        goal_handle.publish_feedback(fb_msg)

        def _on_joint_feedback(feedback_msg):
            """Translate JointCommand feedback into gripper feedback and detect grasp."""
            feedback = feedback_msg.feedback
            contact: Optional[JointContact] = None
            for c in feedback.contact_state:
                if c.name == 'gripper':
                    contact = c
                    break

            if contact is not None:
                width = self._clamp_width(contact.position)
                in_contact = self._detect_contact(contact)
            else:
                width = shared['last_width']
                in_contact = shared['last_in_contact']

            shared['last_width'] = width
            shared['last_in_contact'] = in_contact

            # Publish simplified gripper feedback.
            fb = GripperCommand.Feedback()
            fb.phase = 'contact' if in_contact else 'moving'
            fb.current_width_m = width
            fb.in_contact = in_contact
            # Approximate completion based on width缩小程度。
            # 0 = 起点，1 = 走到 target_width 或接触。
            try:
                start_width = self._cfg.max_width_m  # 粗略使用最大开口作为起点近似
                span = max(start_width - target_width, 1e-6)
                fb.completion_ratio = max(0.0, min(1.0, (start_width - width) / span))
            except Exception:
                fb.completion_ratio = 0.0
            goal_handle.publish_feedback(fb)

            # 仅在 GRIP 时，通过 in_contact + 宽度位置判断是否“夹到物体”。
            if not in_contact:
                return

            # 接触发生在距离完全闭合还有一定裕度的位置，认为夹到物体。
            if width > margin and not shared['object_attached']:
                shared['object_attached'] = True
                shared['contact_width'] = width
                shared['contact_effort'] = float(contact.effort) if contact is not None else 0.0
                self._logger.info(
                    'Gripper contact detected at width=%.3fm (margin=%.3fm)%s',
                    width,
                    margin,
                    label_tag,
                )
                if goal.stop_on_contact and not shared['cancel_requested']:
                    handle = shared.get('goal_handle')
                    if handle is not None:
                        shared['cancel_requested'] = True
                        try:
                            handle.cancel_goal_async()
                        except Exception:
                            # Best-effort cancel; fallback is letting JointCommand run to completion.
                            self._logger.warning(
                                'Failed to request cancel for JointCommand in gripper GRIP%s',
                                label_tag,
                            )

        send_future = self._joint_client.send_goal_async(
            joint_goal,
            feedback_callback=_on_joint_feedback,
        )
        joint_goal_handle = await send_future
        shared['goal_handle'] = joint_goal_handle

        if not joint_goal_handle.accepted:
            msg = 'JointCommand rejected gripper GRIP goal'
            self._logger.error(msg + '%s', label_tag)
            result.success = False
            result.result_code = 'JOINT_REJECTED'
            result.last_error = msg
            result.object_attached = False
            result.in_contact = shared['last_in_contact']
            result.final_width_m = shared['last_width']
            result.contact_position = float('nan')
            result.contact_effort = 0.0
            goal_handle.abort()
            return result

        joint_result = await joint_goal_handle.get_result_async()
        jc_result: JointCommand.Result = joint_result.result

        final_width, final_in_contact, contact = self._extract_gripper_from_result(jc_result)

        # 结合 JointCommand 结果与反馈中记录的接触信息，给出最终判定。
        object_attached = bool(shared['object_attached'])
        result.object_attached = object_attached
        result.in_contact = final_in_contact or object_attached
        result.final_width_m = final_width

        if object_attached:
            result.contact_position = shared['contact_width']
            result.contact_effort = shared['contact_effort']
        elif contact is not None:
            result.contact_position = float(contact.position)
            result.contact_effort = float(contact.effort)
        else:
            result.contact_position = float('nan')
            result.contact_effort = 0.0

        if not jc_result.success:
            # 如果 JointCommand 是因为我们请求的 cancel 而结束，但已经判定夹到物体，则视为成功。
            if object_attached and jc_result.result_code in ('CANCELED', 'PREEMPTED'):
                result.success = True
                result.result_code = 'OBJECT_GRASPED'
                result.last_error = ''
            else:
                result.success = False
                result.result_code = jc_result.result_code or 'JOINT_FAILED'
                result.last_error = jc_result.last_error or ''
        else:
            if object_attached:
                result.success = True
                result.result_code = 'OBJECT_GRASPED'
                result.last_error = ''
            else:
                # 未检测到“早期接触”，若最终宽度非常接近 0，则认为空抓。
                if final_width <= margin:
                    result.success = True
                    result.result_code = 'NO_OBJECT'
                    result.last_error = ''
                else:
                    result.success = True
                    result.result_code = 'SUCCESS'
                    result.last_error = ''

        fb = GripperCommand.Feedback()
        fb.phase = 'complete'
        fb.completion_ratio = 1.0
        fb.current_width_m = final_width
        fb.in_contact = result.in_contact
        goal_handle.publish_feedback(fb)

        if result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    # ------------------------------------------------------------------ extraction helpers
    def _extract_gripper_from_result(
        self,
        jc_result: JointCommand.Result,
    ) -> tuple[float, bool, Optional[JointContact]]:
        """Extract (width, contact_like, contact_msg) for the 'gripper' joint."""
        final_width = 0.0
        in_contact = False
        contact_msg: Optional[JointContact] = None

        # Try to extract final width from final_state JointState.
        try:
            state = jc_result.final_state
            if state.name and 'gripper' in state.name:
                idx = state.name.index('gripper')
                if idx < len(state.position):
                    final_width = float(state.position[idx])
        except Exception:
            final_width = 0.0

        # Extract contact info, if available.
        try:
            for c in jc_result.contact_state:
                if c.name == 'gripper':
                    contact_msg = c
                    in_contact = self._detect_contact(c)
                    break
        except Exception:
            contact_msg = None
            in_contact = False

        return final_width, in_contact, contact_msg

    # ------------------------------------------------------------------ local contact heuristic
    def _detect_contact(self, contact: JointContact) -> bool:
        """Local contact heuristic based on velocity/effort for GRIP logic."""
        try:
            vel = float(contact.velocity)
            eff = float(contact.effort)
        except Exception:
            return False
        return abs(vel) <= self._contact_velocity_eps and abs(eff) >= self._contact_effort_threshold
