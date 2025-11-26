"""GripperAction: wraps JointCommand to expose /robot_driver/action/gripper."""
from __future__ import annotations

import math
import time
from collections import deque
from dataclasses import dataclass
from typing import Optional, Dict, Any, Deque

from rclpy.action import ActionServer, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from robot_driver.action import JointCommand, GripperCommand
from robot_driver.msg import JointContact

from driver.utils.logging_utils import get_logger, get_gripper_logger


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
        # 独立的夹爪调试日志，只写文件，不刷控制台。
        self._gripper_logger = get_gripper_logger(
            name='gripper_action_debug',
            log_dir='log',
            filename='gripper_action.log',
        )

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

        # 滑动窗口 + 差值启发式参数，用于 GRIP 模式下的“是否夹到物体”判定。
        # 这些参数只在 GripperAction 内部使用，不改变 JointAction 的基础数据结构。
        self._window_size = int(
            node.declare_parameter('gripper.contact_window_size', 3).value
        )
        # 自由闭合阶段：速度较大时的最小速度和允许的最大力矩
        self._free_velocity_min = float(
            node.declare_parameter('gripper.free_velocity_min', 0.2).value
        )
        # 力矩抬升与速度下降的阈值（相对于自由闭合阶段的基线）
        self._effort_jump_threshold = float(
            node.declare_parameter('gripper.effort_jump_threshold', 0.2).value
        )
        self._velocity_drop_threshold = float(
            node.declare_parameter('gripper.velocity_drop_threshold', 1.5).value
        )
        # 接触确认时的绝对阈值
        self._effort_min_for_contact = float(
            node.declare_parameter('gripper.effort_min_for_contact', 0.3).value
        )
        self._velocity_after_threshold = float(
            node.declare_parameter('gripper.velocity_after_threshold', 0.05).value
        )
        # 用于空抓/极限判定的宽度阈值（米）；未配置时由 no_object_margin_ratio 推导。
        self._p_close_threshold_m = float(
            node.declare_parameter('gripper.p_close_threshold_m', 0.0).value
        )
        if self._p_close_threshold_m <= 0.0:
            # 例如行程 0.094m * 0.1 ≈ 0.0094m
            self._p_close_threshold_m = max(
                0.0, min(self._cfg.no_object_margin_ratio, 1.0)
            ) * self._cfg.max_width_m

        # 滑动窗口：存储最近若干帧的 |effort| / |velocity| / width_m。
        # 窗口至少保留 2 个样本以便计算差值。
        self._history: Deque[Dict[str, float]] = deque(maxlen=max(self._window_size, 2))

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

    def _width_to_normalized(self, width_m: float) -> float:
        """Map physical width (m) to normalized gripper command [0,1]."""
        w = self._clamp_width(width_m)
        if self._cfg.max_width_m <= 0.0:
            return 0.0
        return max(0.0, min(1.0, w / self._cfg.max_width_m))

    def _normalized_to_width(self, normalized: float) -> float:
        """Map normalized gripper command [0,1] back to physical width (m)."""
        try:
            x = float(normalized)
        except Exception:
            return 0.0
        x = max(0.0, min(1.0, x))
        return x * self._cfg.max_width_m

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

        # Check JointCommand server availability without blocking.
        # NOTE: We use timeout_sec=0.0 for a non-blocking check because both
        # GripperAction and JointCommand live in the same node. A blocking
        # wait_for_server() would deadlock the executor since it cannot process
        # callbacks while this coroutine is blocked.
        if not self._joint_client.server_is_ready():
            # Server not immediately available; this is unexpected since both
            # actions are in the same node. Log and fail fast.
            msg = 'JointCommand action server unavailable for gripper (same-node deadlock avoided)'
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
        # JointCommand 侧使用归一化 gripper 命令空间 [0,1]。
        joint_goal.target.joint_state.position = [self._width_to_normalized(target_width)]
        joint_goal.target.relative = False
        joint_goal.target.speed_scale = float(speed_scale)
        joint_goal.options.timeout = goal.timeout
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
        joint_goal.target.joint_state.position = [self._width_to_normalized(target_width)]
        joint_goal.target.relative = False
        joint_goal.target.speed_scale = float(speed_scale)
        joint_goal.options.timeout = goal.timeout
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
        joint_goal.target.joint_state.position = [self._width_to_normalized(target_width)]
        joint_goal.target.relative = False
        joint_goal.target.speed_scale = float(speed_scale)
        joint_goal.options.timeout = goal.timeout
        joint_goal.options.label = goal.label or 'gripper_grip'

        # Shared state between feedback callback and awaiter.
        shared: Dict[str, Any] = {
            'goal_handle': None,      # type: Optional[Any]
            'object_attached': False,
            'contact_width': float('nan'),
            'contact_effort': 0.0,
            'cancel_requested': False,
            'last_width': 0.0,
        }

        fb_msg = GripperCommand.Feedback()
        fb_msg.phase = 'queued'
        fb_msg.completion_ratio = 0.0
        fb_msg.current_width_m = self._cfg.max_width_m
        fb_msg.in_contact = False
        goal_handle.publish_feedback(fb_msg)

        debug_contact = 'debug' in (goal.label or '').lower()
        if debug_contact:
            # 每次 debug GRIP 前，清空一次调试日志文件，避免无限增长。
            for handler in getattr(self._gripper_logger, 'handlers', []):
                base = getattr(handler, 'baseFilename', None)
                if base:
                    try:
                        with open(base, 'w'):
                            pass
                    except OSError:
                        pass
                    break

        def _on_joint_feedback(feedback_msg):
            """Translate JointCommand feedback into gripper feedback and detect grasp."""
            feedback = feedback_msg.feedback
            contact: Optional[JointContact] = None
            for c in feedback.contact_state:
                if c.name == 'gripper':
                    contact = c
                    break

            if contact is not None:
                # contact.position 对于 gripper 来说是归一化 [0,1]，需要映射为物理宽度。
                width = self._clamp_width(self._normalized_to_width(contact.position))
                # 更新滑动窗口历史，并在 debug 模式下记录原始样本
                self._update_history(width, contact.effort, contact.velocity, debug_contact)
            else:
                width = shared['last_width']

            shared['last_width'] = width

            # 基于滑动窗口检测接触模式
            in_contact = self._detect_contact_window(debug_contact)

            # 在 debug 模式下，把本次 feedback 的 contact_state 全量写入日志文件，便于离线分析。
            if debug_contact:
                self._gripper_logger.info(
                    'feedback: phase=%s completion=%.3f current_width=%.4f',
                    feedback.phase,
                    feedback.completion_ratio,
                    width,
                )
                for c in feedback.contact_state:
                    try:
                        self._gripper_logger.info(
                            '  contact: name=%s e=%.4f v=%.4f p=%.4f',
                            c.name,
                            float(c.effort),
                            float(c.velocity),
                            float(c.position),
                        )
                    except Exception:
                        self._gripper_logger.info('  contact: name=%s (parse error)', c.name)

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

        # Try to extract final width from final_state JointState（归一化 → 物理宽度）。
        try:
            state = jc_result.final_state
            if state.name and 'gripper' in state.name:
                idx = state.name.index('gripper')
                if idx < len(state.position):
                    final_width = self._normalized_to_width(state.position[idx])
        except Exception:
            final_width = 0.0

        # Extract contact info, if available (基础信息，仅用于记录，contact 判定由窗口逻辑驱动)。
        try:
            for c in jc_result.contact_state:
                if c.name == 'gripper':
                    contact_msg = c
                    break
        except Exception:
            contact_msg = None
            in_contact = False

        return final_width, in_contact, contact_msg

    # ------------------------------------------------------------------ local contact heuristic with sliding window
    def _update_history(self, width_m: float, effort: float, velocity: float, debug: bool = False) -> None:
        """Append a new gripper sample into the sliding window.

        如果 ``debug`` 为 True，会在 ``gripper_action.log`` 中记录原始样本，用于离线分析。
        """
        try:
            e = abs(float(effort))
            v = abs(float(velocity))
            w = float(width_m)
        except Exception:
            return
        sample = {
            't': float(time.monotonic()),
            'e': e,
            'v': v,
            'w': w,
        }
        self._history.append(sample)
        if debug:
            self._gripper_logger.info(
                'sample: e=%.4f v=%.4f w=%.4f (len=%d)', e, v, w, len(self._history)
            )

    def _detect_contact_window(self, debug: bool = False) -> bool:
        """Detect contact based on a short history of effort/velocity.

        启发式规则：
        - 在窗口前半段寻找“自由闭合”样本（速度较大）以确定基线 e_base / v_base；
        - 在最后几帧中寻找“力矩抬升 + 速度骤降”的拐点；
        - 要求当前帧力矩达到一定绝对值、速度足够小。
        """
        hist = list(self._history)
        n = len(hist)
        if n < self._window_size:
            if debug:
                self._gripper_logger.info('history too short (n=%d)', n)
            return False

        # 前半段（去掉最近 3 帧）作为“自由闭合”的候选
        cut = max(1, n - 3)
        free_candidates = [s for s in hist[:cut] if s['v'] > self._free_velocity_min]
        if len(free_candidates) < 2:
            if debug:
                self._gripper_logger.info(
                    'insufficient free samples (n=%d, free=%d)', n, len(free_candidates)
                )
            return False

        e_base = sum(s['e'] for s in free_candidates) / len(free_candidates)
        v_base = sum(s['v'] for s in free_candidates) / len(free_candidates)

        recent = hist[-3:]
        e_now_max = max(s['e'] for s in recent)
        v_now_min = min(s['v'] for s in recent)

        delta_eff = e_now_max - e_base
        delta_v = v_base - v_now_min

        current = hist[-1]
        e_t = current['e']
        v_t = current['v']

        ok = True
        reason = []
        if e_t < self._effort_min_for_contact:
            ok = False
            reason.append('e_t < effort_min')
        if v_t > self._velocity_after_threshold:
            ok = False
            reason.append('v_t > v_after')
        if delta_eff < self._effort_jump_threshold:
            ok = False
            reason.append('Δeff < jump')
        if delta_v < self._velocity_drop_threshold:
            ok = False
            reason.append('Δv < drop')

        if debug:
            self._gripper_logger.info(
                'eval: n=%d e_base=%.3f v_base=%.3f e_t=%.3f v_t=%.3f '
                'delta_eff=%.3f delta_v=%.3f ok=%s%s',
                n,
                e_base,
                v_base,
                e_t,
                v_t,
                delta_eff,
                delta_v,
                ok,
                f' ({",".join(reason)})' if not ok else '',
            )

        return ok
