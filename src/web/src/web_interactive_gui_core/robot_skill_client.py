"""RobotSkillClient - Robot Skill Action 调用封装"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
from typing import TYPE_CHECKING, Callable, List, Optional

from rclpy.action import ActionClient
from rclpy.node import Node

# robot_skill Action 定义
from robot_skill.action import GraspRecord, GraspRecordEasy

if TYPE_CHECKING:
    from perception.msg import GraspCandidate


@dataclass
class GraspResult:
    """抓取结果"""

    success: bool
    error_message: str = ""
    steps_executed: int = 0
    candidate_used: int = -1  # 实际使用的候选索引


class RobotSkillClient:
    """Robot Skill Action 调用封装"""

    def __init__(self, node: Node, config: dict) -> None:
        """
        初始化 RobotSkillClient

        Args:
            node: ROS 2 节点实例
            config: 配置字典
                - robot_skill_action_prefix: Action 前缀（默认 /robot_skill/action）
                - grasp_timeout: 抓取超时秒数（默认 120.0）
        """
        self._node = node

        # 配置
        action_prefix = config.get("robot_skill_action_prefix", "/robot_skill/action")
        self._timeout = config.get("grasp_timeout", 120.0)

        # 创建 Action Client
        self._grasp_client = ActionClient(
            node,
            GraspRecord,
            f"{action_prefix}/grasp_record",
        )

        self._grasp_easy_client = ActionClient(
            node,
            GraspRecordEasy,
            f"{action_prefix}/grasp_record_easy",
        )

        node.get_logger().info(
            f"[RobotSkillClient] Action Client 已创建: "
            f"grasp_record={action_prefix}/grasp_record, "
            f"grasp_record_easy={action_prefix}/grasp_record_easy"
        )

    def is_ready(self) -> bool:
        """检查 Action Server 是否可用（非阻塞检查）"""
        return self._grasp_client.server_is_ready() and self._grasp_easy_client.server_is_ready()

    def wait_for_server(self, timeout_sec: float = 5.0) -> bool:
        """等待 Action Server 可用"""
        ok1 = self._grasp_client.wait_for_server(timeout_sec=timeout_sec)
        ok2 = self._grasp_easy_client.wait_for_server(timeout_sec=timeout_sec)
        return ok1 and ok2

    async def grasp(
        self,
        grasp_candidates: List[GraspCandidate],
        source_frame_id: str = "",
        context_id: str = "",
        on_feedback: Optional[Callable[[str, int, str, float], None]] = None,
    ) -> GraspResult:
        """
        调用 grasp_record Action 执行抓取观察动作

        Args:
            grasp_candidates: 抓取候选数组（按置信度降序排列）
            source_frame_id: 候选位姿的源坐标系（若为 base_link 则跳过 TF 转换）
            context_id: 上下文标识符
            on_feedback: 进度回调函数 (phase, step_index, step_desc, progress)

        Returns:
            GraspResult: 抓取结果
        """
        # 检查 server 是否可用
        if not self._grasp_client.server_is_ready():
            return GraspResult(
                success=False,
                error_message="Robot Skill Action Server 不可用",
            )

        if not grasp_candidates:
            return GraspResult(
                success=False,
                error_message="抓取候选为空",
            )

        self._node.get_logger().info(
            f"[RobotSkillClient] 发送抓取请求: "
            f"candidates={len(grasp_candidates)}, source_frame={source_frame_id or 'default'}"
        )

        # 构建 Goal
        goal = GraspRecord.Goal()
        goal.grasp_candidates = grasp_candidates
        goal.source_frame_id = source_frame_id
        goal.context_id = context_id or ""

        try:
            # Feedback 回调
            def feedback_callback(feedback_msg):
                if on_feedback:
                    fb = feedback_msg.feedback
                    on_feedback(fb.phase, fb.step_index, fb.step_desc, fb.progress)

            try:
                goal_handle = await asyncio.wait_for(
                    self._grasp_client.send_goal_async(
                        goal, feedback_callback=feedback_callback
                    ),
                    timeout=30.0,  # 发送超时
                )
            except asyncio.TimeoutError:
                return GraspResult(
                    success=False,
                    error_message="抓取请求发送超时",
                )

            if not goal_handle.accepted:
                return GraspResult(
                    success=False,
                    error_message="抓取请求被拒绝",
                )

            self._node.get_logger().info("[RobotSkillClient] 抓取请求已接受，等待执行...")

            # 等待结果
            try:
                result_response = await asyncio.wait_for(
                    goal_handle.get_result_async(),
                    timeout=self._timeout,
                )
            except asyncio.TimeoutError:
                await goal_handle.cancel_goal_async()
                return GraspResult(
                    success=False,
                    error_message=f"抓取执行超时 (> {self._timeout:.1f}s)",
                )

            result = result_response.result

            if not result.success:
                return GraspResult(
                    success=False,
                    error_message=result.message or result.error_code or "抓取失败",
                    steps_executed=result.steps_executed,
                    candidate_used=result.candidate_used,
                )

            return GraspResult(
                success=True,
                steps_executed=result.steps_executed,
                candidate_used=result.candidate_used,
            )

        except Exception as e:
            self._node.get_logger().error(f"[RobotSkillClient] 抓取异常: {e}")
            return GraspResult(
                success=False,
                error_message=f"抓取异常: {e}",
            )

    async def grasp_easy(
        self,
        point_cloud,
        center_base,
        bbox_min,
        bbox_max,
        context_id: str = "",
        on_feedback: Optional[Callable[[str, int, str, float], None]] = None,
    ) -> GraspResult:
        """
        调用 grasp_record_easy Action 执行几何抓取动作。
        """
        if not self._grasp_easy_client.server_is_ready():
            return GraspResult(
                success=False,
                error_message="Robot Skill Easy Action Server 不可用",
            )

        from geometry_msgs.msg import Point, Vector3  # lazy import for type hints

        goal = GraspRecordEasy.Goal()
        goal.point_cloud = point_cloud
        goal.center_base = center_base if isinstance(center_base, Point) else center_base
        goal.bbox_min = bbox_min if isinstance(bbox_min, Vector3) else bbox_min
        goal.bbox_max = bbox_max if isinstance(bbox_max, Vector3) else bbox_max
        goal.context_id = context_id or ""

        try:
            def feedback_callback(feedback_msg):
                if on_feedback:
                    fb = feedback_msg.feedback
                    on_feedback(fb.phase, fb.step_index, fb.step_desc, fb.progress)

            try:
                goal_handle = await asyncio.wait_for(
                    self._grasp_easy_client.send_goal_async(
                        goal, feedback_callback=feedback_callback
                    ),
                    timeout=30.0,
                )
            except asyncio.TimeoutError:
                return GraspResult(
                    success=False,
                    error_message="简化抓取请求发送超时",
                )

            if not goal_handle.accepted:
                return GraspResult(
                    success=False,
                    error_message="简化抓取请求被拒绝",
                )

            self._node.get_logger().info(
                "[RobotSkillClient] 简化抓取请求已接受，等待执行..."
            )

            try:
                result_response = await asyncio.wait_for(
                    goal_handle.get_result_async(),
                    timeout=self._timeout,
                )
            except asyncio.TimeoutError:
                await goal_handle.cancel_goal_async()
                return GraspResult(
                    success=False,
                    error_message=f"简化抓取执行超时 (> {self._timeout:.1f}s)",
                )

            result = result_response.result

            if not result.success:
                return GraspResult(
                    success=False,
                    error_message=result.message or result.error_code or "简化抓取失败",
                    steps_executed=result.steps_executed,
                )

            return GraspResult(
                success=True,
                steps_executed=result.steps_executed,
            )

        except Exception as exc:  # pylint: disable=broad-except
            self._node.get_logger().error(f"[RobotSkillClient] 简化抓取异常: {exc}")
            return GraspResult(
                success=False,
                error_message=f"简化抓取异常: {exc}",
            )
