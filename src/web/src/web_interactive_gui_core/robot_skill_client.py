"""RobotSkillClient - Robot Skill Action 调用封装"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
from typing import TYPE_CHECKING, Callable, List, Optional

import numpy as np
from geometry_msgs.msg import Point
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

# robot_skill Action 定义
from robot_skill.action import GraspRecord

if TYPE_CHECKING:
    pass


def pointcloud2_to_points(cloud: PointCloud2, max_points: int = 5000) -> List[Point]:
    """
    将 PointCloud2 转换为 Point 列表
    
    Args:
        cloud: ROS PointCloud2 消息
        max_points: 最大点数限制（避免传输过多数据）
    
    Returns:
        List[Point]: 点列表
    """
    import struct
    
    points = []
    
    # 解析点云字段
    field_names = [f.name for f in cloud.fields]
    if 'x' not in field_names or 'y' not in field_names or 'z' not in field_names:
        return points
    
    # 获取字段偏移
    x_offset = next(f.offset for f in cloud.fields if f.name == 'x')
    y_offset = next(f.offset for f in cloud.fields if f.name == 'y')
    z_offset = next(f.offset for f in cloud.fields if f.name == 'z')
    
    point_step = cloud.point_step
    data = cloud.data
    
    # 计算采样步长
    total_points = len(data) // point_step
    step = max(1, total_points // max_points)
    
    for i in range(0, total_points, step):
        offset = i * point_step
        try:
            x = struct.unpack_from('f', data, offset + x_offset)[0]
            y = struct.unpack_from('f', data, offset + y_offset)[0]
            z = struct.unpack_from('f', data, offset + z_offset)[0]
            
            # 过滤无效点
            if np.isfinite(x) and np.isfinite(y) and np.isfinite(z):
                p = Point()
                p.x = float(x)
                p.y = float(y)
                p.z = float(z)
                points.append(p)
        except struct.error:
            continue
    
    return points


@dataclass
class GraspResult:
    """抓取结果"""

    success: bool
    error_message: str = ""
    steps_executed: int = 0


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
                - max_gripper_width: 夹爪最大宽度（默认 0.08）
        """
        self._node = node

        # 配置
        action_prefix = config.get("robot_skill_action_prefix", "/robot_skill/action")
        self._timeout = config.get("grasp_timeout", 120.0)
        self._max_gripper_width = config.get("max_gripper_width", 0.08)

        # 创建 Action Client
        self._grasp_client = ActionClient(
            node,
            GraspRecord,
            f"{action_prefix}/grasp_record",
        )

        node.get_logger().info(
            f"[RobotSkillClient] Action Client 已创建: "
            f"grasp_record={action_prefix}/grasp_record"
        )

    def is_ready(self) -> bool:
        """检查 Action Server 是否可用（非阻塞检查）"""
        return self._grasp_client.server_is_ready()

    def wait_for_server(self, timeout_sec: float = 5.0) -> bool:
        """等待 Action Server 可用"""
        return self._grasp_client.wait_for_server(timeout_sec=timeout_sec)

    async def grasp(
        self,
        point_cloud: PointCloud2,
        context_id: str = "",
        max_gripper_width: Optional[float] = None,
        on_feedback: Optional[Callable[[str, int, str, float], None]] = None,
    ) -> GraspResult:
        """
        调用 grasp_record Action 执行抓取观察动作

        Args:
            point_cloud: 目标点云（ROS PointCloud2）
            context_id: 上下文标识符
            max_gripper_width: 夹爪最大宽度（可选，默认使用配置值）
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

        # 转换点云为 Point 列表
        boundary_points = pointcloud2_to_points(point_cloud)
        if not boundary_points:
            return GraspResult(
                success=False,
                error_message="点云转换失败或点云为空",
            )

        self._node.get_logger().info(
            f"[RobotSkillClient] 点云转换完成: {len(boundary_points)} 个点"
        )

        # 构建 Goal
        goal = GraspRecord.Goal()
        goal.boundary_points = boundary_points
        goal.max_gripper_width = max_gripper_width or self._max_gripper_width
        goal.context_id = context_id or ""

        try:
            # 发送 Goal
            self._node.get_logger().info(
                f"[RobotSkillClient] 发送抓取请求: "
                f"points={len(boundary_points)}, max_width={goal.max_gripper_width:.3f}m"
            )

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
                )

            return GraspResult(
                success=True,
                steps_executed=result.steps_executed,
            )

        except Exception as e:
            self._node.get_logger().error(f"[RobotSkillClient] 抓取异常: {e}")
            return GraspResult(
                success=False,
                error_message=f"抓取异常: {e}",
            )

