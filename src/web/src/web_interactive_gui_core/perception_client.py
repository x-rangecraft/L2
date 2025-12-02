"""PerceptionClient - Perception Action 调用封装"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
from typing import TYPE_CHECKING, Optional

import numpy as np
from cv_bridge import CvBridge
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2

# perception Action 定义
from perception.action import ObjectRecord, ObjectTarget

if TYPE_CHECKING:
    pass


@dataclass
class SegmentResult:
    """分割结果"""

    success: bool
    error_message: str = ""
    visualization: Optional[np.ndarray] = None  # 可视化图（numpy BGR）
    cropped_image: Optional[Image] = None  # 裁剪图（ROS Image，供 record 使用）
    point_cloud: Optional[PointCloud2] = None  # 原始点云（ROS PointCloud2，供 grasp 使用）
    center_3d: Optional[tuple] = None  # 点云中心 (x, y, z)
    bbox_min: Optional[tuple] = None  # 3D 边界框最小点 (x, y, z)
    bbox_max: Optional[tuple] = None  # 3D 边界框最大点 (x, y, z)
    point_count: int = 0  # 有效点数量
    confidence: float = 0.0


@dataclass
class RecordResult:
    """记录结果"""

    success: bool
    error_message: str = ""
    object_id: str = ""


class PerceptionClient:
    """Perception Action 调用封装"""

    def __init__(self, node: Node, config: dict) -> None:
        """
        初始化 PerceptionClient

        Args:
            node: ROS 2 节点实例
            config: 配置字典
                - perception_action_prefix: Action 前缀（默认 /perception/action）
                - action_timeout: Action 超时秒数（默认 30.0）
        """
        self._node = node
        self._bridge = CvBridge()

        # 配置
        action_prefix = config.get("perception_action_prefix", "/perception/action")
        self._timeout = config.get("action_timeout", 30.0)

        # 创建 Action Client
        self._target_client = ActionClient(
            node,
            ObjectTarget,
            f"{action_prefix}/object_target",
        )
        self._record_client = ActionClient(
            node,
            ObjectRecord,
            f"{action_prefix}/object_record",
        )

        node.get_logger().info(
            f"[PerceptionClient] Action Client 已创建: "
            f"object_target={action_prefix}/object_target, "
            f"object_record={action_prefix}/object_record"
        )

    def is_ready(self) -> bool:
        """检查 Action Server 是否可用（非阻塞检查）"""
        # 检查 object_target server（主要功能）
        return self._target_client.server_is_ready()

    def wait_for_server(self, timeout_sec: float = 5.0) -> bool:
        """等待 Action Server 可用"""
        return self._target_client.wait_for_server(timeout_sec=timeout_sec)

    async def segment(
        self,
        color_image: Image,
        depth_image: Image,
        camera_info: CameraInfo,
        click_x: float,
        click_y: float,
    ) -> SegmentResult:
        """
        调用 object_target Action 进行分割

        Args:
            color_image: RGB 图像（ROS Image）
            depth_image: 深度图（ROS Image）
            camera_info: 相机内参
            click_x: 点击 X 坐标（像素）
            click_y: 点击 Y 坐标（像素）

        Returns:
            SegmentResult: 分割结果
        """
        # 检查 server 是否可用
        if not self._target_client.server_is_ready():
            return SegmentResult(
                success=False,
                error_message="Perception Action Server 不可用",
            )

        # 构建 Goal
        goal = ObjectTarget.Goal()
        goal.color_image = color_image
        goal.depth_image = depth_image
        goal.camera_info = camera_info
        goal.click_x = float(click_x)
        goal.click_y = float(click_y)

        try:
            # 发送 Goal
            self._node.get_logger().info(
                f"[PerceptionClient] 发送分割请求: click=({click_x}, {click_y})"
            )
            try:
                goal_handle = await asyncio.wait_for(
                    self._target_client.send_goal_async(goal),
                    timeout=self._timeout,
                )
            except asyncio.TimeoutError:
                return SegmentResult(
                    success=False,
                    error_message=f"分割请求发送超时 (> {self._timeout:.1f}s)",
                )

            if not goal_handle.accepted:
                return SegmentResult(
                    success=False,
                    error_message="分割请求被拒绝",
                )

            # 等待结果
            try:
                result_response = await asyncio.wait_for(
                    goal_handle.get_result_async(),
                    timeout=self._timeout,
                )
            except asyncio.TimeoutError:
                await goal_handle.cancel_goal_async()
                return SegmentResult(
                    success=False,
                    error_message=f"分割超时 (> {self._timeout:.1f}s)",
                )
            result = result_response.result

            if not result.success:
                return SegmentResult(
                    success=False,
                    error_message=result.error_message or "分割失败",
                )

            # 转换可视化图像
            visualization = None
            if result.visualization.data:
                try:
                    visualization = self._bridge.imgmsg_to_cv2(
                        result.visualization, desired_encoding="bgr8"
                    )
                except Exception as e:
                    self._node.get_logger().warning(
                        f"[PerceptionClient] 可视化图像转换失败: {e}"
                    )

            # 提取点云信息
            center_3d = None
            bbox_min = None
            bbox_max = None
            point_count = result.point_count
            
            if result.point_count > 0:
                center_3d = (
                    result.center_3d.x,
                    result.center_3d.y,
                    result.center_3d.z,
                )
                bbox_min = (
                    result.bbox_min.x,
                    result.bbox_min.y,
                    result.bbox_min.z,
                )
                bbox_max = (
                    result.bbox_max.x,
                    result.bbox_max.y,
                    result.bbox_max.z,
                )

            return SegmentResult(
                success=True,
                visualization=visualization,
                cropped_image=result.cropped_image,
                point_cloud=result.point_cloud if result.point_count > 0 else None,
                center_3d=center_3d,
                bbox_min=bbox_min,
                bbox_max=bbox_max,
                point_count=point_count,
                confidence=result.confidence,
            )

        except Exception as e:
            self._node.get_logger().error(f"[PerceptionClient] 分割异常: {e}")
            return SegmentResult(
                success=False,
                error_message=f"分割异常: {e}",
            )

    async def record(
        self,
        cropped_image: Image,
        label: str = "",
        description: str = "",
    ) -> RecordResult:
        """
        调用 object_record Action 记录物体

        Args:
            cropped_image: 裁剪后的目标图像（ROS Image）
            label: 语义标签（可选）
            description: 文本描述（可选）

        Returns:
            RecordResult: 记录结果
        """
        # 检查 server 是否可用
        if not self._record_client.server_is_ready():
            return RecordResult(
                success=False,
                error_message="Perception Record Action Server 不可用",
            )

        # 构建 Goal
        goal = ObjectRecord.Goal()
        goal.cropped_image = cropped_image
        goal.label = label
        goal.description = description

        try:
            # 发送 Goal
            self._node.get_logger().info(
                f"[PerceptionClient] 发送记录请求: label='{label}'"
            )
            try:
                goal_handle = await asyncio.wait_for(
                    self._record_client.send_goal_async(goal),
                    timeout=self._timeout,
                )
            except asyncio.TimeoutError:
                return RecordResult(
                    success=False,
                    error_message=f"记录请求发送超时 (> {self._timeout:.1f}s)",
                )

            if not goal_handle.accepted:
                return RecordResult(
                    success=False,
                    error_message="记录请求被拒绝",
                )

            # 等待结果
            try:
                result_response = await asyncio.wait_for(
                    goal_handle.get_result_async(),
                    timeout=self._timeout,
                )
            except asyncio.TimeoutError:
                await goal_handle.cancel_goal_async()
                return RecordResult(
                    success=False,
                    error_message=f"记录超时 (> {self._timeout:.1f}s)",
                )
            result = result_response.result

            if not result.success:
                return RecordResult(
                    success=False,
                    error_message=result.error_message or "记录失败",
                )

            return RecordResult(
                success=True,
                object_id=result.object_id,
            )

        except Exception as e:
            self._node.get_logger().error(f"[PerceptionClient] 记录异常: {e}")
            return RecordResult(
                success=False,
                error_message=f"记录异常: {e}",
            )
