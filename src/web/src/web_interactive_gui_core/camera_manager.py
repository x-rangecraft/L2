"""CameraManager - 相机数据订阅和缓存"""

from __future__ import annotations

import threading
from typing import TYPE_CHECKING, Optional

import numpy as np
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

if TYPE_CHECKING:
    from rclpy.subscription import Subscription


class CameraManager:
    """相机数据管理 - 订阅并缓存图像、深度图、相机内参"""

    def __init__(self, node: Node, config: dict) -> None:
        """
        初始化 CameraManager

        Args:
            node: ROS 2 节点实例
            config: 配置字典，包含话题名等
                - image_topic: RGB 图像话题
                - depth_topic: 深度图话题
                - camera_info_topic: 相机内参话题
        """
        self._node = node
        self._bridge = CvBridge()

        # 数据缓存
        self._image: Optional[np.ndarray] = None
        self._depth: Optional[np.ndarray] = None
        self._camera_info: Optional[CameraInfo] = None
        self._image_msg: Optional[Image] = None
        self._depth_msg: Optional[Image] = None

        # 线程锁
        self._lock = threading.Lock()

        # 话题名（从配置获取，带默认值）
        image_topic = config.get("image_topic", "/camera/color/image_raw")
        depth_topic = config.get(
            "depth_topic", "/camera/aligned_depth_to_color/image_raw"
        )
        camera_info_topic = config.get("camera_info_topic", "/camera/color/camera_info")

        # QoS 配置
        # 图像话题使用 BEST_EFFORT（与相机驱动匹配）
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # CameraInfo 使用 RELIABLE
        info_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # 创建订阅
        self._image_sub: Subscription = node.create_subscription(
            Image,
            image_topic,
            self._image_callback,
            image_qos,
        )
        self._depth_sub: Subscription = node.create_subscription(
            Image,
            depth_topic,
            self._depth_callback,
            image_qos,
        )
        self._camera_info_sub: Subscription = node.create_subscription(
            CameraInfo,
            camera_info_topic,
            self._camera_info_callback,
            info_qos,
        )

        node.get_logger().info(
            f"[CameraManager] 订阅已创建: "
            f"image={image_topic}, depth={depth_topic}, info={camera_info_topic}"
        )

    def _image_callback(self, msg: Image) -> None:
        """RGB 图像回调"""
        try:
            image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            with self._lock:
                self._image = image
                self._image_msg = msg
        except Exception as e:
            self._node.get_logger().error(f"[CameraManager] 图像转换失败: {e}")

    def _depth_callback(self, msg: Image) -> None:
        """深度图回调"""
        try:
            # 深度图通常是 16UC1 或 32FC1
            if msg.encoding == "16UC1":
                depth = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            elif msg.encoding == "32FC1":
                depth = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            else:
                depth = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

            with self._lock:
                self._depth = depth
                self._depth_msg = msg
        except Exception as e:
            self._node.get_logger().error(f"[CameraManager] 深度图转换失败: {e}")

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        """相机内参回调"""
        with self._lock:
            self._camera_info = msg

    def get_image(self) -> Optional[np.ndarray]:
        """获取最新 RGB 图像（numpy 格式，线程安全）"""
        with self._lock:
            if self._image is not None:
                return self._image.copy()
            return None

    def get_depth(self) -> Optional[np.ndarray]:
        """获取最新深度图（numpy 格式，线程安全）"""
        with self._lock:
            if self._depth is not None:
                return self._depth.copy()
            return None

    def get_camera_info(self) -> Optional[CameraInfo]:
        """获取相机内参"""
        with self._lock:
            return self._camera_info

    def get_image_msg(self) -> Optional[Image]:
        """获取最新 RGB 图像（ROS Image 格式，用于 Action 调用）"""
        with self._lock:
            return self._image_msg

    def get_depth_msg(self) -> Optional[Image]:
        """获取最新深度图（ROS Image 格式，用于 Action 调用）"""
        with self._lock:
            return self._depth_msg

    def is_ready(self) -> tuple[bool, str]:
        """
        检查数据是否就绪

        Returns:
            (ready, message): ready 为 True 表示数据就绪，message 为状态描述
        """
        with self._lock:
            missing = []
            if self._image is None:
                missing.append("RGB图像")
            if self._depth is None:
                missing.append("深度图")
            if self._camera_info is None:
                missing.append("相机内参")

            if missing:
                return False, f"等待数据: {', '.join(missing)}"
            return True, "数据就绪"
