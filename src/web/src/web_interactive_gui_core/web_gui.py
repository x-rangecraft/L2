"""WebInteractiveGui - Web 交互界面节点（协调层）"""

from __future__ import annotations

import asyncio
import base64
import threading
from enum import Enum
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from .camera_manager import CameraManager
from .perception_client import PerceptionClient, SegmentResult
from .web_server import WebServerManager


class State(Enum):
    """节点状态"""

    IDLE = "idle"  # 空闲，无高亮
    SEGMENTING = "segmenting"  # 分割处理中
    HIGHLIGHTED = "highlighted"  # 有分割结果，显示高亮
    RECORDING = "recording"  # 记录处理中


class WebInteractiveGui(Node):
    """Web 交互界面节点 - 协调层"""

    def __init__(self) -> None:
        super().__init__("web_interactive_gui")

        self.get_logger().info("[web_gui] 节点初始化中...")

        # 1. 加载参数
        config = self._load_parameters()

        # 状态管理
        self._state = State.IDLE
        self._state_lock = threading.Lock()

        # 分割结果缓存
        self._visualization: Optional[np.ndarray] = None  # 高亮图（BGR numpy）
        self._cropped_image: Optional[Image] = None  # 裁剪图（ROS Image）
        self._cache_lock = threading.Lock()

        # 2. 创建模块
        self._camera = CameraManager(self, config)
        self._perception = PerceptionClient(self, config)
        self._web_server = WebServerManager(config)

        # 3. 注入回调
        self._web_server.set_callbacks(
            on_segment=self._handle_segment,
            on_record=self._handle_record,
            on_cancel=self._handle_cancel,
            get_status=self._get_status,
            get_health=self._get_health,
        )

        # 4. 启动 Flask 后台线程
        self._web_server.start()

        # 5. 等待 Flask 就绪
        if self._web_server.is_ready():
            self.get_logger().info(
                f"[web_gui] Flask 服务已启动: http://{config['host']}:{config['port']}"
            )
        else:
            self.get_logger().warning("[web_gui] Flask 服务启动超时")

        # 6. 启动图像推送定时器
        update_rate = config.get("update_rate", 30.0)
        timer_period = 1.0 / update_rate
        self._broadcast_timer = self.create_timer(timer_period, self._broadcast_image)

        # 7. 输出启动成功日志（启动脚本监听此标记）
        self.get_logger().info("[web_gui] 启动完成")

    def _load_parameters(self) -> dict:
        """加载 ROS 参数"""
        # 声明参数（带默认值）
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 5000)
        self.declare_parameter("update_rate", 30.0)
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter(
            "depth_topic", "/camera/aligned_depth_to_color/image_raw"
        )
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("perception_action_prefix", "/perception/action")
        self.declare_parameter("action_timeout", 30.0)

        # 读取参数值
        config = {
            "host": self.get_parameter("host").get_parameter_value().string_value,
            "port": self.get_parameter("port").get_parameter_value().integer_value,
            "update_rate": self.get_parameter("update_rate")
            .get_parameter_value()
            .double_value,
            "image_topic": self.get_parameter("image_topic")
            .get_parameter_value()
            .string_value,
            "depth_topic": self.get_parameter("depth_topic")
            .get_parameter_value()
            .string_value,
            "camera_info_topic": self.get_parameter("camera_info_topic")
            .get_parameter_value()
            .string_value,
            "perception_action_prefix": self.get_parameter("perception_action_prefix")
            .get_parameter_value()
            .string_value,
            "action_timeout": self.get_parameter("action_timeout")
            .get_parameter_value()
            .double_value,
        }

        self.get_logger().info(f"[web_gui] 参数加载完成: {config}")
        return config

    def _handle_segment(self, x: float, y: float) -> None:
        """
        处理分割请求（Flask 线程调用，同步）

        在后台线程中异步执行 Action 调用
        """
        # 检查状态
        with self._state_lock:
            if self._state == State.SEGMENTING:
                self.get_logger().warning("[web_gui] 正在分割中，忽略新请求")
                return
            if self._state == State.RECORDING:
                self.get_logger().warning("[web_gui] 正在记录中，忽略分割请求")
                return
            self._state = State.SEGMENTING

        self.get_logger().info(f"[web_gui] 开始分割: click=({x}, {y})")

        # 清除旧的高亮
        self._clear_cache()

        # 在后台线程执行异步 Action
        thread = threading.Thread(
            target=self._run_segment_async,
            args=(x, y),
            daemon=True,
        )
        thread.start()

    def _run_segment_async(self, x: float, y: float) -> None:
        """在后台线程中执行分割 Action"""
        # 创建新的事件循环
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)

        try:
            result = loop.run_until_complete(self._do_segment(x, y))

            if result.success:
                with self._cache_lock:
                    self._visualization = result.visualization
                    self._cropped_image = result.cropped_image

                with self._state_lock:
                    self._state = State.HIGHLIGHTED

                self.get_logger().info(
                    f"[web_gui] 分割成功: confidence={result.confidence:.2f}"
                )
                self._web_server.emit_segment_result(success=True)
            else:
                with self._state_lock:
                    self._state = State.IDLE

                self.get_logger().warning(f"[web_gui] 分割失败: {result.error_message}")
                self._web_server.emit_segment_result(
                    success=False, error=result.error_message
                )

        except Exception as e:
            with self._state_lock:
                self._state = State.IDLE

            self.get_logger().error(f"[web_gui] 分割异常: {e}")
            self._web_server.emit_segment_result(success=False, error=str(e))

        finally:
            loop.close()

    async def _do_segment(self, x: float, y: float):
        """执行分割 Action（异步）"""
        # 获取相机数据
        color_image = self._camera.get_image_msg()
        depth_image = self._camera.get_depth_msg()
        camera_info = self._camera.get_camera_info()

        if color_image is None or depth_image is None or camera_info is None:
            ready, msg = self._camera.is_ready()
            return SegmentResult(success=False, error_message=f"相机数据未就绪: {msg}")

        # 调用 Perception Action
        return await self._perception.segment(
            color_image=color_image,
            depth_image=depth_image,
            camera_info=camera_info,
            click_x=x,
            click_y=y,
        )

    def _handle_record(self, label: str) -> None:
        """
        处理记录请求（Flask 线程调用，同步）

        在后台线程中异步执行 Action 调用
        """
        # 检查状态
        with self._state_lock:
            if self._state != State.HIGHLIGHTED:
                self.get_logger().warning(
                    f"[web_gui] 当前状态 {self._state.value}，无法记录"
                )
                self._web_server.emit_record_result(
                    success=False, error="没有可记录的分割结果"
                )
                return
            self._state = State.RECORDING

        # 检查是否有裁剪图
        with self._cache_lock:
            if self._cropped_image is None:
                with self._state_lock:
                    self._state = State.HIGHLIGHTED
                self._web_server.emit_record_result(
                    success=False, error="裁剪图像缺失"
                )
                return
            cropped_image = self._cropped_image

        self.get_logger().info(f"[web_gui] 开始记录: label='{label}'")

        # 在后台线程执行异步 Action
        thread = threading.Thread(
            target=self._run_record_async,
            args=(cropped_image, label),
            daemon=True,
        )
        thread.start()

    def _run_record_async(self, cropped_image: Image, label: str) -> None:
        """在后台线程中执行记录 Action"""
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)

        try:
            result = loop.run_until_complete(
                self._perception.record(cropped_image=cropped_image, label=label)
            )

            if result.success:
                # 记录成功，清除高亮
                self._clear_cache()

                with self._state_lock:
                    self._state = State.IDLE

                self.get_logger().info(
                    f"[web_gui] 记录成功: object_id={result.object_id}"
                )
                self._web_server.emit_record_result(
                    success=True, object_id=result.object_id
                )
            else:
                # 记录失败，恢复到高亮状态
                with self._state_lock:
                    self._state = State.HIGHLIGHTED

                self.get_logger().warning(f"[web_gui] 记录失败: {result.error_message}")
                self._web_server.emit_record_result(
                    success=False, error=result.error_message
                )

        except Exception as e:
            with self._state_lock:
                self._state = State.HIGHLIGHTED

            self.get_logger().error(f"[web_gui] 记录异常: {e}")
            self._web_server.emit_record_result(success=False, error=str(e))

        finally:
            loop.close()

    def _handle_cancel(self) -> None:
        """处理取消请求"""
        with self._state_lock:
            if self._state == State.SEGMENTING or self._state == State.RECORDING:
                self.get_logger().warning(
                    f"[web_gui] 当前状态 {self._state.value}，无法取消"
                )
                return

        self._clear_cache()

        with self._state_lock:
            self._state = State.IDLE

        self.get_logger().info("[web_gui] 已取消高亮")

    def _get_status(self) -> dict:
        """获取当前状态"""
        with self._state_lock:
            state = self._state

        with self._cache_lock:
            has_highlight = self._visualization is not None

        return {
            "state": state.value,
            "has_highlight": has_highlight,
            "processing": state in (State.SEGMENTING, State.RECORDING),
        }

    def _get_health(self) -> dict:
        """获取健康状态"""
        camera_ready, camera_msg = self._camera.is_ready()
        perception_ready = self._perception.is_ready()
        web_ready = self._web_server.is_ready()

        all_ready = web_ready  # 最低要求：Web 服务可用

        return {
            "status": "ok" if all_ready else "degraded",
            "ready": all_ready,
            "camera": {"ready": camera_ready, "message": camera_msg},
            "perception": {"ready": perception_ready},
            "web": {"ready": web_ready},
        }

    def _broadcast_image(self) -> None:
        """定时推送图像（Timer 回调，30Hz）"""
        # 获取当前图像
        image = self._camera.get_image()
        if image is None:
            return  # 没有图像，跳过

        # 编码为 JPEG Base64
        try:
            _, buffer = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 80])
            image_b64 = base64.b64encode(buffer).decode("utf-8")
        except Exception as e:
            self.get_logger().error(f"[web_gui] 图像编码失败: {e}")
            return

        # 检查是否有高亮叠加
        overlay_b64 = None
        with self._cache_lock:
            if self._visualization is not None:
                try:
                    _, overlay_buffer = cv2.imencode(
                        ".jpg", self._visualization, [cv2.IMWRITE_JPEG_QUALITY, 80]
                    )
                    overlay_b64 = base64.b64encode(overlay_buffer).decode("utf-8")
                except Exception:
                    pass

        # 推送到浏览器
        self._web_server.emit_image_update(
            image_b64=image_b64,
            overlay_b64=overlay_b64,
            opacity=0.6 if overlay_b64 else 0.0,
        )

    def _clear_cache(self) -> None:
        """清除分割结果缓存"""
        with self._cache_lock:
            self._visualization = None
            self._cropped_image = None


def main(args=None):
    """节点入口函数"""
    rclpy.init(args=args)
    node = WebInteractiveGui()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


__all__ = ["WebInteractiveGui", "State", "main"]
