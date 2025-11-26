#!/usr/bin/env python3
"""Reusable ROS 2 node that exposes a web GUI for image streaming and click publishing."""

from __future__ import annotations

import base64
import queue
import threading
import time
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from flask import Flask, jsonify, render_template, request
from flask_cors import CORS
from flask_socketio import SocketIO, emit
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


class WebInteractiveGuiNode(Node):
    """ROS 2 node powering the reusable web GUI."""

    def __init__(self) -> None:
        super().__init__('web_interactive_gui')

        # Parameters --------------------------------------------------------
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 5000)
        self.declare_parameter('update_rate', 30.0)
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('overlay_topic', '/sam2/visualization')
        self.declare_parameter('overlay_enabled', True)
        self.declare_parameter('overlay_hold_duration', 0.0)
        self.declare_parameter('overlay_fade_duration', 1.0)
        self.declare_parameter('click_topic', '/click_point')
        self.declare_parameter('frame_id', 'camera_color_optical_frame')

        self.host: str = self.get_parameter('host').get_parameter_value().string_value
        self.port: int = int(self.get_parameter('port').value)
        self.update_rate: float = float(self.get_parameter('update_rate').value)
        self.overlay_hold_duration: float = float(
            self.get_parameter('overlay_hold_duration').value
        )
        self.overlay_fade_duration: float = float(
            self.get_parameter('overlay_fade_duration').value
        )
        self.image_topic: str = self.get_parameter('image_topic').value
        self.overlay_topic: str = self.get_parameter('overlay_topic').value
        self.overlay_enabled: bool = bool(self.get_parameter('overlay_enabled').value)
        self.click_topic: str = self.get_parameter('click_topic').value
        self.frame_id: str = self.get_parameter('frame_id').value

        # Internal state ----------------------------------------------------
        self.bridge = CvBridge()
        self.current_image: Optional[np.ndarray] = None
        self.current_overlay: Optional[np.ndarray] = None
        self.last_overlay_time: Optional[float] = None
        self.image_lock = threading.Lock()
        self.click_queue: queue.Queue = queue.Queue()

        # QoS profiles ------------------------------------------------------
        self.sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ROS entities ------------------------------------------------------
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self._image_callback,
            self.sensor_qos,
        )
        self.overlay_sub = None
        if self.overlay_enabled and self.overlay_topic:
            self.overlay_sub = self.create_subscription(
                Image,
                self.overlay_topic,
                self._overlay_callback,
                self.reliable_qos,
            )
        else:
            self.get_logger().info('Overlay disabled (参数 overlay_enabled=false 或 overlay_topic 为空)')

        self.click_pub = self.create_publisher(
            PointStamped,
            self.click_topic,
            self.reliable_qos,
        )

        # Flask / Socket.IO -------------------------------------------------
        package_dir = Path(__file__).resolve().parent
        template_dir = package_dir / 'templates'
        static_dir = package_dir / 'static'
        self.app = Flask(__name__, template_folder=str(template_dir), static_folder=str(static_dir))
        CORS(self.app)
        self.socketio = SocketIO(self.app, cors_allowed_origins='*', async_mode='threading')
        self._setup_routes()

        self.flask_thread = threading.Thread(target=self._run_flask, daemon=True)
        self.flask_thread.start()

        # Timers ------------------------------------------------------------
        update_period = 1.0 / max(1e-3, self.update_rate)
        self.image_timer = self.create_timer(update_period, self._broadcast_image)
        self.click_timer = self.create_timer(0.01, self._process_click_queue)

        self.get_logger().info(
            f'web_interactive_gui ready: host=http://{self.host}:{self.port}, '
            f'image_topic={self.image_topic}, overlay_topic={self.overlay_topic or "<disabled>"}'
        )

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------
    def _image_callback(self, msg: Image) -> None:
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        except Exception as exc:  # pragma: no cover
            self.get_logger().error(f'图像解码失败: {exc}')
            return

        with self.image_lock:
            self.current_image = cv_image

    def _overlay_callback(self, msg: Image) -> None:
        try:
            overlay = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            overlay = cv2.cvtColor(overlay, cv2.COLOR_BGR2RGB)
        except Exception as exc:  # pragma: no cover
            self.get_logger().error(f'Overlay 解码失败: {exc}')
            return

        with self.image_lock:
            self.current_overlay = overlay
            self.last_overlay_time = time.time()

    # ------------------------------------------------------------------
    # Timers
    # ------------------------------------------------------------------
    def _broadcast_image(self) -> None:
        with self.image_lock:
            if self.current_image is None:
                return
            base_image = self.current_image.copy()
            overlay_image = self.current_overlay.copy() if self.current_overlay is not None else None
            overlay_time = self.last_overlay_time

        payload = {}
        try:
            _, base_buffer = cv2.imencode('.jpg', cv2.cvtColor(base_image, cv2.COLOR_RGB2BGR))
        except Exception as exc:
            self.get_logger().error(f'基准图像编码失败: {exc}')
            return
        payload['image'] = base64.b64encode(base_buffer).decode('utf-8')

        overlay_opacity = 0.0
        if overlay_image is not None and overlay_time is not None:
            elapsed = time.time() - overlay_time
            hold = max(0.0, self.overlay_hold_duration)
            fade = max(0.0, self.overlay_fade_duration)
            if hold == 0.0 and fade == 0.0:
                overlay_image = None
                with self.image_lock:
                    self.current_overlay = None
                    self.last_overlay_time = None
            else:
                max_visible = hold + (fade if fade > 0.0 else 0.0)
                if elapsed <= max_visible or (fade == 0.0 and elapsed <= hold):
                    overlay_opacity = 1.0
                    if elapsed > hold and fade > 0.0:
                        fade_elapsed = max(0.0, elapsed - hold)
                        fade_ratio = min(1.0, fade_elapsed / fade)
                        overlay_opacity = max(0.0, 1.0 - fade_ratio)
                else:
                    overlay_image = None
                    with self.image_lock:
                        self.current_overlay = None
                        self.last_overlay_time = None

        if overlay_image is not None and overlay_opacity > 0.0:
            try:
                _, overlay_buffer = cv2.imencode(
                    '.jpg', cv2.cvtColor(overlay_image, cv2.COLOR_RGB2BGR)
                )
                payload['overlay'] = base64.b64encode(overlay_buffer).decode('utf-8')
                payload['overlay_opacity'] = overlay_opacity
            except Exception as exc:
                self.get_logger().error(f'Overlay 编码失败: {exc}')

        self.socketio.emit('image_update', payload)

    def _process_click_queue(self) -> None:
        try:
            while not self.click_queue.empty():
                x, y = self.click_queue.get_nowait()
                msg = PointStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = self.frame_id
                msg.point.x = float(x)
                msg.point.y = float(y)
                msg.point.z = 0.0

                with self.image_lock:
                    image_shape = self.current_image.shape if self.current_image is not None else None
                if image_shape is None:
                    self.get_logger().warn('收到点击但当前没有图像帧，仍然发布坐标')
                else:
                    h, w = image_shape[:2]
                    self.get_logger().info(
                        f'发布点击: ({x:.0f}, {y:.0f}) on {w}x{h}, topic={self.click_topic}'
                    )

                self.click_pub.publish(msg)

                subscribers = self.click_pub.get_subscription_count()
                if subscribers == 0:
                    self.get_logger().warn('当前 /click_point 无订阅者，请确认下游节点是否运行')
        except queue.Empty:  # pragma: no cover
            return

    # ------------------------------------------------------------------
    # Flask helpers
    # ------------------------------------------------------------------
    def _setup_routes(self) -> None:
        @self.app.route('/')
        def index():  # pragma: no cover - exercised via runtime
            return render_template('interactive_viewer.html')

        @self.app.route('/healthz')
        def healthz():  # pragma: no cover
            status = 'ok' if self.current_image is not None else 'waiting_image'
            return jsonify({'status': status, 'image_topic': self.image_topic})

        @self.app.route('/click', methods=['POST'])
        def handle_click():  # pragma: no cover
            data = request.get_json(force=True) or {}
            x = float(data.get('x', 0))
            y = float(data.get('y', 0))
            self.click_queue.put((x, y))
            return jsonify({'status': 'queued', 'x': x, 'y': y})

        @self.socketio.on('connect')
        def on_connect():  # pragma: no cover
            self.get_logger().info('Web 客户端已连接')
            emit('connection_response', {'data': 'connected'})

        @self.socketio.on('disconnect')
        def on_disconnect():  # pragma: no cover
            self.get_logger().info('Web 客户端已断开')

    def _run_flask(self) -> None:
        self.socketio.run(
            self.app,
            host=self.host,
            port=self.port,
            debug=False,
            use_reloader=False,
            allow_unsafe_werkzeug=True,
        )


def main(args=None):
    rclpy.init(args=args)
    node = WebInteractiveGuiNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:  # pragma: no cover
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


__all__ = ['WebInteractiveGuiNode', 'main']
