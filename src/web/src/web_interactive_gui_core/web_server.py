"""WebServerManager - Flask/Socket.IO 服务管理"""

from __future__ import annotations

import logging
import threading
from pathlib import Path
from typing import TYPE_CHECKING, Callable, Optional

from flask import Flask, jsonify, render_template, request
from flask_cors import CORS
from flask_socketio import SocketIO

if TYPE_CHECKING:
    pass


class WebServerManager:
    """Flask/Socket.IO 服务管理"""

    def __init__(self, config: dict) -> None:
        """
        初始化 WebServerManager

        Args:
            config: 配置字典
                - host: 绑定地址（默认 0.0.0.0）
                - port: 端口（默认 5000）
        """
        self._host = config.get("host", "0.0.0.0")
        self._port = config.get("port", 5000)

        # 回调函数（由 set_callbacks 注入）
        self._on_segment: Optional[Callable[[float, float], None]] = None
        self._on_record: Optional[Callable[[str], None]] = None
        self._on_cancel: Optional[Callable[[], None]] = None
        self._get_status: Optional[Callable[[], dict]] = None
        self._get_health: Optional[Callable[[], dict]] = None

        # 线程状态
        self._thread: Optional[threading.Thread] = None
        self._ready = False
        self._ready_event = threading.Event()

        # 设置模板和静态文件路径
        template_dir = Path(__file__).parent / "templates"
        static_dir = Path(__file__).parent / "static"

        # 创建 Flask 应用
        self._app = Flask(
            __name__,
            template_folder=str(template_dir),
            static_folder=str(static_dir),
        )
        CORS(self._app)

        # 禁用 Flask 默认日志（避免刷屏）
        log = logging.getLogger("werkzeug")
        log.setLevel(logging.WARNING)

        # 创建 SocketIO
        self._socketio = SocketIO(
            self._app,
            cors_allowed_origins="*",
            async_mode="threading",
            logger=False,
            engineio_logger=False,
        )

        # 注册路由和事件
        self._register_routes()
        self._register_socketio_events()

    def set_callbacks(
        self,
        on_segment: Callable[[float, float], None],
        on_record: Callable[[str], None],
        on_cancel: Callable[[], None],
        get_status: Callable[[], dict],
        get_health: Callable[[], dict],
    ) -> None:
        """
        设置业务回调函数

        Args:
            on_segment: 分割请求回调 (x, y)
            on_record: 记录请求回调 (label)
            on_cancel: 取消请求回调
            get_status: 获取状态回调
            get_health: 获取健康状态回调
        """
        self._on_segment = on_segment
        self._on_record = on_record
        self._on_cancel = on_cancel
        self._get_status = get_status
        self._get_health = get_health

    def _register_routes(self) -> None:
        """注册 HTTP 路由"""

        @self._app.route("/")
        def index():
            """Web UI 页面"""
            return render_template("interactive_viewer.html")

        @self._app.route("/segment", methods=["POST"])
        def segment():
            """触发分割"""
            if self._on_segment is None:
                return jsonify({"status": "error", "error": "回调未设置"}), 500

            data = request.get_json() or {}
            x = data.get("x")
            y = data.get("y")

            if x is None or y is None:
                return jsonify({"status": "error", "error": "缺少 x 或 y 参数"}), 400

            try:
                self._on_segment(float(x), float(y))
                return jsonify({"status": "ok", "x": x, "y": y})
            except Exception as e:
                return jsonify({"status": "error", "error": str(e)}), 500

        @self._app.route("/click", methods=["POST"])
        def click():
            """兼容旧版 /click 路由（实际调用分割）"""
            if self._on_segment is None:
                return jsonify({"status": "error", "error": "回调未设置"}), 500

            data = request.get_json() or {}
            x = data.get("x")
            y = data.get("y")

            if x is None or y is None:
                return jsonify({"status": "error", "error": "缺少 x 或 y 参数"}), 400

            try:
                self._on_segment(float(x), float(y))
                return jsonify({"status": "ok", "x": x, "y": y})
            except Exception as e:
                return jsonify({"status": "error", "error": str(e)}), 500

        @self._app.route("/record", methods=["POST"])
        def record():
            """记录物体"""
            if self._on_record is None:
                return jsonify({"status": "error", "error": "回调未设置"}), 500

            data = request.get_json() or {}
            label = data.get("label", "")

            try:
                self._on_record(label)
                return jsonify({"status": "ok"})
            except Exception as e:
                return jsonify({"status": "error", "error": str(e)}), 500

        @self._app.route("/cancel", methods=["POST"])
        def cancel():
            """取消高亮"""
            if self._on_cancel is None:
                return jsonify({"status": "error", "error": "回调未设置"}), 500

            try:
                self._on_cancel()
                return jsonify({"status": "ok"})
            except Exception as e:
                return jsonify({"status": "error", "error": str(e)}), 500

        @self._app.route("/status", methods=["GET"])
        def status():
            """获取当前状态"""
            if self._get_status is None:
                return jsonify({"has_highlight": False, "processing": False})
            return jsonify(self._get_status())

        @self._app.route("/healthz", methods=["GET"])
        def healthz():
            """健康检查"""
            if self._get_health is None:
                return jsonify({"status": "ok", "ready": self._ready})
            return jsonify(self._get_health())

    def _register_socketio_events(self) -> None:
        """注册 Socket.IO 事件"""

        @self._socketio.on("connect")
        def on_connect():
            """客户端连接"""
            pass  # 可以在这里做连接统计

        @self._socketio.on("disconnect")
        def on_disconnect():
            """客户端断开"""
            pass  # 可以在这里做断开统计

    def start(self) -> None:
        """启动 Flask 后台线程"""
        if self._thread is not None and self._thread.is_alive():
            return  # 已经启动

        def run_server():
            """Flask 服务线程"""
            # 标记就绪
            self._ready = True
            self._ready_event.set()

            # 运行 Flask（阻塞）
            self._socketio.run(
                self._app,
                host=self._host,
                port=self._port,
                debug=False,
                use_reloader=False,
                allow_unsafe_werkzeug=True,
            )

        self._thread = threading.Thread(target=run_server, daemon=True)
        self._thread.start()

        # 等待 Flask 启动
        self._ready_event.wait(timeout=5.0)

    def is_ready(self) -> bool:
        """检查 Flask 线程是否启动"""
        return self._ready and self._thread is not None and self._thread.is_alive()

    def emit_image_update(
        self,
        image_b64: str,
        overlay_b64: Optional[str] = None,
        opacity: float = 1.0,
    ) -> None:
        """
        推送图像更新到浏览器

        Args:
            image_b64: Base64 编码的 JPEG 图像
            overlay_b64: Base64 编码的叠加层图像（可选）
            opacity: 叠加层透明度
        """
        if not self._ready:
            return

        data = {"image": image_b64}
        if overlay_b64:
            data["overlay"] = overlay_b64
            data["overlay_opacity"] = opacity

        self._socketio.emit("image_update", data)

    def emit_segment_result(self, success: bool, error: str = "") -> None:
        """
        推送分割结果通知

        Args:
            success: 是否成功
            error: 错误信息（失败时）
        """
        if not self._ready:
            return

        data = {"success": success}
        if not success and error:
            data["error"] = error

        self._socketio.emit("segment_result", data)

    def emit_record_result(
        self,
        success: bool,
        object_id: str = "",
        error: str = "",
    ) -> None:
        """
        推送记录结果通知

        Args:
            success: 是否成功
            object_id: 物体 ID（成功时）
            error: 错误信息（失败时）
        """
        if not self._ready:
            return

        data = {"success": success}
        if success and object_id:
            data["object_id"] = object_id
        if not success and error:
            data["error"] = error

        self._socketio.emit("record_result", data)
