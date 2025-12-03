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
        self._on_grasp: Optional[Callable[[], None]] = None
        self._on_grasp_pose: Optional[Callable[[], None]] = None
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
        on_grasp: Callable[[], None],
        on_grasp_pose: Callable[[], None],
        on_cancel: Callable[[], None],
        get_status: Callable[[], dict],
        get_health: Callable[[], dict],
    ) -> None:
        """
        设置业务回调函数

        Args:
            on_segment: 分割请求回调 (x, y)
            on_grasp: 记录请求回调（调用 grasp_record action）
            on_grasp_pose: 姿态测算请求回调
            on_cancel: 取消请求回调
            get_status: 获取状态回调
            get_health: 获取健康状态回调
        """
        self._on_segment = on_segment
        self._on_grasp = on_grasp
        self._on_grasp_pose = on_grasp_pose
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

        @self._app.route("/grasp", methods=["POST"])
        def grasp():
            """执行抓取动作"""
            if self._on_grasp is None:
                return jsonify({"status": "error", "error": "回调未设置"}), 500

            try:
                self._on_grasp()
                return jsonify({"status": "ok"})
            except Exception as e:
                return jsonify({"status": "error", "error": str(e)}), 500

        @self._app.route("/grasp_pose", methods=["POST"])
        def grasp_pose():
            """姿态测算（调用 Grasp Service）"""
            if self._on_grasp_pose is None:
                return jsonify({"status": "error", "error": "回调未设置"}), 500

            try:
                self._on_grasp_pose()
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

    def emit_segment_result(
        self,
        success: bool,
        error: str = "",
        info_3d: dict = None,
    ) -> None:
        """
        推送分割结果通知

        Args:
            success: 是否成功
            error: 错误信息（失败时）
            info_3d: 3D 信息字典，包含 center_3d, bbox_min, bbox_max, point_count, confidence, center_3d_base
        """
        if not self._ready:
            return

        data = {"success": success}
        if not success and error:
            data["error"] = error
        
        # 添加 3D 信息
        if success and info_3d:
            if info_3d.get("center_3d"):
                data["center_3d"] = {
                    "x": round(info_3d["center_3d"][0], 4),
                    "y": round(info_3d["center_3d"][1], 4),
                    "z": round(info_3d["center_3d"][2], 4),
                }
            if info_3d.get("bbox_min"):
                data["bbox_min"] = {
                    "x": round(info_3d["bbox_min"][0], 4),
                    "y": round(info_3d["bbox_min"][1], 4),
                    "z": round(info_3d["bbox_min"][2], 4),
                }
            if info_3d.get("bbox_max"):
                data["bbox_max"] = {
                    "x": round(info_3d["bbox_max"][0], 4),
                    "y": round(info_3d["bbox_max"][1], 4),
                    "z": round(info_3d["bbox_max"][2], 4),
                }
            # 转换后的 base_link 坐标
            if info_3d.get("center_3d_base"):
                data["center_3d_base"] = {
                    "x": round(info_3d["center_3d_base"][0], 4),
                    "y": round(info_3d["center_3d_base"][1], 4),
                    "z": round(info_3d["center_3d_base"][2], 4),
                }
            data["point_count"] = info_3d.get("point_count", 0)
            data["confidence"] = round(info_3d.get("confidence", 0), 4)

        self._socketio.emit("segment_result", data)

    def emit_grasp_feedback(
        self,
        phase: str,
        step_index: int,
        step_desc: str,
        progress: float,
    ) -> None:
        """
        推送抓取进度反馈

        Args:
            phase: 当前阶段 (planning/executing)
            step_index: 当前步骤索引
            step_desc: 步骤描述
            progress: 进度 (0.0-1.0)
        """
        if not self._ready:
            return

        data = {
            "phase": phase,
            "step_index": step_index,
            "step_desc": step_desc,
            "progress": round(progress, 2),
        }

        self._socketio.emit("grasp_feedback", data)

    def emit_grasp_result(
        self,
        success: bool,
        steps_executed: int = 0,
        candidate_used: int = -1,
        error: str = "",
    ) -> None:
        """
        推送抓取结果通知

        Args:
            success: 是否成功
            steps_executed: 执行的步骤数
            candidate_used: 实际使用的候选索引
            error: 错误信息（失败时）
        """
        if not self._ready:
            return

        data = {"success": success}
        if success:
            data["steps_executed"] = steps_executed
            data["candidate_used"] = candidate_used
        if not success and error:
            data["error"] = error

        self._socketio.emit("grasp_result", data)

    def emit_grasp_pose_result(
        self,
        success: bool,
        candidates: list = None,
        candidate_count: int = 0,
        inference_time_ms: float = 0.0,
        error: str = "",
    ) -> None:
        """
        推送姿态测算结果通知

        Args:
            success: 是否成功
            candidates: 抓取候选列表（前2个）
            candidate_count: 总候选数量
            inference_time_ms: 推理耗时（毫秒）
            error: 错误信息（失败时）
        """
        if not self._ready:
            return

        data = {"success": success}
        if success:
            data["candidates"] = candidates or []
            data["candidate_count"] = candidate_count
            data["inference_time_ms"] = round(inference_time_ms, 2)
        if not success and error:
            data["error"] = error

        self._socketio.emit("grasp_pose_result", data)
