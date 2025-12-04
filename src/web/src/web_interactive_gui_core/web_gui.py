"""WebInteractiveGui - Web 交互界面节点（协调层）"""

from __future__ import annotations

import asyncio
import base64
import threading
import time
from enum import Enum
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import Point, Pose
from builtin_interfaces.msg import Time as TimeMsg

from tf_tools.srv import TransformPoints, TransformPoses
from perception.srv import Grasp
from perception.msg import GraspCandidate

from .camera_manager import CameraManager
from .perception_client import PerceptionClient, SegmentResult
from .robot_skill_client import RobotSkillClient
from .web_server import WebServerManager

# TF 坐标系配置
CAMERA_FRAME = 'camera_color_optical_frame'
ROBOT_BASE_FRAME = 'base_link'
TF_TRANSFORM_SERVICE = '/tf_tools/service/transform_points'
TF_POSES_SERVICE = '/tf_tools/service/transform_poses'
GRASP_SERVICE = '/perception/service/grasp'


class State(Enum):
    """节点状态"""

    IDLE = "idle"  # 空闲，无高亮
    SEGMENTING = "segmenting"  # 分割处理中
    HIGHLIGHTED = "highlighted"  # 有分割结果，显示高亮
    RECORDING = "recording"  # 记录处理中
    GRASPING = "grasping"  # 抓取执行中
    POSE_ESTIMATING = "pose_estimating"  # 姿态测算中


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
        self._point_cloud: Optional[PointCloud2] = None  # 点云（ROS PointCloud2）
        self._segment_3d_info: Optional[dict] = None  # 3D 信息缓存（camera + base_link）
        
        # 抓取候选缓存
        self._grasp_candidates_camera: list = []  # grasp 返回的原始数据（相机坐标系）
        self._grasp_candidates_base: list = []    # TF 转换后的数据（base_link 坐标系）
        
        self._cache_lock = threading.Lock()

        # 2. 创建模块
        self._camera = CameraManager(self, config)
        self._perception = PerceptionClient(self, config)
        self._robot_skill = RobotSkillClient(self, config)
        self._web_server = WebServerManager(config)
        
        # TF 转换服务客户端
        self._tf_client = self.create_client(TransformPoints, TF_TRANSFORM_SERVICE)
        self._tf_poses_client = self.create_client(TransformPoses, TF_POSES_SERVICE)
        
        # Grasp 姿态测算服务客户端
        self._grasp_client = self.create_client(Grasp, GRASP_SERVICE)

        # 3. 注入回调
        self._web_server.set_callbacks(
            on_segment=self._handle_segment,
            on_grasp=self._handle_grasp,
            on_grasp_easy=self._handle_grasp_easy,
            on_grasp_pose=self._handle_grasp_pose,
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
                # 构建 3D 信息（相机坐标系）
                info_3d = {
                    "center_3d": result.center_3d,
                    "bbox_min": result.bbox_min,
                    "bbox_max": result.bbox_max,
                    "point_count": result.point_count,
                    "confidence": result.confidence,
                    # base_link 坐标系下的数据（通过 TF 转换）
                    "center_3d_base": None,
                    "bbox_min_base": None,
                    "bbox_max_base": None,
                }

                # ========== Step: 统一进行坐标转换（camera -> base_link） ==========
                try:
                    transformed = loop.run_until_complete(
                        self._transform_pointcloud_to_base_link(
                            point_cloud=result.point_cloud,
                            center_camera=result.center_3d,
                            bbox_min=result.bbox_min,
                            bbox_max=result.bbox_max,
                        )
                    )
                except Exception as exc:  # 防御性兜底，避免打断主流程
                    transformed = None
                    self.get_logger().warning(
                        f"[web_gui] 点云坐标转换异常（可忽略）: {exc}"
                    )

                if transformed:
                    info_3d["center_3d_base"] = transformed.get("center_base")
                    info_3d["bbox_min_base"] = transformed.get("bbox_min_base")
                    info_3d["bbox_max_base"] = transformed.get("bbox_max_base")
                    # 用转换后的点云替换缓存，确保后续 grasp_easy 使用 base_link 数据
                    result_point_cloud = transformed.get("point_cloud_base") or result.point_cloud
                else:
                    result_point_cloud = result.point_cloud
                
                with self._cache_lock:
                    self._visualization = result.visualization
                    self._cropped_image = result.cropped_image
                    self._point_cloud = result_point_cloud
                    self._segment_3d_info = info_3d

                with self._state_lock:
                    self._state = State.HIGHLIGHTED

                self.get_logger().info(
                    f"[web_gui] 分割成功: confidence={result.confidence:.2f}, "
                    f"center_3d={result.center_3d}, point_count={result.point_count}"
                )
                self._web_server.emit_segment_result(success=True, info_3d=info_3d)
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

    async def _transform_point_to_base(
        self, point_camera: tuple
    ) -> tuple | None:
        """
        将单个点从相机坐标系转换到 base_link 坐标系
        
        Args:
            point_camera: (x, y, z) 相机坐标系下的点
            
        Returns:
            (x, y, z) base_link 坐标系下的点，或 None 如果转换失败
        """
        if not self._tf_client.service_is_ready():
            if not self._tf_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().warning(
                    f"[web_gui] TF 服务 {TF_TRANSFORM_SERVICE} 不可用"
                )
                return None

        # 构建请求
        request = TransformPoints.Request()
        request.source_frame = CAMERA_FRAME
        request.target_frame = ROBOT_BASE_FRAME
        request.stamp = TimeMsg()  # 使用最新 TF
        
        point = Point()
        point.x = float(point_camera[0])
        point.y = float(point_camera[1])
        point.z = float(point_camera[2])
        request.points_in = [point]

        try:
            future = self._tf_client.call_async(request)
            # ROS2 Service Future 不是标准的 concurrent.futures.Future
            # 需要在事件循环中等待完成
            timeout_sec = 3.0
            start_time = time.time()
            
            while not future.done():
                # 检查超时
                elapsed = time.time() - start_time
                if elapsed > timeout_sec:
                    self.get_logger().warning(f"[web_gui] TF 转换超时 (> {timeout_sec:.1f}s)")
                    return None
                
                # 让 ROS2 处理一次
                rclpy.spin_once(self, timeout_sec=0.1)
                # 让出控制权给事件循环
                await asyncio.sleep(0.01)
            
            response = future.result()
            if not response.success:
                self.get_logger().warning(f"[web_gui] TF 转换失败: {response.message}")
                return None
            
            if response.points_out:
                p = response.points_out[0]
                return (p.x, p.y, p.z)
            return None
            
        except asyncio.TimeoutError:
            self.get_logger().warning("[web_gui] TF 转换超时")
            return None
        except Exception as e:
            self.get_logger().warning(f"[web_gui] TF 转换异常: {e}")
            return None

    @staticmethod
    def _as_point_tuple(point_value) -> Optional[tuple[float, float, float]]:
        """Normalize various point representations into a tuple."""
        if point_value is None:
            return None
        if isinstance(point_value, Point):
            return (float(point_value.x), float(point_value.y), float(point_value.z))
        if isinstance(point_value, (list, tuple)) and len(point_value) >= 3:
            return (
                float(point_value[0]),
                float(point_value[1]),
                float(point_value[2]),
            )
        return None

    async def _transform_pointcloud_to_base_link(
        self,
        point_cloud: PointCloud2,
        center_camera: Optional[Point],
        bbox_min: Optional[Point],
        bbox_max: Optional[Point],
    ) -> Optional[dict]:
        """
        将 object_target 的 3D 结果从相机坐标系转换到 base_link 坐标系。

        Args:
            point_cloud: camera_color_optical_frame 下的目标点云
            center_camera: 质心（camera）
            bbox_min: 边界框最小点（camera）
            bbox_max: 边界框最大点（camera）

        Returns:
            dict 包含:
                - point_cloud_base: 转换到 base_link 的点云（目前直接复用输入，占位以便未来扩展）
                - center_base: (x, y, z) 元组
                - bbox_min_base: (x, y, z) 元组
                - bbox_max_base: (x, y, z) 元组
            若转换失败则返回 None。
        """
        # 当前实现只对质心 / bbox 做点转换；点云整体暂不通过 TF 服务转换，
        # 预留 point_cloud_base 字段便于未来扩展。
        if center_camera is None and bbox_min is None and bbox_max is None:
            return None

        center_base = None
        bbox_min_base = None
        bbox_max_base = None

        center_tuple = self._as_point_tuple(center_camera)
        bbox_min_tuple = self._as_point_tuple(bbox_min)
        bbox_max_tuple = self._as_point_tuple(bbox_max)

        if center_tuple is not None:
            center_base = await self._transform_point_to_base(center_tuple)

        if bbox_min_tuple is not None:
            bbox_min_base = await self._transform_point_to_base(bbox_min_tuple)

        if bbox_max_tuple is not None:
            bbox_max_base = await self._transform_point_to_base(bbox_max_tuple)

        if center_base is None and bbox_min_base is None and bbox_max_base is None:
            return None

        return {
            "point_cloud_base": point_cloud,  # 目前仍为 camera frame，占位字段
            "center_base": center_base,
            "bbox_min_base": bbox_min_base,
            "bbox_max_base": bbox_max_base,
        }

    def _convert_segment_points_to_base(self, info_3d: dict) -> tuple[
        Optional[tuple[float, float, float]],
        Optional[tuple[float, float, float]],
        Optional[tuple[float, float, float]],
    ]:
        """Convert cached camera-frame 3D info to base_link via TF service."""

        center_camera = self._as_point_tuple(info_3d.get("center_3d"))
        if center_camera is None:
            return None, None, None

        bbox_min_camera = self._as_point_tuple(info_3d.get("bbox_min"))
        bbox_max_camera = self._as_point_tuple(info_3d.get("bbox_max"))

        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            center_base = loop.run_until_complete(
                self._transform_point_to_base(center_camera)
            )
            bbox_min_base = None
            bbox_max_base = None

            if bbox_min_camera is not None:
                bbox_min_base = loop.run_until_complete(
                    self._transform_point_to_base(bbox_min_camera)
                )

            if bbox_max_camera is not None:
                bbox_max_base = loop.run_until_complete(
                    self._transform_point_to_base(bbox_max_camera)
                )

            return center_base, bbox_min_base, bbox_max_base
        finally:
            asyncio.set_event_loop(None)
            loop.close()

    def _start_grasp_easy_thread(
        self,
        point_cloud: PointCloud2,
        center_base,
        bbox_min_base,
        bbox_max_base,
    ) -> None:
        thread = threading.Thread(
            target=self._run_grasp_easy_async,
            args=(point_cloud, center_base, bbox_min_base, bbox_max_base),
            daemon=True,
        )
        thread.start()

    def _handle_grasp(self) -> None:
        """
        处理记录请求（Flask 线程调用，同步）

        点击"记录"按钮触发，整理分割得到的 3D 信息并调用 grasp_record_easy 动作。
        """
        # 检查状态
        with self._state_lock:
            if self._state != State.HIGHLIGHTED:
                self.get_logger().warning(
                    f"[web_gui] 当前状态 {self._state.value}，无法记录"
                )
                self._web_server.emit_grasp_result(
                    success=False, error="没有可记录的分割结果"
                )
                return
            self._state = State.RECORDING

        with self._cache_lock:
            point_cloud = self._point_cloud
            info_3d = self._segment_3d_info.copy() if self._segment_3d_info else {}

        if point_cloud is None or not info_3d:
            with self._state_lock:
                self._state = State.HIGHLIGHTED
            self._web_server.emit_grasp_result(
                success=False, error="点云或 3D 信息缺失，无法记录"
            )
            return

        center_base, bbox_min_base, bbox_max_base = self._convert_segment_points_to_base(info_3d)
        if center_base is None:
            with self._state_lock:
                self._state = State.HIGHLIGHTED
            self._web_server.emit_grasp_result(
                success=False, error="质心坐标转换失败，无法执行抓取"
            )
            return

        self.get_logger().info(
            "[web_gui] 记录按钮触发 grasp_record_easy: "
            f"center_camera={info_3d.get('center_3d')}, center_base={center_base}"
        )

        self._start_grasp_easy_thread(
            point_cloud=point_cloud,
            center_base=center_base,
            bbox_min_base=bbox_min_base,
            bbox_max_base=bbox_max_base,
        )

    def _handle_grasp_easy(self) -> None:
        """
        处理“易夹取”请求：基于 object_target 结果的几何抓取。
        """
        with self._state_lock:
            if self._state != State.HIGHLIGHTED:
                self.get_logger().warning(
                    f"[web_gui] 当前状态 {self._state.value}，无法执行易夹取"
                )
                self._web_server.emit_grasp_result(
                    success=False, error="没有可抓取的 3D 目标，请先完成分割"
                )
                return
            self._state = State.GRASPING

        with self._cache_lock:
            point_cloud = self._point_cloud
            info_3d = self._segment_3d_info or {}

        if point_cloud is None or not info_3d:
            with self._state_lock:
                self._state = State.HIGHLIGHTED
            self._web_server.emit_grasp_result(
                success=False, error="点云或 3D 信息缺失，无法执行易夹取"
            )
            return

        center_base = info_3d.get("center_3d_base")
        bbox_min_base = info_3d.get("bbox_min_base")
        bbox_max_base = info_3d.get("bbox_max_base")

        if not center_base:
            with self._state_lock:
                self._state = State.HIGHLIGHTED
            self._web_server.emit_grasp_result(
                success=False, error="缺少 base_link 下的质心信息"
            )
            return

        self.get_logger().info(
            "[web_gui] 开始易夹取: "
            f"center_base={center_base}, "
            f"bbox_min_base={bbox_min_base}, bbox_max_base={bbox_max_base}"
        )

        self._start_grasp_easy_thread(
            point_cloud=point_cloud,
            center_base=center_base,
            bbox_min_base=bbox_min_base,
            bbox_max_base=bbox_max_base,
        )

    def _run_grasp_easy_async(
        self,
        point_cloud: PointCloud2,
        center_base,
        bbox_min_base,
        bbox_max_base,
    ) -> None:
        """后台线程中调用 grasp_record_easy。"""
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)

        from geometry_msgs.msg import Point, Vector3

        center_msg = Point(x=center_base[0], y=center_base[1], z=center_base[2])
        bbox_min_msg = None
        bbox_max_msg = None
        if bbox_min_base is not None:
            bbox_min_msg = Vector3(
                x=bbox_min_base[0], y=bbox_min_base[1], z=bbox_min_base[2]
            )
        if bbox_max_base is not None:
            bbox_max_msg = Vector3(
                x=bbox_max_base[0], y=bbox_max_base[1], z=bbox_max_base[2]
            )

        def on_feedback(phase: str, step_index: int, step_desc: str, progress: float):
            self._web_server.emit_grasp_feedback(
                phase=phase,
                step_index=step_index,
                step_desc=step_desc,
                progress=progress,
            )

        try:
            result = loop.run_until_complete(
                self._robot_skill.grasp_easy(
                    point_cloud=point_cloud,
                    center_base=center_msg,
                    bbox_min=bbox_min_msg,
                    bbox_max=bbox_max_msg,
                    context_id="web_grasp_easy",
                    on_feedback=on_feedback,
                )
            )

            if result.success:
                self._clear_cache()
                with self._state_lock:
                    self._state = State.IDLE

                self.get_logger().info(
                    f"[web_gui] 易夹取成功: steps_executed={result.steps_executed}"
                )
                self._web_server.emit_grasp_result(
                    success=True,
                    steps_executed=result.steps_executed,
                )
            else:
                with self._state_lock:
                    self._state = State.HIGHLIGHTED

                self.get_logger().warning(
                    f"[web_gui] 易夹取失败: {result.error_message}"
                )
                self._web_server.emit_grasp_result(
                    success=False, error=result.error_message
                )
        except Exception as exc:  # pylint: disable=broad-except
            with self._state_lock:
                self._state = State.HIGHLIGHTED

            self.get_logger().error(f"[web_gui] 易夹取异常: {exc}")
            self._web_server.emit_grasp_result(success=False, error=str(exc))
        finally:
            loop.close()

    def _handle_grasp_pose(self) -> None:
        """
        处理姿态测算请求（Flask 线程调用，同步）

        在后台线程中调用 Grasp Service
        """
        # 检查状态
        with self._state_lock:
            if self._state != State.HIGHLIGHTED:
                self.get_logger().warning(
                    f"[web_gui] 当前状态 {self._state.value}，无法进行姿态测算"
                )
                self._web_server.emit_grasp_pose_result(
                    success=False, error="没有可测算的分割结果"
                )
                return
            self._state = State.POSE_ESTIMATING

        # 检查是否有点云
        with self._cache_lock:
            if self._point_cloud is None:
                with self._state_lock:
                    self._state = State.HIGHLIGHTED
                self._web_server.emit_grasp_pose_result(
                    success=False, error="点云数据缺失"
                )
                return
            point_cloud = self._point_cloud

        self.get_logger().info("[web_gui] 开始姿态测算...")

        # 在后台线程执行服务调用
        thread = threading.Thread(
            target=self._run_grasp_pose_async,
            args=(point_cloud,),
            daemon=True,
        )
        thread.start()

    def _run_grasp_pose_async(self, point_cloud: PointCloud2) -> None:
        """在后台线程中执行姿态测算 Service 调用"""
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)

        try:
            result = loop.run_until_complete(self._do_grasp_pose(point_cloud))

            if result["success"]:
                with self._state_lock:
                    self._state = State.HIGHLIGHTED

                self.get_logger().info(
                    f"[web_gui] 姿态测算成功: 返回 {result['candidate_count']} 个候选"
                )
                self._web_server.emit_grasp_pose_result(
                    success=True,
                    candidates=result["candidates"],
                    candidate_count=result["candidate_count"],
                    inference_time_ms=result["inference_time_ms"],
                )
            else:
                with self._state_lock:
                    self._state = State.HIGHLIGHTED

                self.get_logger().warning(f"[web_gui] 姿态测算失败: {result['error']}")
                self._web_server.emit_grasp_pose_result(
                    success=False, error=result["error"]
                )

        except Exception as e:
            with self._state_lock:
                self._state = State.HIGHLIGHTED

            self.get_logger().error(f"[web_gui] 姿态测算异常: {e}")
            self._web_server.emit_grasp_pose_result(success=False, error=str(e))

        finally:
            loop.close()

    async def _do_grasp_pose(self, point_cloud: PointCloud2) -> dict:
        """执行姿态测算 Service 调用（异步）"""
        # 检查 Grasp 服务是否可用
        if not self._grasp_client.service_is_ready():
            if not self._grasp_client.wait_for_service(timeout_sec=5.0):
                return {
                    "success": False,
                    "error": f"Grasp 服务 {GRASP_SERVICE} 不可用",
                }

        # 构建请求
        request = Grasp.Request()
        request.point_cloud = point_cloud
        request.max_candidates = 50
        request.min_confidence = 0.1  # Contact-GraspNet输出范围约0-0.2，使用0.1作为阈值

        try:
            # ========== Step 1: 调用 Grasp 服务 ==========
            future = self._grasp_client.call_async(request)
            timeout_sec = 30.0
            start_time = time.time()
            
            while not future.done():
                elapsed = time.time() - start_time
                if elapsed > timeout_sec:
                    return {
                        "success": False,
                        "error": f"姿态测算超时 (> {timeout_sec:.1f}s)",
                    }
                rclpy.spin_once(self, timeout_sec=0.1)
                await asyncio.sleep(0.01)

            response = future.result()
            if not response.success:
                return {
                    "success": False,
                    "error": response.error_message or "姿态测算失败",
                }

            if response.candidate_count == 0:
                return {
                    "success": False,
                    "error": "未找到有效的抓取候选",
                }

            # ========== Step 2: 保存原始候选（相机坐标系） ==========
            candidates_camera = []
            for c in response.candidates:
                candidates_camera.append({
                    "position": {
                        "x": float(c.position.x),
                        "y": float(c.position.y),
                        "z": float(c.position.z),
                    },
                    "orientation": {
                        "x": float(c.orientation.x),
                        "y": float(c.orientation.y),
                        "z": float(c.orientation.z),
                        "w": float(c.orientation.w),
                    },
                    "width": float(c.width),
                    "confidence": float(c.confidence),
                })

            # ========== Step 3: 调用 TF 转换服务 ==========
            candidates_base = await self._transform_grasp_candidates_to_base(
                response.candidates
            )
            
            if candidates_base is None:
                return {
                    "success": False,
                    "error": "TF 坐标转换失败",
                }

            # ========== Step 4: 保存到缓存 ==========
            with self._cache_lock:
                self._grasp_candidates_camera = candidates_camera
                self._grasp_candidates_base = candidates_base

            self.get_logger().info(
                f"[web_gui] 抓取候选已缓存: {len(candidates_camera)} 个 (camera), "
                f"{len(candidates_base)} 个 (base_link)"
            )

            # ========== Step 5: 返回前端展示数据（第一个候选的两个坐标系） ==========
            display_candidates = []
            
            # 第一行：camera 坐标系（原始数据）
            if candidates_camera:
                c = candidates_camera[0]
                display_candidates.append({
                    "frame": "camera",
                    "position": {
                        "x": round(c["position"]["x"], 4),
                        "y": round(c["position"]["y"], 4),
                        "z": round(c["position"]["z"], 4),
                    },
                    "orientation": {
                        "x": round(c["orientation"]["x"], 4),
                        "y": round(c["orientation"]["y"], 4),
                        "z": round(c["orientation"]["z"], 4),
                        "w": round(c["orientation"]["w"], 4),
                    },
                    "width": round(c["width"], 4),
                    "confidence": round(c["confidence"], 4),
                })
            
            # 第二行：base_link 坐标系（转换后）
            if candidates_base:
                c = candidates_base[0]
                display_candidates.append({
                    "frame": "base_link",
                    "position": {
                        "x": round(c["position"]["x"], 4),
                        "y": round(c["position"]["y"], 4),
                        "z": round(c["position"]["z"], 4),
                    },
                    "orientation": {
                        "x": round(c["orientation"]["x"], 4),
                        "y": round(c["orientation"]["y"], 4),
                        "z": round(c["orientation"]["z"], 4),
                        "w": round(c["orientation"]["w"], 4),
                    },
                    "width": round(c["width"], 4),
                    "confidence": round(c["confidence"], 4),
                })

            return {
                "success": True,
                "candidates": display_candidates,
                "candidate_count": response.candidate_count,
                "inference_time_ms": response.inference_time_ms,
            }

        except asyncio.TimeoutError:
            return {
                "success": False,
                "error": "姿态测算超时 (> 30s)",
            }
        except Exception as e:
            self.get_logger().error(f"[web_gui] 姿态测算异常: {e}")
            return {
                "success": False,
                "error": f"姿态测算异常: {e}",
            }

    async def _transform_grasp_candidates_to_base(
        self, candidates: list
    ) -> list | None:
        """
        将抓取候选从相机坐标系转换到 base_link 坐标系
        
        Args:
            candidates: GraspCandidate 列表（ROS 消息）
            
        Returns:
            转换后的候选列表（字典格式），或 None 如果转换失败
        """
        if not candidates:
            return []
        
        # 检查 TF Poses 服务是否可用
        if not self._tf_poses_client.service_is_ready():
            if not self._tf_poses_client.wait_for_service(timeout_sec=3.0):
                self.get_logger().warning(
                    f"[web_gui] TF Poses 服务 {TF_POSES_SERVICE} 不可用"
                )
                return None

        # 构建请求
        request = TransformPoses.Request()
        request.source_frame = CAMERA_FRAME
        request.target_frame = ROBOT_BASE_FRAME
        request.stamp = TimeMsg()  # 使用最新 TF
        
        # 将 GraspCandidate 转换为 Pose
        poses_in = []
        for c in candidates:
            pose = Pose()
            pose.position.x = float(c.position.x)
            pose.position.y = float(c.position.y)
            pose.position.z = float(c.position.z)
            pose.orientation.x = float(c.orientation.x)
            pose.orientation.y = float(c.orientation.y)
            pose.orientation.z = float(c.orientation.z)
            pose.orientation.w = float(c.orientation.w)
            poses_in.append(pose)
        
        request.poses_in = poses_in

        try:
            future = self._tf_poses_client.call_async(request)
            timeout_sec = 5.0
            start_time = time.time()
            
            while not future.done():
                elapsed = time.time() - start_time
                if elapsed > timeout_sec:
                    self.get_logger().warning(
                        f"[web_gui] TF Poses 转换超时 (> {timeout_sec:.1f}s)"
                    )
                    return None
                rclpy.spin_once(self, timeout_sec=0.1)
                await asyncio.sleep(0.01)
            
            response = future.result()
            if not response.success:
                self.get_logger().warning(
                    f"[web_gui] TF Poses 转换失败: {response.message}"
                )
                return None

            # 组装转换后的候选（保留 width 和 confidence）
            candidates_base = []
            for i, pose_out in enumerate(response.poses_out):
                original = candidates[i]
                candidates_base.append({
                    "position": {
                        "x": float(pose_out.position.x),
                        "y": float(pose_out.position.y),
                        "z": float(pose_out.position.z),
                    },
                    "orientation": {
                        "x": float(pose_out.orientation.x),
                        "y": float(pose_out.orientation.y),
                        "z": float(pose_out.orientation.z),
                        "w": float(pose_out.orientation.w),
                    },
                    "width": float(original.width),          # 保留原值
                    "confidence": float(original.confidence),  # 保留原值
                })

            return candidates_base

        except Exception as e:
            self.get_logger().error(f"[web_gui] TF Poses 转换异常: {e}")
            return None

    def _handle_cancel(self) -> None:
        """处理取消请求"""
        with self._state_lock:
            if self._state in (State.SEGMENTING, State.RECORDING, State.GRASPING, State.POSE_ESTIMATING):
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
            "processing": state in (State.SEGMENTING, State.RECORDING, State.GRASPING, State.POSE_ESTIMATING),
        }

    def _get_health(self) -> dict:
        """获取健康状态"""
        camera_ready, camera_msg = self._camera.is_ready()
        perception_ready = self._perception.is_ready()
        robot_skill_ready = self._robot_skill.is_ready()
        web_ready = self._web_server.is_ready()

        all_ready = web_ready  # 最低要求：Web 服务可用

        return {
            "status": "ok" if all_ready else "degraded",
            "ready": all_ready,
            "camera": {"ready": camera_ready, "message": camera_msg},
            "perception": {"ready": perception_ready},
            "robot_skill": {"ready": robot_skill_ready},
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
            self._point_cloud = None
            self._segment_3d_info = None
            # 清除抓取候选缓存
            self._grasp_candidates_camera = []
            self._grasp_candidates_base = []


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
