"""
PerceptionNode - 主节点（协调层）

ROS 2 节点入口，协调各模块，注册 Action/Service
"""

import asyncio
import logging
import time
from pathlib import Path
from typing import Dict, Any, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from geometry_msgs.msg import Point, Vector3

# 导入生成的消息
from perception.action import (
    Segment,
    PointCloud as PointCloudAction,
    Vectorize,
    ObjectTarget,
    ObjectRecord,
    ObjectProcess,
)
from perception.srv import (
    SaveObject,
    AddSample,
    UpdateObject,
    DeleteObject,
    DeleteSample,
    QueryByDesc,
    QueryByImage,
    QueryByLabel,
    QueryById,
    ListObjects,
    ListSamples,
    ClearAll,
    GetStatus,
    Grasp,
)
from perception.msg import ObjectInfo, SampleInfo, GraspCandidate

# 导入核心模块
from .constants import (
    DEFAULT_STORAGE_DIR,
    TIMEOUT_SEGMENT,
    TIMEOUT_POINTCLOUD,
    TIMEOUT_VECTORIZE,
    TIMEOUT_GRASP,
    DEFAULT_TOP_K,
    DEFAULT_MIN_SIMILARITY,
    DEFAULT_MAX_CANDIDATES,
    DEFAULT_MIN_GRASP_CONFIDENCE,
    Z_RANGE_MIN,
    Z_RANGE_MAX,
)
from .error_codes import PerceptionError, ErrorCode
from .segmentation import SegmentationModule
from .pointcloud import PointCloudModule
from .vectorizer import VectorizerModule
from .vector_manager import VectorManager
from .object_storage_manager import ObjectStorageManager
from .async_worker import AsyncWorker
from .grasp_module import GraspModule, GraspResult

logger = logging.getLogger(__name__)


class PerceptionNode(Node):
    """
    Perception 主节点
    
    协调各模块，提供 Action/Service 接口
    """
    
    def __init__(self):
        super().__init__('perception_node')
        
        self._ready = False
        self._start_time = time.time()
        self._requests_processed = 0
        self._requests_failed = 0
        
        # CV Bridge
        self._bridge = CvBridge()
        
        # 回调组（允许并发）
        self._callback_group = ReentrantCallbackGroup()
        
        # 加载配置
        self._load_config()
        
        # 创建模块实例（不初始化）
        self._create_modules()
        
        # 启动异步初始化
        self._init_timer = self.create_timer(
            0.1, self._init_timer_callback, callback_group=self._callback_group
        )
        
        self.get_logger().info("PerceptionNode 创建完成，开始初始化...")
    
    def _load_config(self):
        """加载配置"""
        # 获取包路径
        self._package_dir = Path(__file__).parent.parent.parent
        
        # 存储目录
        self._storage_dir = self._package_dir / DEFAULT_STORAGE_DIR
        
        # 模块配置
        self._segmentation_config = {
            'engine_path': str(self._package_dir / 'models' / 'nanosam_image_encoder.engine'),
            'decoder_path': str(self._package_dir / 'models' / 'nanosam_mask_decoder.engine'),
        }
        
        self._pointcloud_config = {
            'output_frame_id': 'camera_color_optical_frame',
        }
        
        self._vectorizer_config = {
            'clip': {'model': 'openai/clip-vit-base-patch32', 'device': 'cuda'},
            'dino': {'model_path': str(self._package_dir / 'models' / 'dinov2_vits14.pth'), 'device': 'cuda'},
        }
        
        self._vector_config = {
            'index_type': 'IndexFlatL2',
        }
        
        self._grasp_config = {
            'model_path': str(self._package_dir / 'models' / 'contact_graspnet' / 'model.pt'),
            'config_path': str(self._package_dir / 'models' / 'contact_graspnet' / 'config.yaml'),
            'max_candidates': DEFAULT_MAX_CANDIDATES,
            'min_confidence': DEFAULT_MIN_GRASP_CONFIDENCE,
            'z_range': [Z_RANGE_MIN, Z_RANGE_MAX],
            'timeout': TIMEOUT_GRASP,
        }
    
    def _create_modules(self):
        """创建模块实例"""
        # 创建共享的 AsyncWorker（所有模块共享）
        self._worker = AsyncWorker(name="perception_async_worker")
        self._worker.start()
        
        # 核心模块
        self._segmentation = SegmentationModule(self._segmentation_config, self._worker)
        self._pointcloud = PointCloudModule(self._pointcloud_config, self._worker)
        self._vectorizer = VectorizerModule(self._vectorizer_config, self._worker)
        self._vector_manager = VectorManager(
            self._vector_config,
            str(self._storage_dir),
            self._worker
        )
        self._storage = ObjectStorageManager(
            {},
            str(self._storage_dir),
            self._vector_manager,
            self._worker
        )
        
        # 抓取模块（共享同一个 worker）
        self._grasp = GraspModule(self._grasp_config, self._worker)
    
    def _init_timer_callback(self):
        """初始化定时器回调"""
        # 只执行一次
        self._init_timer.cancel()
        
        # 启动异步初始化（在新的事件循环中运行）
        asyncio.run(self._initialize_modules())
    
    async def _initialize_modules(self):
        """异步初始化所有模块"""
        try:
            self.get_logger().info("开始初始化模块...")
            
            # 并行初始化独立模块
            await asyncio.gather(
                self._segmentation.initialize(),
                self._pointcloud.initialize(),
                self._vectorizer.initialize(),
                self._vector_manager.initialize(),
            )
            
            # 串行初始化依赖模块
            await self._storage.initialize()
            
            # 初始化抓取模块（单独初始化，因为加载时间较长）
            self.get_logger().info("初始化抓取模块（Contact-GraspNet）...")
            await self._grasp.initialize()
            
            # 检查所有模块就绪
            if not all([
                self._segmentation.is_ready,
                self._pointcloud.is_ready,
                self._vectorizer.is_ready,
                self._vector_manager.is_ready,
                self._storage.is_ready,
                self._grasp.is_ready,
            ]):
                raise RuntimeError("部分模块初始化失败")
            
            # 注册服务
            self._register_actions()
            self._register_services()
            
            self._ready = True
            self.get_logger().info("✅ PerceptionNode 服务就绪")
            
        except Exception as e:
            self.get_logger().error(f"模块初始化失败: {e}")
            self._ready = False
    
    def _register_actions(self):
        """注册 Action Server"""
        # 原子 Action
        self._segment_action = ActionServer(
            self, Segment, '/perception/action/segment',
            execute_callback=self._wrap_async_execute(self._segment_execute),
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )
        
        self._pointcloud_action = ActionServer(
            self, PointCloudAction, '/perception/action/pointcloud',
            execute_callback=self._wrap_async_execute(self._pointcloud_execute),
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )
        
        self._vectorize_action = ActionServer(
            self, Vectorize, '/perception/action/vectorize',
            execute_callback=self._wrap_async_execute(self._vectorize_execute),
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )
        
        # 组合 Action
        self._object_target_action = ActionServer(
            self, ObjectTarget, '/perception/action/object_target',
            execute_callback=self._wrap_async_execute(self._object_target_execute),
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )
        
        self._object_record_action = ActionServer(
            self, ObjectRecord, '/perception/action/object_record',
            execute_callback=self._wrap_async_execute(self._object_record_execute),
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )
        
        self._object_process_action = ActionServer(
            self, ObjectProcess, '/perception/action/object_process',
            execute_callback=self._wrap_async_execute(self._object_process_execute),
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )
        
        self.get_logger().info("注册 6 个 Action Server")
    
    def _register_services(self):
        """注册 Service Server"""
        # 存储服务
        self.create_service(SaveObject, '/perception/service/save_object',
                           self._save_object_callback, callback_group=self._callback_group)
        self.create_service(AddSample, '/perception/service/add_sample',
                           self._add_sample_callback, callback_group=self._callback_group)
        self.create_service(UpdateObject, '/perception/service/update_object',
                           self._update_object_callback, callback_group=self._callback_group)
        self.create_service(DeleteObject, '/perception/service/delete_object',
                           self._delete_object_callback, callback_group=self._callback_group)
        self.create_service(DeleteSample, '/perception/service/delete_sample',
                           self._delete_sample_callback, callback_group=self._callback_group)
        
        # 查询服务
        self.create_service(QueryByDesc, '/perception/service/query_by_desc',
                           self._query_by_desc_callback, callback_group=self._callback_group)
        self.create_service(QueryByImage, '/perception/service/query_by_image',
                           self._query_by_image_callback, callback_group=self._callback_group)
        self.create_service(QueryByLabel, '/perception/service/query_by_label',
                           self._query_by_label_callback, callback_group=self._callback_group)
        self.create_service(QueryById, '/perception/service/query_by_id',
                           self._query_by_id_callback, callback_group=self._callback_group)
        
        # 列表服务
        self.create_service(ListObjects, '/perception/service/list_objects',
                           self._list_objects_callback, callback_group=self._callback_group)
        self.create_service(ListSamples, '/perception/service/list_samples',
                           self._list_samples_callback, callback_group=self._callback_group)
        self.create_service(ClearAll, '/perception/service/clear_all',
                           self._clear_all_callback, callback_group=self._callback_group)
        
        # 系统服务
        self.create_service(GetStatus, '/perception/service/get_status',
                           self._get_status_callback, callback_group=self._callback_group)
        
        # ⚠️ 抓取服务 - 使用 MutuallyExclusiveCallbackGroup 避免并发 GPU 冲突
        self._grasp_callback_group = MutuallyExclusiveCallbackGroup()
        self.create_service(Grasp, '/perception/service/grasp',
                           self._grasp_callback, callback_group=self._grasp_callback_group)
        
        self.get_logger().info("注册 14 个 Service Server（含 grasp 服务）")
    
    def _goal_callback(self, goal_request):
        """Action Goal 回调"""
        if not self._ready:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT
    
    def _cancel_callback(self, goal_handle):
        """Action Cancel 回调"""
        return CancelResponse.ACCEPT
    
    # =========================================================================
    # Action 回调实现
    # =========================================================================
    
    async def _segment_execute(self, goal_handle):
        """Segment Action 执行"""
        result = Segment.Result()
        
        try:
            goal = goal_handle.request
            
            # 转换图像
            image = self._bridge.imgmsg_to_cv2(goal.color_image, 'rgb8')
            
            # 发送反馈
            feedback = Segment.Feedback()
            feedback.status = "encoding"
            feedback.progress = 0.3
            goal_handle.publish_feedback(feedback)
            
            # 执行分割
            seg_result = await self._segmentation.segment(
                image, goal.click_x, goal.click_y
            )
            
            feedback.status = "decoding"
            feedback.progress = 0.8
            goal_handle.publish_feedback(feedback)
            
            # 生成可视化
            visualization = self._segmentation.create_visualization(
                image, seg_result.mask, (goal.click_x, goal.click_y)
            )
            
            # 填充结果
            result.success = True
            result.mask = self._bridge.cv2_to_imgmsg(seg_result.mask, 'mono8')
            result.cropped_image = self._bridge.cv2_to_imgmsg(seg_result.cropped_image, 'rgb8')
            result.visualization = self._bridge.cv2_to_imgmsg(visualization, 'rgb8')
            result.confidence = seg_result.confidence
            result.mask_area_pixels = seg_result.mask_area
            
            self._requests_processed += 1
            goal_handle.succeed()
            
        except PerceptionError as e:
            result.success = False
            result.error_message = e.to_error_message()
            self._requests_failed += 1
            goal_handle.succeed()
            
        except Exception as e:
            result.success = False
            result.error_message = f"[E9003] 内部异常: {e}"
            self._requests_failed += 1
            goal_handle.succeed()
        
        return result
    
    async def _pointcloud_execute(self, goal_handle):
        """PointCloud Action 执行"""
        result = PointCloudAction.Result()
        
        try:
            goal = goal_handle.request
            
            # 转换图像
            mask = self._bridge.imgmsg_to_cv2(goal.mask, 'mono8')
            depth = self._bridge.imgmsg_to_cv2(goal.depth_image, 'passthrough')
            
            # 发送反馈
            feedback = PointCloudAction.Feedback()
            feedback.status = "computing"
            feedback.progress = 0.5
            goal_handle.publish_feedback(feedback)
            
            # 计算点云
            pc_result = await self._pointcloud.compute(mask, depth, goal.camera_info)
            
            # 填充结果
            result.success = True
            result.point_cloud = pc_result.point_cloud
            result.center_3d = Point(
                x=pc_result.center_3d[0],
                y=pc_result.center_3d[1],
                z=pc_result.center_3d[2]
            )
            result.bbox_min = Vector3(
                x=pc_result.bbox_min[0],
                y=pc_result.bbox_min[1],
                z=pc_result.bbox_min[2]
            )
            result.bbox_max = Vector3(
                x=pc_result.bbox_max[0],
                y=pc_result.bbox_max[1],
                z=pc_result.bbox_max[2]
            )
            result.point_count = pc_result.point_count
            
            self._requests_processed += 1
            goal_handle.succeed()
            
        except PerceptionError as e:
            result.success = False
            result.error_message = e.to_error_message()
            self._requests_failed += 1
            goal_handle.succeed()
            
        except Exception as e:
            result.success = False
            result.error_message = f"[E9003] 内部异常: {e}"
            self._requests_failed += 1
            goal_handle.succeed()
        
        return result
    
    async def _vectorize_execute(self, goal_handle):
        """Vectorize Action 执行"""
        result = Vectorize.Result()
        
        try:
            goal = goal_handle.request
            
            # 转换图像
            image = self._bridge.imgmsg_to_cv2(goal.cropped_image, 'rgb8')
            
            # 发送反馈
            feedback = Vectorize.Feedback()
            feedback.status = "clip_encoding"
            feedback.progress = 0.3
            goal_handle.publish_feedback(feedback)
            
            # 向量化
            vec_result = await self._vectorizer.extract(image)
            
            feedback.status = "dino_encoding"
            feedback.progress = 0.8
            goal_handle.publish_feedback(feedback)
            
            # 填充结果
            result.success = True
            result.clip_embedding = vec_result.clip_embedding.tolist()
            result.dino_embedding = vec_result.dino_embedding.tolist()
            
            self._requests_processed += 1
            goal_handle.succeed()
            
        except PerceptionError as e:
            result.success = False
            result.error_message = e.to_error_message()
            self._requests_failed += 1
            goal_handle.succeed()
            
        except Exception as e:
            result.success = False
            result.error_message = f"[E9003] 内部异常: {e}"
            self._requests_failed += 1
            goal_handle.succeed()
        
        return result
    
    async def _object_target_execute(self, goal_handle):
        """ObjectTarget Action 执行（分割 + 点云）"""
        result = ObjectTarget.Result()
        
        try:
            goal = goal_handle.request
            feedback = ObjectTarget.Feedback()

            color_img = goal.color_image
            depth_img = goal.depth_image
            cam_info = goal.camera_info
            goal_log = (
                "[object_target] Goal received: "
                f"color={getattr(color_img, 'width', 0)}x{getattr(color_img, 'height', 0)}"
                f" enc={getattr(color_img, 'encoding', 'unknown')} step={getattr(color_img, 'step', 0)} | "
                f"depth={getattr(depth_img, 'width', 0)}x{getattr(depth_img, 'height', 0)}"
                f" enc={getattr(depth_img, 'encoding', 'unknown')} step={getattr(depth_img, 'step', 0)} | "
                f"camera frame={getattr(getattr(cam_info, 'header', None), 'frame_id', '')}"
                f" ({getattr(cam_info, 'width', 0)}x{getattr(cam_info, 'height', 0)}) | "
                f"click=({goal.click_x:.1f}, {goal.click_y:.1f})"
            )
            self.get_logger().info(goal_log)

            intrinsics_log = (
                "[object_target] Camera intrinsics "
                f"K={list(cam_info.k)} D_first5={list(cam_info.d[:5]) if cam_info.d else []}"
            )
            self.get_logger().debug(intrinsics_log)
            
            # 转换图像
            image = self._bridge.imgmsg_to_cv2(goal.color_image, 'rgb8')
            depth = self._bridge.imgmsg_to_cv2(goal.depth_image, 'passthrough')
            
            # Step 1: 分割
            feedback.status = "segmenting"
            feedback.progress = 0.2
            goal_handle.publish_feedback(feedback)
            
            seg_result = await self._segmentation.segment(
                image, goal.click_x, goal.click_y
            )
            
            # Step 2: 点云计算
            feedback.status = "computing_pointcloud"
            feedback.progress = 0.6
            goal_handle.publish_feedback(feedback)
            
            pc_result = await self._pointcloud.compute(
                seg_result.mask, depth, goal.camera_info
            )
            
            # 生成可视化
            visualization = self._segmentation.create_visualization(
                image, seg_result.mask, (goal.click_x, goal.click_y)
            )
            
            # 填充结果
            result.success = True
            result.mask = self._bridge.cv2_to_imgmsg(seg_result.mask, 'mono8')
            result.cropped_image = self._bridge.cv2_to_imgmsg(seg_result.cropped_image, 'rgb8')
            result.visualization = self._bridge.cv2_to_imgmsg(visualization, 'rgb8')
            result.confidence = seg_result.confidence
            result.mask_area_pixels = seg_result.mask_area
            result.point_cloud = pc_result.point_cloud
            result.center_3d = Point(
                x=pc_result.center_3d[0],
                y=pc_result.center_3d[1],
                z=pc_result.center_3d[2]
            )
            result.bbox_min = Vector3(
                x=pc_result.bbox_min[0],
                y=pc_result.bbox_min[1],
                z=pc_result.bbox_min[2]
            )
            result.bbox_max = Vector3(
                x=pc_result.bbox_max[0],
                y=pc_result.bbox_max[1],
                z=pc_result.bbox_max[2]
            )
            result.point_count = pc_result.point_count
            
            self._requests_processed += 1
            goal_handle.succeed()
            
        except PerceptionError as e:
            result.success = False
            result.error_message = e.to_error_message()
            self._requests_failed += 1
            goal_handle.succeed()
            
        except Exception as e:
            result.success = False
            result.error_message = f"[E9003] 内部异常: {e}"
            self._requests_failed += 1
            goal_handle.succeed()
        
        return result
    
    async def _object_record_execute(self, goal_handle):
        """ObjectRecord Action 执行（向量化 + 保存）"""
        result = ObjectRecord.Result()
        
        try:
            goal = goal_handle.request
            feedback = ObjectRecord.Feedback()
            
            # 转换图像
            image = self._bridge.imgmsg_to_cv2(goal.cropped_image, 'rgb8')
            
            # Step 1: 向量化
            feedback.status = "vectorizing"
            feedback.progress = 0.3
            goal_handle.publish_feedback(feedback)
            
            vec_result = await self._vectorizer.extract(image)
            
            # Step 2: 保存
            feedback.status = "saving"
            feedback.progress = 0.8
            goal_handle.publish_feedback(feedback)
            
            object_id, created_at = self._storage.save_object(
                image,
                vec_result.clip_embedding,
                vec_result.dino_embedding,
                label=goal.label,
                description=goal.description
            )
            
            # 填充结果
            result.success = True
            result.object_id = object_id
            result.created_at = created_at
            result.clip_embedding = vec_result.clip_embedding.tolist()
            result.dino_embedding = vec_result.dino_embedding.tolist()
            
            self._requests_processed += 1
            goal_handle.succeed()
            
        except PerceptionError as e:
            result.success = False
            result.error_message = e.to_error_message()
            self._requests_failed += 1
            goal_handle.succeed()
            
        except Exception as e:
            result.success = False
            result.error_message = f"[E9003] 内部异常: {e}"
            self._requests_failed += 1
            goal_handle.succeed()
        
        return result
    
    async def _object_process_execute(self, goal_handle):
        """ObjectProcess Action 执行（完整流程）"""
        result = ObjectProcess.Result()
        
        try:
            goal = goal_handle.request
            feedback = ObjectProcess.Feedback()
            
            # 转换图像
            image = self._bridge.imgmsg_to_cv2(goal.color_image, 'rgb8')
            depth = self._bridge.imgmsg_to_cv2(goal.depth_image, 'passthrough')
            
            # Step 1: 分割
            feedback.status = "segmenting"
            feedback.progress = 0.15
            goal_handle.publish_feedback(feedback)
            
            seg_result = await self._segmentation.segment(
                image, goal.click_x, goal.click_y
            )
            
            # Step 2: 点云计算
            feedback.status = "computing_pointcloud"
            feedback.progress = 0.35
            goal_handle.publish_feedback(feedback)
            
            pc_result = await self._pointcloud.compute(
                seg_result.mask, depth, goal.camera_info
            )
            
            # Step 3: 向量化
            feedback.status = "vectorizing"
            feedback.progress = 0.6
            goal_handle.publish_feedback(feedback)
            
            vec_result = await self._vectorizer.extract(seg_result.cropped_image)
            
            # Step 4: 保存
            feedback.status = "saving"
            feedback.progress = 0.85
            goal_handle.publish_feedback(feedback)
            
            object_id, created_at = self._storage.save_object(
                seg_result.cropped_image,
                vec_result.clip_embedding,
                vec_result.dino_embedding,
                label=goal.label,
                description=goal.description
            )
            
            # 生成可视化
            visualization = self._segmentation.create_visualization(
                image, seg_result.mask, (goal.click_x, goal.click_y)
            )
            
            # 填充结果
            result.success = True
            result.object_id = object_id
            result.created_at = created_at
            result.mask = self._bridge.cv2_to_imgmsg(seg_result.mask, 'mono8')
            result.cropped_image = self._bridge.cv2_to_imgmsg(seg_result.cropped_image, 'rgb8')
            result.visualization = self._bridge.cv2_to_imgmsg(visualization, 'rgb8')
            result.confidence = seg_result.confidence
            result.mask_area_pixels = seg_result.mask_area
            result.point_cloud = pc_result.point_cloud
            result.center_3d = Point(
                x=pc_result.center_3d[0],
                y=pc_result.center_3d[1],
                z=pc_result.center_3d[2]
            )
            result.bbox_min = Vector3(
                x=pc_result.bbox_min[0],
                y=pc_result.bbox_min[1],
                z=pc_result.bbox_min[2]
            )
            result.bbox_max = Vector3(
                x=pc_result.bbox_max[0],
                y=pc_result.bbox_max[1],
                z=pc_result.bbox_max[2]
            )
            result.point_count = pc_result.point_count
            result.clip_embedding = vec_result.clip_embedding.tolist()
            result.dino_embedding = vec_result.dino_embedding.tolist()
            
            self._requests_processed += 1
            goal_handle.succeed()
            
        except PerceptionError as e:
            result.success = False
            result.error_message = e.to_error_message()
            self._requests_failed += 1
            goal_handle.succeed()
            
        except Exception as e:
            result.success = False
            result.error_message = f"[E9003] 内部异常: {e}"
            self._requests_failed += 1
            goal_handle.succeed()
        
        return result
    
    # =========================================================================
    # Service 回调实现
    # =========================================================================
    
    def _save_object_callback(self, request, response):
        """SaveObject Service 回调"""
        try:
            image = self._bridge.imgmsg_to_cv2(request.cropped_image, 'rgb8')
            clip_emb = np.array(request.clip_embedding, dtype=np.float32)
            dino_emb = np.array(request.dino_embedding, dtype=np.float32)
            
            object_id, created_at = self._storage.save_object(
                image, clip_emb, dino_emb,
                label=request.label,
                description=request.description
            )
            
            response.success = True
            response.object_id = object_id
            response.created_at = created_at
            
        except PerceptionError as e:
            response.success = False
            response.error_message = e.to_error_message()
        except Exception as e:
            response.success = False
            response.error_message = f"[E9003] 内部异常: {e}"
        
        return response
    
    def _add_sample_callback(self, request, response):
        """AddSample Service 回调"""
        try:
            clip_emb = np.array(request.clip_embedding, dtype=np.float32)
            dino_emb = np.array(request.dino_embedding, dtype=np.float32)
            
            sample_id, total = self._storage.add_sample(
                request.object_id, clip_emb, dino_emb
            )
            
            response.success = True
            response.sample_id = sample_id
            response.total_samples = total
            
        except PerceptionError as e:
            response.success = False
            response.error_message = e.to_error_message()
        except Exception as e:
            response.success = False
            response.error_message = f"[E9003] 内部异常: {e}"
        
        return response
    
    def _update_object_callback(self, request, response):
        """UpdateObject Service 回调"""
        try:
            updated_at = self._storage.update_object(
                request.object_id,
                label=request.label if request.label else None,
                description=request.description if request.description else None
            )
            
            response.success = True
            response.updated_at = updated_at
            
        except PerceptionError as e:
            response.success = False
            response.error_message = e.to_error_message()
        except Exception as e:
            response.success = False
            response.error_message = f"[E9003] 内部异常: {e}"
        
        return response
    
    def _delete_object_callback(self, request, response):
        """DeleteObject Service 回调"""
        try:
            deleted = self._storage.delete_object(request.object_id)
            
            response.success = True
            response.deleted_samples = deleted
            
        except PerceptionError as e:
            response.success = False
            response.error_message = e.to_error_message()
        except Exception as e:
            response.success = False
            response.error_message = f"[E9003] 内部异常: {e}"
        
        return response
    
    def _delete_sample_callback(self, request, response):
        """DeleteSample Service 回调"""
        try:
            remaining = self._storage.delete_sample(
                request.object_id, request.sample_id
            )
            
            response.success = True
            response.remaining_samples = remaining
            
        except PerceptionError as e:
            response.success = False
            response.error_message = e.to_error_message()
        except Exception as e:
            response.success = False
            response.error_message = f"[E9003] 内部异常: {e}"
        
        return response
    
    def _query_by_desc_callback(self, request, response):
        """QueryByDesc Service 回调"""
        try:
            if not request.description:
                raise PerceptionError(ErrorCode.QUERY_TEXT_EMPTY)
            
            # 编码文本（使用同步方法）
            text_emb = self._vectorizer._encode_text_sync(request.description)
            
            # 查询
            top_k = request.top_k if request.top_k > 0 else DEFAULT_TOP_K
            min_sim = request.min_similarity if request.min_similarity > 0 else DEFAULT_MIN_SIMILARITY
            
            results = self._storage.query_by_clip(text_emb, top_k, min_sim)
            
            response.success = True
            response.objects = [self._to_object_info_msg(info) for info, _ in results]
            response.similarities = [sim for _, sim in results]
            
        except PerceptionError as e:
            response.success = False
            response.error_message = e.to_error_message()
        except Exception as e:
            response.success = False
            response.error_message = f"[E9003] 内部异常: {e}"
        
        return response
    
    def _query_by_image_callback(self, request, response):
        """QueryByImage Service 回调"""
        try:
            if request.query_image.data is None or len(request.query_image.data) == 0:
                raise PerceptionError(ErrorCode.QUERY_IMAGE_EMPTY)
            
            # 转换图像
            image = self._bridge.imgmsg_to_cv2(request.query_image, 'rgb8')
            
            # 提取特征（使用同步方法）
            dino_emb = self._vectorizer._extract_dino(image)
            
            # 查询
            top_k = request.top_k if request.top_k > 0 else DEFAULT_TOP_K
            min_sim = request.min_similarity if request.min_similarity > 0 else DEFAULT_MIN_SIMILARITY
            
            results = self._storage.query_by_dino(dino_emb, top_k, min_sim)
            
            response.success = True
            response.objects = [self._to_object_info_msg(info) for info, _ in results]
            response.similarities = [sim for _, sim in results]
            
        except PerceptionError as e:
            response.success = False
            response.error_message = e.to_error_message()
        except Exception as e:
            response.success = False
            response.error_message = f"[E9003] 内部异常: {e}"
        
        return response
    
    def _query_by_label_callback(self, request, response):
        """QueryByLabel Service 回调"""
        try:
            results = self._storage.query_by_label(request.label)
            
            response.success = True
            response.objects = [self._to_object_info_msg(info) for info in results]
            
        except PerceptionError as e:
            response.success = False
            response.error_message = e.to_error_message()
        except Exception as e:
            response.success = False
            response.error_message = f"[E9003] 内部异常: {e}"
        
        return response
    
    def _query_by_id_callback(self, request, response):
        """QueryById Service 回调"""
        try:
            info = self._storage.query_by_id(request.object_id)
            
            if info is None:
                raise PerceptionError(ErrorCode.OBJECT_NOT_FOUND, request.object_id)
            
            response.success = True
            response.object = self._to_object_info_msg(info)
            response.sample_count = info.sample_count
            
            if request.include_image:
                image = self._storage.get_object_image(request.object_id)
                if image is not None:
                    response.cropped_image = self._bridge.cv2_to_imgmsg(image, 'rgb8')
            
        except PerceptionError as e:
            response.success = False
            response.error_message = e.to_error_message()
        except Exception as e:
            response.success = False
            response.error_message = f"[E9003] 内部异常: {e}"
        
        return response
    
    def _list_objects_callback(self, request, response):
        """ListObjects Service 回调"""
        try:
            offset = request.offset if request.offset > 0 else 0
            limit = request.limit if request.limit > 0 else 100
            
            objects, total = self._storage.list_objects(
                label_filter=request.label_filter,
                offset=offset,
                limit=limit
            )
            
            response.success = True
            response.objects = [self._to_object_info_msg(info) for info in objects]
            response.total_count = total
            
        except PerceptionError as e:
            response.success = False
            response.error_message = e.to_error_message()
        except Exception as e:
            response.success = False
            response.error_message = f"[E9003] 内部异常: {e}"
        
        return response
    
    def _list_samples_callback(self, request, response):
        """ListSamples Service 回调"""
        try:
            samples = self._storage.list_samples(request.object_id)
            
            response.success = True
            response.samples = [self._to_sample_info_msg(s) for s in samples]
            response.total_count = len(samples)
            
        except PerceptionError as e:
            response.success = False
            response.error_message = e.to_error_message()
        except Exception as e:
            response.success = False
            response.error_message = f"[E9003] 内部异常: {e}"
        
        return response
    
    def _clear_all_callback(self, request, response):
        """ClearAll Service 回调"""
        try:
            if not request.confirm:
                response.success = False
                response.error_message = "[E9003] 需要 confirm=true 确认"
                return response
            
            obj_count, sample_count = self._storage.clear_all()
            
            response.success = True
            response.deleted_objects = obj_count
            response.deleted_samples = sample_count
            
        except PerceptionError as e:
            response.success = False
            response.error_message = e.to_error_message()
        except Exception as e:
            response.success = False
            response.error_message = f"[E9003] 内部异常: {e}"
        
        return response
    
    def _get_status_callback(self, request, response):
        """GetStatus Service 回调"""
        try:
            response.ready = self._ready
            response.node_status = "ready" if self._ready else "initializing"
            
            response.segmentation_ready = self._segmentation.is_ready
            response.pointcloud_ready = self._pointcloud.is_ready
            response.vectorizer_ready = self._vectorizer.is_ready
            response.storage_ready = self._storage.is_ready
            
            # 抓取模块状态（如果 GetStatus.srv 中有该字段）
            # response.grasp_ready = self._grasp.is_ready
            
            # GPU 内存（尝试获取）
            try:
                import torch
                if torch.cuda.is_available():
                    response.gpu_memory_used_mb = int(torch.cuda.memory_allocated() / 1024 / 1024)
                    response.gpu_memory_total_mb = int(torch.cuda.get_device_properties(0).total_memory / 1024 / 1024)
            except:
                pass
            
            # 索引统计
            stats = self._storage.get_stats()
            response.total_objects = stats.get('total_objects', 0)
            response.total_samples = stats.get('total_samples', 0)
            
            # 运行统计
            response.uptime_seconds = int(time.time() - self._start_time)
            response.requests_processed = self._requests_processed
            response.requests_failed = self._requests_failed
            
        except Exception as e:
            self.get_logger().error(f"GetStatus 错误: {e}")
        
        return response
    
    def _grasp_callback(self, request, response):
        """
        Grasp Service 回调
        
        接收点云数据，返回抓取候选位姿
        """
        start_time = time.time()
        
        try:
            # 检查模块是否就绪
            if not self._grasp.is_ready:
                raise PerceptionError(ErrorCode.NODE_NOT_READY, "抓取模块未就绪")
            
            # 解析点云数据
            point_cloud = self._parse_pointcloud2(request.point_cloud)
            
            if point_cloud is None or point_cloud.shape[0] == 0:
                raise PerceptionError(ErrorCode.POINT_COUNT_INSUFFICIENT, "点云数据为空")
            
            self.get_logger().debug(
                f"[grasp] 收到点云: {point_cloud.shape[0]} 点"
            )
            
            # 解析可选的掩码
            mask = None
            if request.mask.data and len(request.mask.data) > 0:
                mask_img = self._bridge.imgmsg_to_cv2(request.mask, 'mono8')
                mask = mask_img.flatten() > 0  # 转为布尔掩码
            
            # 解析参数
            max_candidates = request.max_candidates if request.max_candidates > 0 else DEFAULT_MAX_CANDIDATES
            min_confidence = request.min_confidence if request.min_confidence > 0 else DEFAULT_MIN_GRASP_CONFIDENCE
            
            # 执行推理（同步等待异步结果）
            # 使用 asyncio.run 创建新的事件循环来等待结果
            grasp_result = asyncio.run(
                self._grasp.predict(
                    point_cloud=point_cloud,
                    mask=mask,
                    max_candidates=max_candidates,
                    min_confidence=min_confidence
                )
            )
            
            # 构造响应
            response.success = True
            response.candidates = [
                self._grasp_candidate_to_msg(c) for c in grasp_result.candidates
            ]
            response.candidate_count = len(grasp_result.candidates)
            response.inference_time_ms = grasp_result.inference_time_ms
            response.error_message = ""
            
            self._requests_processed += 1
            
            total_time = (time.time() - start_time) * 1000
            self.get_logger().info(
                f"[grasp] 完成: {len(grasp_result.candidates)} 候选, "
                f"推理={grasp_result.inference_time_ms:.1f}ms, 总计={total_time:.1f}ms"
            )
            
        except PerceptionError as e:
            response.success = False
            response.error_message = e.to_error_message()
            response.candidates = []
            response.candidate_count = 0
            response.inference_time_ms = 0.0
            self._requests_failed += 1
            self.get_logger().error(f"[grasp] 错误: {e.to_error_message()}")
            
        except Exception as e:
            response.success = False
            response.error_message = f"[E9003] 内部异常: {e}"
            response.candidates = []
            response.candidate_count = 0
            response.inference_time_ms = 0.0
            self._requests_failed += 1
            self.get_logger().error(f"[grasp] 内部异常: {e}")
        
        return response
    
    def _parse_pointcloud2(self, msg: PointCloud2) -> Optional[np.ndarray]:
        """
        解析 PointCloud2 消息为 numpy 数组
        
        Args:
            msg: sensor_msgs/PointCloud2 消息
            
        Returns:
            np.ndarray: Nx3 点云数组 (x, y, z)，如果解析失败返回 None
        """
        import struct
        
        if msg.width == 0 or msg.height == 0:
            return None
        
        # 计算点数
        point_count = msg.width * msg.height
        
        # 查找 x, y, z 字段的偏移量
        field_offsets = {}
        for field in msg.fields:
            if field.name in ('x', 'y', 'z'):
                field_offsets[field.name] = field.offset
        
        if not all(k in field_offsets for k in ('x', 'y', 'z')):
            self.get_logger().error("PointCloud2 缺少 x, y, z 字段")
            return None
        
        # 解析点云数据
        points = np.zeros((point_count, 3), dtype=np.float32)
        data = msg.data
        point_step = msg.point_step
        
        for i in range(point_count):
            base = i * point_step
            try:
                points[i, 0] = struct.unpack_from('f', data, base + field_offsets['x'])[0]
                points[i, 1] = struct.unpack_from('f', data, base + field_offsets['y'])[0]
                points[i, 2] = struct.unpack_from('f', data, base + field_offsets['z'])[0]
            except struct.error:
                continue
        
        # 过滤无效点（NaN 或无穷大）
        valid_mask = np.isfinite(points).all(axis=1)
        points = points[valid_mask]
        
        return points
    
    def _grasp_candidate_to_msg(self, candidate) -> GraspCandidate:
        """
        将 GraspCandidate 数据类转换为 ROS 消息
        
        Args:
            candidate: grasp_module.GraspCandidate 实例
            
        Returns:
            GraspCandidate: ROS 消息
        """
        from geometry_msgs.msg import Point, Quaternion
        
        msg = GraspCandidate()
        
        # 位置
        msg.position = Point(
            x=float(candidate.position[0]),
            y=float(candidate.position[1]),
            z=float(candidate.position[2])
        )
        
        # 姿态四元数 (x, y, z, w)
        msg.orientation = Quaternion(
            x=float(candidate.orientation[0]),
            y=float(candidate.orientation[1]),
            z=float(candidate.orientation[2]),
            w=float(candidate.orientation[3])
        )
        
        # 参数
        msg.width = float(candidate.width)
        msg.confidence = float(candidate.confidence)
        
        # 变换矩阵（展平为 16 个 float64）
        msg.transform_matrix = [float(x) for x in candidate.transform_matrix.flatten().tolist()]
        
        return msg
    
    # =========================================================================
    # 辅助方法
    # =========================================================================
    
    def _to_object_info_msg(self, info) -> ObjectInfo:
        """转换为 ObjectInfo 消息"""
        msg = ObjectInfo()
        msg.object_id = info.object_id
        msg.label = info.label
        msg.description = info.description
        msg.created_at = info.created_at
        msg.updated_at = info.updated_at
        msg.sample_count = info.sample_count
        return msg
    
    def _to_sample_info_msg(self, info) -> SampleInfo:
        """转换为 SampleInfo 消息"""
        msg = SampleInfo()
        msg.sample_id = info.sample_id
        msg.object_id = info.object_id
        msg.created_at = info.created_at
        return msg

    def _wrap_async_execute(self, coro_func):
        """
        将 async execute 包装为同步回调，供 ActionServer 调用。
        
        MultiThreadedExecutor 不原生支持 async def 回调，
        需要使用 asyncio.run() 创建事件循环来执行协程。
        """
        def wrapper(goal_handle):
            return asyncio.run(coro_func(goal_handle))
        return wrapper

    def destroy_node(self):
        """销毁节点并关闭异步工作线程。"""
        try:
            if hasattr(self, '_worker') and self._worker:
                self._worker.shutdown()
        except Exception as exc:
            self.get_logger().warn(f"关闭 AsyncWorker 失败: {exc}")
        return super().destroy_node()


def main(args=None):
    """节点入口"""
    rclpy.init(args=args)
    
    node = PerceptionNode()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
