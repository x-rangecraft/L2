"""
PointCloudModule - 点云计算模块

深度图 → 3D 点云计算
"""

import asyncio
import logging
import struct
from dataclasses import dataclass
from typing import Dict, Any, Tuple, Optional

import numpy as np

from sensor_msgs.msg import PointCloud2, PointField, CameraInfo

from .constants import (
    MIN_POINT_COUNT,
    MAX_INVALID_DEPTH_RATIO,
    MIN_DEPTH_MM,
    MAX_DEPTH_MM,
    DEFAULT_OUTPUT_FRAME_ID,
    TIMEOUT_POINTCLOUD,
)
from .error_codes import PerceptionError, ErrorCode
from .async_worker import AsyncWorker

logger = logging.getLogger(__name__)


@dataclass
class PointCloudResult:
    """点云计算结果"""
    point_cloud: PointCloud2                    # 目标点云 (XYZ)
    center_3d: Tuple[float, float, float]       # 质心坐标 (x, y, z)
    bbox_min: Tuple[float, float, float]        # 边界框最小点
    bbox_max: Tuple[float, float, float]        # 边界框最大点
    point_count: int                            # 有效点数量


class PointCloudModule:
    """
    点云计算模块
    
    根据分割掩码和深度图，计算目标物体的 3D 点云
    """
    
    def __init__(self, config: Dict[str, Any], worker: AsyncWorker):
        """
        初始化配置
        
        Args:
            config: 配置字典，包含:
                - output_frame_id: 输出坐标系
                - min_point_count: 最小有效点数
                - min_depth_mm: 最小有效深度 (mm)
                - max_depth_mm: 最大有效深度 (mm)
            worker: AsyncWorker 实例
        """
        if worker is None:
            raise ValueError("PointCloudModule 需要有效的 AsyncWorker 实例")
        self._config = config
        self._worker = worker
        self._ready = False
        
        # 配置参数
        self._output_frame_id = config.get('output_frame_id', DEFAULT_OUTPUT_FRAME_ID)
        self._min_point_count = config.get('min_point_count', MIN_POINT_COUNT)
        self._min_depth_mm = config.get('min_depth_mm', MIN_DEPTH_MM)
        self._max_depth_mm = config.get('max_depth_mm', MAX_DEPTH_MM)
        self._max_invalid_ratio = config.get('max_invalid_depth_ratio', MAX_INVALID_DEPTH_RATIO)
        self._compute_timeout = config.get('timeout', TIMEOUT_POINTCLOUD)
        
        logger.info("PointCloudModule 配置已加载")
    
    @property
    def is_ready(self) -> bool:
        """检查模块是否就绪"""
        return self._ready
    
    async def initialize(self) -> bool:
        """
        异步初始化
        
        点云模块是轻量级的，初始化只需要验证配置
        
        Returns:
            bool: 是否初始化成功
        """
        try:
            # 验证配置
            if self._min_depth_mm >= self._max_depth_mm:
                raise PerceptionError(
                    ErrorCode.CONFIG_READ_FAILED,
                    f"深度范围无效: min={self._min_depth_mm}, max={self._max_depth_mm}"
                )
            
            self._ready = True
            logger.info("PointCloudModule 初始化完成")
            return True
            
        except PerceptionError:
            raise
        except Exception as e:
            logger.error(f"PointCloudModule 初始化失败: {e}")
            raise PerceptionError(ErrorCode.INTERNAL_ERROR, str(e))
    
    async def compute(
        self,
        mask: np.ndarray,
        depth_image: np.ndarray,
        camera_info: CameraInfo
    ) -> PointCloudResult:
        """
        计算点云
        
        Args:
            mask: 分割掩码 (H, W)，uint8，255=目标
            depth_image: 深度图 (H, W)，uint16，单位 mm
            camera_info: 相机内参
            
        Returns:
            PointCloudResult: 点云计算结果
            
        Raises:
            PerceptionError: 计算失败时抛出
        """
        if not self._ready:
            raise PerceptionError(ErrorCode.NODE_NOT_READY, "点云模块未就绪")
        
        # 验证输入
        if depth_image is None or depth_image.size == 0:
            raise PerceptionError(ErrorCode.DEPTH_IMAGE_EMPTY)
        
        if mask.shape[:2] != depth_image.shape[:2]:
            raise PerceptionError(
                ErrorCode.MASK_DEPTH_SIZE_MISMATCH,
                f"掩码 {mask.shape[:2]} != 深度图 {depth_image.shape[:2]}"
            )
        
        # 提取相机内参
        fx, fy, cx, cy = self._extract_intrinsics(camera_info)
        if fx <= 0 or fy <= 0:
            raise PerceptionError(
                ErrorCode.CAMERA_INFO_INVALID,
                f"相机内参无效: fx={fx}, fy={fy}"
            )
        
        frame_id = getattr(camera_info, 'header', None)
        frame_id = getattr(frame_id, 'frame_id', None) or self._output_frame_id

        try:
            return await asyncio.wait_for(
                self._worker.run_callable(
                    self._compute_pointcloud,
                    mask,
                    depth_image,
                    fx,
                    fy,
                    cx,
                    cy,
                    frame_id,
                ),
                timeout=self._compute_timeout,
            )
        except asyncio.TimeoutError as exc:
            raise PerceptionError(
                ErrorCode.POINTCLOUD_COMPUTE_TIMEOUT,
                f"超过 {self._compute_timeout:.1f}s 未完成"
            ) from exc
    
    def _extract_intrinsics(self, camera_info: CameraInfo) -> Tuple[float, float, float, float]:
        """从 CameraInfo 提取相机内参"""
        # K 矩阵格式: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        k = camera_info.k
        fx = k[0]
        fy = k[4]
        cx = k[2]
        cy = k[5]
        return fx, fy, cx, cy
    
    def _compute_pointcloud(
        self,
        mask: np.ndarray,
        depth_image: np.ndarray,
        fx: float,
        fy: float,
        cx: float,
        cy: float,
        frame_id: str,
    ) -> PointCloudResult:
        """计算点云（同步方法，在线程池中执行）"""
        h, w = depth_image.shape[:2]
        
        # 确保深度图是单通道
        if depth_image.ndim == 3:
            depth_image = depth_image[:, :, 0]
        
        # 转换深度图类型
        depth = depth_image.astype(np.float32)
        
        # 获取掩码内的有效像素
        mask_bool = mask > 0
        
        # 深度有效性检查
        valid_depth = (depth >= self._min_depth_mm) & (depth <= self._max_depth_mm)
        valid_mask = mask_bool & valid_depth
        
        # 检查无效点比例
        total_mask_pixels = np.sum(mask_bool)
        valid_pixels = np.sum(valid_mask)
        
        if total_mask_pixels > 0:
            invalid_ratio = 1.0 - (valid_pixels / total_mask_pixels)
            if invalid_ratio > self._max_invalid_ratio:
                raise PerceptionError(
                    ErrorCode.DEPTH_INVALID_RATIO_HIGH,
                    f"深度无效点比例 {invalid_ratio:.2%} > {self._max_invalid_ratio:.2%}"
                )
        
        if valid_pixels < self._min_point_count:
            raise PerceptionError(
                ErrorCode.POINT_COUNT_INSUFFICIENT,
                f"有效点数 {valid_pixels} < {self._min_point_count}"
            )
        
        # 获取有效像素坐标
        v_coords, u_coords = np.where(valid_mask)
        depths = depth[valid_mask]
        
        # 3D 反投影
        # Z = d (转换为米)
        # X = (u - cx) * Z / fx
        # Y = (v - cy) * Z / fy
        z = depths / 1000.0  # mm -> m
        x = (u_coords - cx) * z / fx
        y = (v_coords - cy) * z / fy
        
        # 组合点云
        points = np.stack([x, y, z], axis=1).astype(np.float32)
        
        # 计算质心
        center_x = float(np.mean(x))
        center_y = float(np.mean(y))
        center_z = float(np.mean(z))
        
        # 计算边界框
        bbox_min = (float(np.min(x)), float(np.min(y)), float(np.min(z)))
        bbox_max = (float(np.max(x)), float(np.max(y)), float(np.max(z)))
        
        # 创建 PointCloud2 消息
        point_cloud = self._create_pointcloud2(points, frame_id)
        
        logger.debug(
            f"点云计算完成: {len(points)} 点, "
            f"中心=({center_x:.3f}, {center_y:.3f}, {center_z:.3f})"
        )
        
        return PointCloudResult(
            point_cloud=point_cloud,
            center_3d=(center_x, center_y, center_z),
            bbox_min=bbox_min,
            bbox_max=bbox_max,
            point_count=len(points)
        )
    
    def _create_pointcloud2(self, points: np.ndarray, frame_id: str) -> PointCloud2:
        """
        创建 PointCloud2 消息
        
        Args:
            points: 点云数组 (N, 3)，float32
            
        Returns:
            PointCloud2: ROS 点云消息
        """
        msg = PointCloud2()
        msg.header.frame_id = frame_id
        
        # 设置字段
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        msg.is_bigendian = False
        msg.point_step = 12  # 3 * 4 bytes (float32)
        msg.height = 1
        msg.width = len(points)
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        
        # 序列化点云数据
        msg.data = points.tobytes()
        
        return msg
    
    @staticmethod
    def pointcloud2_to_array(msg: PointCloud2) -> np.ndarray:
        """
        将 PointCloud2 消息转换为 numpy 数组
        
        Args:
            msg: PointCloud2 消息
            
        Returns:
            np.ndarray: 点云数组 (N, 3)
        """
        # 解析数据
        data = np.frombuffer(msg.data, dtype=np.float32)
        points = data.reshape(-1, 3)
        return points
    
    def compute_from_depth_and_mask_sync(
        self,
        mask: np.ndarray,
        depth_image: np.ndarray,
        fx: float,
        fy: float,
        cx: float,
        cy: float
    ) -> Tuple[np.ndarray, Tuple[float, float, float], Tuple[float, float, float], Tuple[float, float, float]]:
        """
        同步计算点云（用于测试或简单场景）
        
        Returns:
            Tuple: (points, center_3d, bbox_min, bbox_max)
        """
        result = self._compute_pointcloud(
            mask,
            depth_image,
            fx,
            fy,
            cx,
            cy,
            self._output_frame_id,
        )
        points = self.pointcloud2_to_array(result.point_cloud)
        return points, result.center_3d, result.bbox_min, result.bbox_max
