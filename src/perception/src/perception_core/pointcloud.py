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

import cv2

from .constants import (
    MIN_POINT_COUNT,
    MAX_INVALID_DEPTH_RATIO,
    MIN_DEPTH_MM,
    MAX_DEPTH_MM,
    DEFAULT_OUTPUT_FRAME_ID,
    TIMEOUT_POINTCLOUD,
    EDGE_FILTER_ENABLED,
    MASK_EROSION_PIXELS,
    DEPTH_GRADIENT_FILTER_ENABLED,
    DEPTH_GRADIENT_THRESHOLD,
    DEPTH_OUTLIER_FILTER_ENABLED,
    DEPTH_OUTLIER_MAD_RATIO,
    DEPTH_MEDIAN_FILTER_ENABLED,
    DEPTH_MEDIAN_FILTER_SIZE,
    DEPTH_3D_MAD_FILTER_ENABLED,
    DEPTH_3D_MAD_RATIO,
    WEIGHTED_CENTER_ENABLED,
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
        
        # 边缘深度过滤配置（解决 RealSense flying pixels 问题）
        # 三重过滤：Mask 腐蚀 → 深度梯度过滤 → 中位数+MAD 统计过滤
        self._edge_filter_enabled = config.get('edge_filter_enabled', EDGE_FILTER_ENABLED)
        
        # Step 1: Mask 腐蚀
        self._mask_erosion_pixels = config.get('mask_erosion_pixels', MASK_EROSION_PIXELS)
        
        # Step 2: 深度梯度过滤
        self._depth_gradient_filter_enabled = config.get('depth_gradient_filter_enabled', DEPTH_GRADIENT_FILTER_ENABLED)
        self._depth_gradient_threshold = config.get('depth_gradient_threshold', DEPTH_GRADIENT_THRESHOLD)
        
        # Step 3: 中位数+MAD 统计过滤
        self._depth_outlier_filter_enabled = config.get('depth_outlier_filter_enabled', DEPTH_OUTLIER_FILTER_ENABLED)
        self._depth_outlier_mad_ratio = config.get('depth_outlier_mad_ratio', DEPTH_OUTLIER_MAD_RATIO)
        
        # 深度图预处理
        self._depth_median_filter_enabled = config.get('depth_median_filter_enabled', DEPTH_MEDIAN_FILTER_ENABLED)
        self._depth_median_filter_size = config.get('depth_median_filter_size', DEPTH_MEDIAN_FILTER_SIZE)
        
        # 3D空间MAD过滤（增强版）
        self._depth_3d_mad_filter_enabled = config.get('depth_3d_mad_filter_enabled', DEPTH_3D_MAD_FILTER_ENABLED)
        self._depth_3d_mad_ratio = config.get('depth_3d_mad_ratio', DEPTH_3D_MAD_RATIO)
        
        # 加权质心
        self._weighted_center_enabled = config.get('weighted_center_enabled', WEIGHTED_CENTER_ENABLED)
        
        logger.info(
            f"PointCloudModule 配置已加载 - 三项优化: "
            f"[优化1]深度图预处理={self._depth_median_filter_enabled}(size={self._depth_median_filter_size}), "
            f"[优化2]3D空间MAD={self._depth_3d_mad_filter_enabled}(ratio={self._depth_3d_mad_ratio}), "
            f"[优化3]加权质心={self._weighted_center_enabled} | "
            f"基础过滤: [腐蚀={self._mask_erosion_pixels}px, 梯度={self._depth_gradient_filter_enabled}]"
        )
    
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
        """
        计算点云（同步方法，在线程池中执行）
        
        三重过滤流程：
        1. Mask 腐蚀：排除边缘像素
        2. 深度梯度过滤：排除 2D 深度图中的深度跳变区域
        3. 中位数+MAD 统计过滤：排除 Z 轴上的异常深度值
        """
        h, w = depth_image.shape[:2]
        
        # 确保深度图是单通道
        if depth_image.ndim == 3:
            depth_image = depth_image[:, :, 0]
        
        # =====================================================================
        # 优化1: 深度图预处理 - 中值滤波（在反投影前减少噪声）
        # =====================================================================
        if self._depth_median_filter_enabled:
            # 转换为uint16进行中值滤波（OpenCV要求）
            depth_uint16 = depth_image.astype(np.uint16)
            # 确保核大小为奇数
            ksize = self._depth_median_filter_size
            if ksize % 2 == 0:
                ksize += 1
            depth_filtered = cv2.medianBlur(depth_uint16, ksize)
            depth = depth_filtered.astype(np.float32)
            logger.info(f"[优化1-深度图预处理] 中值滤波完成 (核大小={ksize})")
        else:
            # 转换深度图类型
            depth = depth_image.astype(np.float32)
        
        # 获取掩码内的有效像素
        mask_bool = mask > 0
        original_mask_pixels = int(np.sum(mask_bool))
        
        # =====================================================================
        # 三重过滤 Step 1: Mask 腐蚀（排除边缘像素）
        # =====================================================================
        if self._edge_filter_enabled and self._mask_erosion_pixels > 0:
            mask_bool = self._erode_mask(mask_bool, self._mask_erosion_pixels)
            after_erosion_pixels = int(np.sum(mask_bool))
            logger.info(
                f"[三重过滤 1/3] Mask 腐蚀: {original_mask_pixels:,} -> {after_erosion_pixels:,} 像素 "
                f"(排除 {original_mask_pixels - after_erosion_pixels:,} 边缘像素)"
            )
        else:
            after_erosion_pixels = original_mask_pixels
        
        # =====================================================================
        # 三重过滤 Step 2: 深度梯度过滤（排除深度跳变区域）
        # =====================================================================
        if self._edge_filter_enabled and self._depth_gradient_filter_enabled:
            gradient_mask = self._compute_depth_gradient_mask(
                depth, mask_bool, self._depth_gradient_threshold
            )
            mask_bool = mask_bool & gradient_mask
            after_gradient_pixels = int(np.sum(mask_bool))
            logger.info(
                f"[三重过滤 2/3] 深度梯度过滤: {after_erosion_pixels:,} -> {after_gradient_pixels:,} 像素 "
                f"(排除 {after_erosion_pixels - after_gradient_pixels:,} 深度跳变像素)"
            )
        else:
            after_gradient_pixels = after_erosion_pixels
        
        # 深度有效性检查
        valid_depth = (depth >= self._min_depth_mm) & (depth <= self._max_depth_mm)
        valid_mask = mask_bool & valid_depth
        
        # 检查无效点比例
        total_mask_pixels = int(np.sum(mask_bool))
        valid_pixels = int(np.sum(valid_mask))
        
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
        points_before_mad_filter = len(points)
        
        # =====================================================================
        # 优化2: 3D空间MAD过滤（排除空间异常值）
        # =====================================================================
        points_after_mad_filter = points_before_mad_filter
        
        if len(points) > 0:
            # 优先使用3D空间MAD过滤（更全面，同时检测X/Y/Z方向）
            if self._depth_3d_mad_filter_enabled:
                points = self._filter_3d_outliers_mad(points, self._depth_3d_mad_ratio)
                points_after_mad_filter = len(points)
                logger.info(
                    f"[优化2-3D空间MAD过滤] {points_before_mad_filter:,} -> {points_after_mad_filter:,} 点 "
                    f"(排除 {points_before_mad_filter - points_after_mad_filter:,} 空间异常点, ratio={self._depth_3d_mad_ratio})"
                )
            # 否则使用Z轴MAD过滤（向后兼容）
            elif self._edge_filter_enabled and self._depth_outlier_filter_enabled:
                points = self._filter_depth_outliers_mad(points, self._depth_outlier_mad_ratio)
                points_after_mad_filter = len(points)
                logger.info(
                    f"[三重过滤 3/3] Z轴MAD过滤: {points_before_mad_filter:,} -> {points_after_mad_filter:,} 点 "
                    f"(排除 {points_before_mad_filter - points_after_mad_filter:,} Z轴异常点)"
                )
        
        if len(points) < self._min_point_count:
            raise PerceptionError(
                ErrorCode.POINT_COUNT_INSUFFICIENT,
                f"过滤后有效点数 {len(points)} < {self._min_point_count}"
            )
        
        # =====================================================================
        # 优化3: 加权质心计算（提升质心精度）
        # =====================================================================
        if self._weighted_center_enabled:
            center_x, center_y, center_z = self._compute_weighted_center(points)
            logger.debug(f"[优化3-加权质心] 使用距离加权计算质心")
        else:
            center_x = float(np.mean(points[:, 0]))
            center_y = float(np.mean(points[:, 1]))
            center_z = float(np.mean(points[:, 2]))
        
        # 计算边界框
        bbox_min = (float(np.min(points[:, 0])), float(np.min(points[:, 1])), float(np.min(points[:, 2])))
        bbox_max = (float(np.max(points[:, 0])), float(np.max(points[:, 1])), float(np.max(points[:, 2])))
        
        # 计算最终尺寸用于日志
        size_mm = (
            (bbox_max[0] - bbox_min[0]) * 1000,
            (bbox_max[1] - bbox_min[1]) * 1000,
            (bbox_max[2] - bbox_min[2]) * 1000,
        )
        
        # 创建 PointCloud2 消息
        point_cloud = self._create_pointcloud2(points, frame_id)
        
        logger.info(
            f"[点云计算完成] {len(points):,} 点, "
            f"尺寸={size_mm[0]:.1f}×{size_mm[1]:.1f}×{size_mm[2]:.1f}mm, "
            f"质心=({center_x:.3f}, {center_y:.3f}, {center_z:.3f})m"
        )
        
        return PointCloudResult(
            point_cloud=point_cloud,
            center_3d=(center_x, center_y, center_z),
            bbox_min=bbox_min,
            bbox_max=bbox_max,
            point_count=len(points)
        )
    
    def _erode_mask(self, mask_bool: np.ndarray, erosion_pixels: int) -> np.ndarray:
        """
        对 mask 进行形态学腐蚀，排除边缘像素
        
        Args:
            mask_bool: 布尔掩码
            erosion_pixels: 腐蚀像素数
            
        Returns:
            腐蚀后的布尔掩码
        """
        if erosion_pixels <= 0:
            return mask_bool
        
        # 转换为 uint8 用于 OpenCV 操作
        mask_uint8 = mask_bool.astype(np.uint8) * 255
        
        # 创建腐蚀核（圆形核效果更好）
        kernel_size = erosion_pixels * 2 + 1
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, 
            (kernel_size, kernel_size)
        )
        
        # 执行腐蚀
        eroded = cv2.erode(mask_uint8, kernel, iterations=1)
        
        return eroded > 0
    
    def _compute_depth_gradient_mask(
        self, 
        depth: np.ndarray, 
        mask_bool: np.ndarray, 
        threshold: float
    ) -> np.ndarray:
        """
        计算深度梯度掩码，排除深度跳变区域
        
        使用 Sobel 算子检测深度图中的梯度，排除梯度过大的像素
        
        Args:
            depth: 深度图 (H, W)，单位 mm
            mask_bool: 布尔掩码
            threshold: 梯度阈值 (mm)
            
        Returns:
            布尔掩码，True 表示深度连续（保留），False 表示深度跳变（排除）
        """
        # 只在 mask 区域内计算梯度
        # 先将 mask 外的深度设为 0，避免边界影响
        depth_masked = depth.copy()
        depth_masked[~mask_bool] = 0
        
        # 使用 Sobel 算子计算梯度
        # ksize=3 是 3x3 的 Sobel 核
        grad_x = cv2.Sobel(depth_masked, cv2.CV_32F, 1, 0, ksize=3)
        grad_y = cv2.Sobel(depth_masked, cv2.CV_32F, 0, 1, ksize=3)
        
        # 计算梯度幅值
        gradient_magnitude = np.sqrt(grad_x**2 + grad_y**2)
        
        # 梯度小于阈值的像素认为是深度连续的
        gradient_mask = gradient_magnitude < threshold
        
        # 只在 mask 区域内应用
        gradient_mask = gradient_mask | (~mask_bool)
        
        # 统计信息
        if logger.isEnabledFor(logging.DEBUG):
            masked_gradients = gradient_magnitude[mask_bool]
            if len(masked_gradients) > 0:
                logger.debug(
                    f"[深度梯度] 梯度统计: "
                    f"max={np.max(masked_gradients):.1f}mm, "
                    f"mean={np.mean(masked_gradients):.1f}mm, "
                    f"median={np.median(masked_gradients):.1f}mm, "
                    f"threshold={threshold:.1f}mm"
                )
        
        return gradient_mask
    
    def _filter_depth_outliers_mad(self, points: np.ndarray, mad_ratio: float) -> np.ndarray:
        """
        使用中位数 + MAD（中位数绝对偏差）过滤深度异常值
        
        MAD 比标准差对异常值更鲁棒：
        - 中位数不受异常值影响
        - MAD = median(|X - median(X)|) 也不受异常值影响
        
        Args:
            points: Nx3 点云数组
            mad_ratio: MAD 倍数阈值（推荐 2.5-3.5）
            
        Returns:
            过滤后的点云数组
        """
        if len(points) == 0:
            return points
        
        z_values = points[:, 2]
        
        # 计算中位数
        z_median = np.median(z_values)
        
        # 计算 MAD（中位数绝对偏差）
        z_mad = np.median(np.abs(z_values - z_median))
        
        if z_mad < 1e-6:
            # MAD 太小，所有点深度几乎相同，不过滤
            return points
        
        # 计算有效范围
        z_min_valid = z_median - mad_ratio * z_mad
        z_max_valid = z_median + mad_ratio * z_mad
        
        # 过滤
        valid_mask = (z_values >= z_min_valid) & (z_values <= z_max_valid)
        filtered_points = points[valid_mask]
        
        logger.debug(
            f"[MAD过滤] Z范围: [{z_min_valid:.3f}, {z_max_valid:.3f}]m "
            f"(中位数={z_median:.3f}m, MAD={z_mad:.4f}m, ratio={mad_ratio})"
        )
        
        return filtered_points
    
    def _filter_3d_outliers_mad(self, points: np.ndarray, mad_ratio: float) -> np.ndarray:
        """
        使用3D空间MAD（中位数绝对偏差）过滤空间异常值
        
        相比仅Z轴过滤，3D空间MAD能同时检测X、Y、Z方向的异常点
        通过计算每个点到质心的3D距离，然后对距离做MAD过滤
        
        Args:
            points: Nx3 点云数组
            mad_ratio: MAD 倍数阈值（推荐 2.5-3.5）
            
        Returns:
            过滤后的点云数组
        """
        if len(points) == 0:
            return points
        
        # 使用中位数作为初始质心（比均值更鲁棒）
        center_3d = np.median(points, axis=0)
        
        # 计算每个点到质心的3D欧氏距离
        distances = np.linalg.norm(points - center_3d, axis=1)
        
        # 计算距离的中位数
        dist_median = np.median(distances)
        
        # 计算距离的MAD（中位数绝对偏差）
        dist_mad = np.median(np.abs(distances - dist_median))
        
        if dist_mad < 1e-6:
            # MAD 太小，所有点距离几乎相同，不过滤
            return points
        
        # 计算有效距离范围
        dist_max_valid = dist_median + mad_ratio * dist_mad
        
        # 过滤：保留距离在有效范围内的点
        valid_mask = distances <= dist_max_valid
        filtered_points = points[valid_mask]
        
        logger.debug(
            f"[3D-MAD过滤] 距离范围: [0, {dist_max_valid:.4f}]m "
            f"(中位数={dist_median:.4f}m, MAD={dist_mad:.4f}m, ratio={mad_ratio}), "
            f"过滤后保留 {len(filtered_points)}/{len(points)} 点"
        )
        
        return filtered_points
    
    def _compute_weighted_center(self, points: np.ndarray) -> Tuple[float, float, float]:
        """
        计算加权质心
        
        使用距离加权：距离初始质心越近的点权重越大
        这样可以减少异常点对质心计算的影响
        
        Args:
            points: Nx3 点云数组
            
        Returns:
            加权质心坐标 (x, y, z)
        """
        if len(points) == 0:
            return 0.0, 0.0, 0.0
        
        # 使用中位数作为初始质心（比均值更鲁棒）
        center_initial = np.median(points, axis=0)
        
        # 计算每个点到初始质心的距离
        distances = np.linalg.norm(points - center_initial, axis=1)
        
        # 计算权重：距离越近权重越大（使用倒数，加小值避免除零）
        # 也可以使用高斯权重：weights = np.exp(-distances**2 / (2 * sigma**2))
        weights = 1.0 / (distances + 1e-6)
        
        # 归一化权重
        weights = weights / np.sum(weights)
        
        # 计算加权平均
        center_weighted = np.average(points, axis=0, weights=weights)
        
        logger.debug(
            f"[加权质心] 初始质心={center_initial}, 加权质心={center_weighted}, "
            f"平均距离={np.mean(distances):.4f}m"
        )
        
        return float(center_weighted[0]), float(center_weighted[1]), float(center_weighted[2])
    
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
