"""
SegmentationModule - 分割模块

封装 SAM2，提供图像分割能力
参考: L1/l1_stage2_segmentation/l1_stage2_segmentation/perception/sam2_engine.py
"""

import asyncio
import logging
import time
import threading
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple, Dict, Any

import cv2
import numpy as np

from .constants import (
    MIN_CONFIDENCE,
    MIN_MASK_AREA,
    MAX_MASK_AREA_RATIO,
    SAM2_CONFIG_FILENAME,
    TIMEOUT_SEGMENT,
)
from .error_codes import PerceptionError, ErrorCode
from .async_worker import AsyncWorker

# 延迟导入 SAM2Engine
SAM2_AVAILABLE = False
SAM2_IMPORT_ERROR = None

try:
    from .sam2_engine import SAM2Engine
    SAM2_AVAILABLE = True
except ImportError as exc:
    SAM2_IMPORT_ERROR = exc

logger = logging.getLogger(__name__)


@dataclass
class SegmentResult:
    """分割结果"""
    mask: np.ndarray              # 分割掩码 (H, W)，uint8，255=目标，0=背景
    cropped_image: np.ndarray     # 裁剪后的目标图像 (H', W', 3)
    confidence: float             # 置信度 (0.0-1.0)
    mask_area: int                # 掩码面积（像素数）
    bbox: Tuple[int, int, int, int]  # 边界框 (x, y, w, h)


class SegmentationModule:
    """
    分割模块
    
    封装 SAM2 引擎，提供基于点击的交互式分割能力
    """
    
    def __init__(self, config: Dict[str, Any], worker: AsyncWorker):
        """
        初始化配置
        
        Args:
            config: 配置字典，包含:
                - config_path: SAM2 配置文件路径
                - models_dir: 模型目录路径
                - min_confidence: 最小置信度阈值
                - min_mask_area: 最小掩码面积
            worker: AsyncWorker 实例
        """
        if worker is None:
            raise ValueError("SegmentationModule 需要有效的 AsyncWorker 实例")
        self._config = config
        self._worker = worker
        self._ready = False
        self._engine: Optional[SAM2Engine] = None
        self._engine_lock = threading.Lock()
        
        # 配置参数
        self._min_confidence = config.get('min_confidence', MIN_CONFIDENCE)
        self._min_mask_area = config.get('min_mask_area', MIN_MASK_AREA)
        self._max_mask_area_ratio = config.get('max_mask_area_ratio', MAX_MASK_AREA_RATIO)
        self._segment_timeout = config.get('timeout', TIMEOUT_SEGMENT)
        
        # 性能统计
        self._segment_times = []
        
        logger.info("SegmentationModule 配置已加载")
    
    @property
    def is_ready(self) -> bool:
        """检查模块是否就绪"""
        return self._ready
    
    async def initialize(self) -> bool:
        """
        异步初始化，加载 SAM2 引擎
        
        Returns:
            bool: 是否初始化成功
            
        Raises:
            PerceptionError: 初始化失败时抛出
        """
        try:
            return await self._worker.run_callable(self._initialize_sync)
        except PerceptionError:
            raise
        except Exception as e:
            logger.error(f"SegmentationModule 初始化失败: {e}")
            raise PerceptionError(
                ErrorCode.SAM2_ENGINE_LOAD_FAILED,
                str(e)
            )

    def _initialize_sync(self) -> bool:
        """实际的 SAM2 加载流程，在 AsyncWorker 线程中执行。"""
        start_time = time.time()
        
        # 检查 SAM2 是否可用
        if not SAM2_AVAILABLE:
            raise PerceptionError(
                ErrorCode.SAM2_ENGINE_LOAD_FAILED,
                f"SAM2 引擎不可用: {SAM2_IMPORT_ERROR}"
            )
        
        # 获取配置路径
        config_path = self._config.get('config_path')
        models_dir = self._config.get('models_dir')
        
        if not config_path:
            raise PerceptionError(
                ErrorCode.SAM2_ENGINE_LOAD_FAILED,
                "未指定 SAM2 配置文件路径"
            )
        
        config_path = Path(config_path)
        if not config_path.exists():
            raise PerceptionError(
                ErrorCode.SAM2_ENGINE_LOAD_FAILED,
                f"SAM2 配置文件不存在: {config_path}"
            )
        
        logger.info(f"加载 SAM2 配置: {config_path}")
        logger.info(f"模型目录: {models_dir}")
        
        # 设置环境变量，让 SAM2Engine 能找到模型文件
        import os
        if models_dir:
            os.environ['PERCEPTION_MODELS_DIR'] = str(models_dir)
        
        # 加载 SAM2 引擎
        load_start = time.time()
        try:
            self._engine = SAM2Engine(
                config_path=str(config_path),
                device='cuda',
                package_share_dir=Path(models_dir) if models_dir else None
            )
        except Exception as e:
            raise PerceptionError(
                ErrorCode.SAM2_ENGINE_LOAD_FAILED,
                f"SAM2 引擎初始化失败: {e}"
            )
        
        load_time = time.time() - load_start
        logger.info(f"SAM2 引擎加载完成 (耗时={load_time:.2f}s)")
        
        self._ready = True
        total_time = time.time() - start_time
        logger.info(f"SegmentationModule 初始化完成 (总耗时={total_time:.2f}s)")
        return True
    
    async def segment(
        self,
        image: np.ndarray,
        click_x: float,
        click_y: float
    ) -> SegmentResult:
        """
        执行分割
        
        Args:
            image: RGB 图像 (H, W, 3)
            click_x: 点击坐标 X（像素）
            click_y: 点击坐标 Y（像素）
            
        Returns:
            SegmentResult: 分割结果
            
        Raises:
            PerceptionError: 分割失败时抛出
        """
        if not self._ready:
            raise PerceptionError(ErrorCode.NODE_NOT_READY, "分割模块未就绪")
        
        try:
            return await asyncio.wait_for(
                self._worker.run_callable(
                    self._segment_sync,
                    image,
                    click_x,
                    click_y
                ),
                timeout=self._segment_timeout,
            )
        except asyncio.TimeoutError as exc:
            raise PerceptionError(
                ErrorCode.SAM2_INFERENCE_TIMEOUT,
                f"分割超时 (> {self._segment_timeout:.1f}s)"
            ) from exc

    def _segment_sync(
        self,
        image: np.ndarray,
        click_x: float,
        click_y: float
    ) -> SegmentResult:
        """同步执行分割逻辑（运行在 AsyncWorker 的线程池中）。"""
        if image is None or image.size == 0:
            raise PerceptionError(ErrorCode.IMAGE_EMPTY)
        
        if image.ndim != 3 or image.shape[2] != 3:
            raise PerceptionError(
                ErrorCode.IMAGE_FORMAT_UNSUPPORTED,
                f"期望 (H, W, 3)，实际 {image.shape}"
            )
        
        h, w = image.shape[:2]
        if not (0 <= click_x < w and 0 <= click_y < h):
            raise PerceptionError(
                ErrorCode.CLICK_OUT_OF_RANGE,
                f"点击坐标 ({click_x}, {click_y}) 超出图像范围 ({w}, {h})"
            )
        
        start_time = time.time()
        
        with self._engine_lock:
            # 使用 SAM2Engine 执行分割
            try:
                # 编码图像
                self._engine.encode_image(image)
                
                # 解码掩码
                point_coords = np.array([[click_x, click_y]], dtype=np.float32)
                point_labels = np.array([1], dtype=np.int32)  # 1 = foreground
                mask, confidence = self._engine.decode_mask(point_coords, point_labels)
            except Exception as e:
                raise PerceptionError(
                    ErrorCode.SAM2_INFERENCE_ERROR,
                    f"SAM2 推理失败: {e}"
                )
        
        segment_time = time.time() - start_time
        self._segment_times.append(segment_time)
        
        # 确保掩码是二值的
        mask_binary = (mask > 0).astype(np.uint8) * 255
        
        # 后处理
        mask_binary = self._post_process_mask(mask_binary)
        
        mask_area = int(np.sum(mask_binary > 0))
        total_pixels = h * w
        
        if mask_area == 0:
            raise PerceptionError(ErrorCode.SEGMENT_RESULT_EMPTY)
        
        if confidence < self._min_confidence:
            raise PerceptionError(
                ErrorCode.CONFIDENCE_TOO_LOW,
                f"置信度 {confidence:.3f} < {self._min_confidence}"
            )
        
        if mask_area < self._min_mask_area:
            raise PerceptionError(
                ErrorCode.MASK_AREA_TOO_SMALL,
                f"掩码面积 {mask_area} < {self._min_mask_area}"
            )
        
        area_ratio = mask_area / total_pixels
        if area_ratio > self._max_mask_area_ratio:
            raise PerceptionError(
                ErrorCode.MASK_AREA_TOO_LARGE,
                f"掩码面积占比 {area_ratio:.2%} > {self._max_mask_area_ratio:.2%}"
            )
        
        cropped_image, bbox = self._crop_by_mask(image, mask_binary)
        
        logger.debug(
            f"分割完成: confidence={confidence:.3f}, area={mask_area}, "
            f"time={segment_time*1000:.1f}ms"
        )
        
        return SegmentResult(
            mask=mask_binary,
            cropped_image=cropped_image,
            confidence=confidence,
            mask_area=mask_area,
            bbox=bbox
        )
    
    def _post_process_mask(self, mask: np.ndarray) -> np.ndarray:
        """后处理掩码"""
        # 形态学操作：闭运算填充小孔
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # 保留最大连通区域
        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(
            mask, connectivity=8
        )
        if num_labels > 1:
            largest_label = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])
            mask = (labels == largest_label).astype(np.uint8) * 255
        
        # 平滑边界
        mask = cv2.GaussianBlur(mask, (3, 3), 0)
        mask = (mask > 127).astype(np.uint8) * 255
        
        return mask
    
    def _crop_by_mask(
        self,
        image: np.ndarray,
        mask: np.ndarray
    ) -> Tuple[np.ndarray, Tuple[int, int, int, int]]:
        """根据掩码裁剪图像"""
        # 找到掩码边界框
        coords = cv2.findNonZero(mask)
        if coords is None:
            return image.copy(), (0, 0, image.shape[1], image.shape[0])
        
        x, y, w, h = cv2.boundingRect(coords)
        
        # 裁剪
        cropped = image[y:y+h, x:x+w].copy()
        
        # 应用掩码（背景设为黑色）
        mask_crop = mask[y:y+h, x:x+w]
        cropped[mask_crop == 0] = 0
        
        return cropped, (x, y, w, h)
    
    def create_visualization(
        self,
        image: np.ndarray,
        mask: np.ndarray,
        click_point: Tuple[float, float],
        mask_color: Tuple[int, int, int] = (0, 255, 0),
        mask_alpha: float = 0.5,
        point_color: Tuple[int, int, int] = (255, 0, 0),
        point_radius: int = 10
    ) -> np.ndarray:
        """
        生成可视化叠加图
        
        Args:
            image: 原始 RGB 图像
            mask: 分割掩码
            click_point: 点击坐标 (x, y)
            mask_color: 掩码颜色 (R, G, B)
            mask_alpha: 掩码透明度
            point_color: 点击点颜色 (R, G, B)
            point_radius: 点击点半径
            
        Returns:
            np.ndarray: 可视化图像 (H, W, 3)
        """
        vis = image.copy()
        
        # 叠加掩码
        mask_overlay = np.zeros_like(vis)
        mask_overlay[mask > 0] = mask_color
        vis = cv2.addWeighted(vis, 1.0, mask_overlay, mask_alpha, 0)
        
        # 绘制掩码边界
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        cv2.drawContours(vis, contours, -1, mask_color, 2)
        
        # 绘制点击点
        click_x, click_y = int(click_point[0]), int(click_point[1])
        cv2.circle(vis, (click_x, click_y), point_radius, point_color, -1)
        cv2.circle(vis, (click_x, click_y), point_radius + 2, (255, 255, 255), 2)
        
        return vis
    
    def get_performance_stats(self) -> Dict[str, float]:
        """获取性能统计"""
        stats = {}
        
        if self._segment_times:
            times = np.array(self._segment_times)
            stats['avg_segment_time_ms'] = float(np.mean(times) * 1000)
            stats['max_segment_time_ms'] = float(np.max(times) * 1000)
        
        # 从 SAM2Engine 获取更详细的统计
        if self._engine:
            engine_stats = self._engine.get_performance_stats()
            stats.update(engine_stats)
        
        return stats
