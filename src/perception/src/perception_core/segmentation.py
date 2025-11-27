"""
SegmentationModule - 分割模块

封装 NanoSAM，提供图像分割能力
参考: L1/l1_stage2_segmentation/l1_stage2_segmentation/perception/nanosam_engine.py
"""

import asyncio
import logging
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple, Dict, Any

import cv2
import numpy as np

from .constants import (
    MIN_CONFIDENCE,
    MIN_MASK_AREA,
    MAX_MASK_AREA_RATIO,
    NANOSAM_ENCODER_FILENAME,
    NANOSAM_DECODER_FILENAME,
)
from .error_codes import PerceptionError, ErrorCode

# 延迟导入 NanoSAM（可能不可用）
NANOSAM_AVAILABLE = False
NANOSAM_IMPORT_ERROR = None

try:
    import torch
    from nanosam.utils.predictor import (
        load_image_encoder_engine,
        load_mask_decoder_engine,
        upscale_mask,
    )
    NANOSAM_AVAILABLE = True
except ImportError as exc:
    NANOSAM_IMPORT_ERROR = exc


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
    
    封装 NanoSAM TensorRT 引擎，提供基于点击的交互式分割能力
    """
    
    def __init__(self, config: Dict[str, Any]):
        """
        初始化配置
        
        Args:
            config: 配置字典，包含:
                - engine_path: NanoSAM 编码器引擎路径
                - decoder_path: NanoSAM 解码器引擎路径
                - min_confidence: 最小置信度阈值
                - min_mask_area: 最小掩码面积
        """
        self._config = config
        self._ready = False
        self._predictor = None
        self._engine_lock = asyncio.Lock()
        
        # 配置参数
        self._min_confidence = config.get('min_confidence', MIN_CONFIDENCE)
        self._min_mask_area = config.get('min_mask_area', MIN_MASK_AREA)
        self._max_mask_area_ratio = config.get('max_mask_area_ratio', MAX_MASK_AREA_RATIO)
        
        # 性能统计
        self._encode_times = []
        self._decode_times = []
        
        logger.info("SegmentationModule 配置已加载")
    
    @property
    def is_ready(self) -> bool:
        """检查模块是否就绪"""
        return self._ready
    
    async def initialize(self) -> bool:
        """
        异步初始化，加载 NanoSAM 引擎
        
        Returns:
            bool: 是否初始化成功
            
        Raises:
            PerceptionError: 初始化失败时抛出
        """
        try:
            # 检查 NanoSAM 是否可用
            if not NANOSAM_AVAILABLE:
                raise PerceptionError(
                    ErrorCode.NANOSAM_ENGINE_LOAD_FAILED,
                    f"NanoSAM 未安装: {NANOSAM_IMPORT_ERROR}"
                )
            
            # 检查 CUDA
            if not torch.cuda.is_available():
                raise PerceptionError(
                    ErrorCode.CUDA_NOT_AVAILABLE,
                    "CUDA 不可用"
                )
            
            # 解析引擎路径
            encoder_path = self._resolve_engine_path(
                self._config.get('engine_path', NANOSAM_ENCODER_FILENAME)
            )
            decoder_path = self._resolve_engine_path(
                self._config.get('decoder_path', NANOSAM_DECODER_FILENAME)
            )
            
            # 验证文件存在
            if not encoder_path.exists():
                raise PerceptionError(
                    ErrorCode.NANOSAM_ENGINE_LOAD_FAILED,
                    f"编码器引擎文件不存在: {encoder_path}"
                )
            if not decoder_path.exists():
                raise PerceptionError(
                    ErrorCode.NANOSAM_DECODER_LOAD_FAILED,
                    f"解码器引擎文件不存在: {decoder_path}"
                )
            
            logger.info(f"加载 NanoSAM 编码器: {encoder_path}")
            logger.info(f"加载 NanoSAM 解码器: {decoder_path}")
            
            # 在线程池中加载引擎（避免阻塞事件循环）
            loop = asyncio.get_event_loop()
            await loop.run_in_executor(
                None,
                self._load_predictor,
                str(encoder_path),
                str(decoder_path)
            )
            
            # Warmup
            await loop.run_in_executor(None, self._warmup)
            
            self._ready = True
            logger.info("SegmentationModule 初始化完成")
            return True
            
        except PerceptionError:
            raise
        except Exception as e:
            logger.error(f"SegmentationModule 初始化失败: {e}")
            raise PerceptionError(
                ErrorCode.NANOSAM_ENGINE_LOAD_FAILED,
                str(e)
            )
    
    def _resolve_engine_path(self, path_str: str) -> Path:
        """解析引擎文件路径"""
        path = Path(path_str)
        if path.is_absolute():
            return path
        
        # 尝试相对于包目录
        package_dir = Path(__file__).parent.parent.parent
        candidate = package_dir / path_str
        if candidate.exists():
            return candidate
        
        # 尝试 models 目录
        models_dir = package_dir / 'models'
        candidate = models_dir / Path(path_str).name
        if candidate.exists():
            return candidate
        
        return path
    
    def _load_predictor(self, encoder_path: str, decoder_path: str):
        """加载 NanoSAM 预测器（同步方法，在线程池中执行）"""
        self._predictor = _FastPredictor(
            encoder_path,
            decoder_path,
            image_encoder_size=self._config.get('image_encoder_size', 1024),
            orig_image_encoder_size=self._config.get('orig_image_encoder_size', 1024),
        )
    
    def _warmup(self):
        """预热引擎"""
        if self._predictor is None:
            return
        
        try:
            dummy = np.zeros((480, 640, 3), dtype=np.uint8)
            self._predictor.set_image(dummy)
            self._predictor.predict(
                np.array([[320, 240]], dtype=np.float32),
                np.array([1], dtype=np.float32)
            )
            self._predictor.features = None
            logger.info("NanoSAM warmup 完成")
        except Exception as e:
            logger.warning(f"NanoSAM warmup 失败: {e}")
    
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
        
        # 验证输入
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
        
        async with self._engine_lock:
            loop = asyncio.get_event_loop()
            
            # 编码图像
            start_encode = time.time()
            await loop.run_in_executor(None, self._predictor.set_image, image)
            encode_time = time.time() - start_encode
            self._encode_times.append(encode_time)
            
            # 解码掩码
            start_decode = time.time()
            point_coords = np.array([[click_x, click_y]], dtype=np.float32)
            point_labels = np.array([1], dtype=np.float32)  # 1 = 前景点
            
            mask_result = await loop.run_in_executor(
                None,
                self._predictor.predict,
                point_coords,
                point_labels,
                None  # mask_input
            )
            decode_time = time.time() - start_decode
            self._decode_times.append(decode_time)
        
        hi_res_mask, mask_iou, _ = mask_result
        confidence = float(mask_iou.view(-1)[0].detach().cpu().item())
        
        # 转换为 numpy
        mask_np = (
            hi_res_mask[0, 0]
            .detach()
            .cpu()
            .numpy()
        )
        
        # 后处理
        mask_binary = self._post_process_mask(mask_np, confidence)
        
        # 计算掩码面积
        mask_area = int(np.sum(mask_binary > 0))
        total_pixels = h * w
        
        # 验证结果
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
        
        if mask_area / total_pixels > self._max_mask_area_ratio:
            logger.warning(
                f"掩码面积占比过高: {mask_area / total_pixels:.2%}"
            )
        
        # 裁剪目标图像
        cropped_image, bbox = self._crop_by_mask(image, mask_binary)
        
        logger.debug(
            f"分割完成: confidence={confidence:.3f}, area={mask_area}, "
            f"encode={encode_time*1000:.1f}ms, decode={decode_time*1000:.1f}ms"
        )
        
        return SegmentResult(
            mask=mask_binary,
            cropped_image=cropped_image,
            confidence=confidence,
            mask_area=mask_area,
            bbox=bbox
        )
    
    def _post_process_mask(self, mask: np.ndarray, confidence: float) -> np.ndarray:
        """后处理掩码"""
        # 二值化
        mask_binary = (mask > 0).astype(np.uint8) * 255
        
        # 形态学操作：闭运算填充小孔
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask_binary = cv2.morphologyEx(mask_binary, cv2.MORPH_CLOSE, kernel)
        
        # 保留最大连通区域
        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(
            mask_binary, connectivity=8
        )
        if num_labels > 1:
            largest_label = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])
            mask_binary = (labels == largest_label).astype(np.uint8) * 255
        
        # 平滑边界
        mask_binary = cv2.GaussianBlur(mask_binary, (3, 3), 0)
        mask_binary = (mask_binary > 127).astype(np.uint8) * 255
        
        return mask_binary
    
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
        
        if self._encode_times:
            times = np.array(self._encode_times)
            stats['avg_encode_time_ms'] = float(np.mean(times) * 1000)
            stats['max_encode_time_ms'] = float(np.max(times) * 1000)
        
        if self._decode_times:
            times = np.array(self._decode_times)
            stats['avg_decode_time_ms'] = float(np.mean(times) * 1000)
            stats['max_decode_time_ms'] = float(np.max(times) * 1000)
        
        return stats


class _FastPredictor:
    """
    NanoSAM 快速预测器
    
    复用自 L1 的 FastPredictor，优化 CUDA 内存分配
    """
    
    def __init__(
        self,
        image_encoder_engine: str,
        mask_decoder_engine: str,
        image_encoder_size: int = 1024,
        orig_image_encoder_size: int = 1024,
        max_points: int = 16,
    ):
        self.device = torch.device("cuda")
        self.stream = torch.cuda.Stream(device=self.device)
        
        self.image_encoder_engine = load_image_encoder_engine(image_encoder_engine)
        self.mask_decoder_engine = load_mask_decoder_engine(mask_decoder_engine)
        self.image_encoder_size = int(image_encoder_size)
        self.orig_image_encoder_size = int(orig_image_encoder_size)
        
        # 预分配张量
        self._mean = torch.tensor(
            [123.675, 116.28, 103.53], device=self.device, dtype=torch.float32
        ).view(3, 1, 1)
        self._std = torch.tensor(
            [58.395, 57.12, 57.375], device=self.device, dtype=torch.float32
        ).view(3, 1, 1)
        
        self._image_tensor = torch.zeros(
            (1, 3, self.image_encoder_size, self.image_encoder_size),
            device=self.device,
            dtype=torch.float32,
        )
        
        self._mask_input = torch.zeros(
            (1, 1, 256, 256), device=self.device, dtype=torch.float32
        )
        self._has_mask_input = torch.zeros((1,), device=self.device, dtype=torch.float32)
        
        self._point_coords = torch.zeros(
            (1, max_points, 2), device=self.device, dtype=torch.float32
        )
        self._point_labels = torch.zeros(
            (1, max_points), device=self.device, dtype=torch.float32
        )
        
        self.features: Optional[torch.Tensor] = None
        self.image_height: Optional[int] = None
        self.image_width: Optional[int] = None
    
    def _ensure_point_capacity(self, required: int) -> None:
        if required <= self._point_coords.shape[1]:
            return
        new_size = max(required, self._point_coords.shape[1] * 2)
        self._point_coords = torch.zeros(
            (1, new_size, 2), device=self.device, dtype=torch.float32
        )
        self._point_labels = torch.zeros(
            (1, new_size), device=self.device, dtype=torch.float32
        )
    
    def set_image(self, image: np.ndarray) -> None:
        """设置图像并编码"""
        image_np = np.asarray(image, dtype=np.uint8)
        self.image_height, self.image_width = int(image_np.shape[0]), int(image_np.shape[1])
        
        # 计算缩放尺寸
        aspect_ratio = self.image_width / max(self.image_height, 1)
        if aspect_ratio >= 1.0:
            resize_width = self.image_encoder_size
            resize_height = max(1, int(round(self.image_encoder_size / aspect_ratio)))
        else:
            resize_height = self.image_encoder_size
            resize_width = max(1, int(round(self.image_encoder_size * aspect_ratio)))
        
        # 缩放图像
        resized = cv2.resize(
            image_np,
            (resize_width, resize_height),
            interpolation=cv2.INTER_LINEAR,
        )
        resized = np.ascontiguousarray(resized)
        
        # 转换为张量并归一化
        tensor = torch.from_numpy(resized).to(
            device=self.device, dtype=torch.float32, non_blocking=True
        )
        tensor = tensor.permute(2, 0, 1)
        tensor = (tensor - self._mean) / self._std
        
        # 编码
        with torch.cuda.stream(self.stream):
            self._image_tensor.zero_()
            self._image_tensor[:, :, :resize_height, :resize_width].copy_(tensor)
            self.features = self.image_encoder_engine(self._image_tensor)
        
        torch.cuda.current_stream().wait_stream(self.stream)
    
    def predict(
        self,
        points: np.ndarray,
        point_labels: Optional[np.ndarray],
        mask_input: Optional[torch.Tensor] = None,
    ):
        """预测掩码"""
        if self.features is None:
            raise RuntimeError("必须先调用 set_image")
        
        coords = np.asarray(points, dtype=np.float32)
        if coords.ndim == 1:
            coords = coords.reshape(1, 2)
        
        if point_labels is None:
            labels = np.ones((coords.shape[0],), dtype=np.float32)
        else:
            labels = np.asarray(point_labels, dtype=np.float32)
            if labels.ndim == 0:
                labels = labels.reshape(1)
        
        num_points = coords.shape[0]
        self._ensure_point_capacity(num_points)
        
        # 缩放坐标
        scale = self.orig_image_encoder_size / max(self.image_height, self.image_width)
        coords_scaled = coords * scale
        
        coords_tensor = torch.from_numpy(coords_scaled).to(
            device=self.device, dtype=torch.float32, non_blocking=True
        )
        labels_tensor = torch.from_numpy(labels).to(
            device=self.device, dtype=torch.float32, non_blocking=True
        )
        
        with torch.cuda.stream(self.stream):
            self._point_coords[0, :num_points, :].copy_(coords_tensor)
            self._point_labels[0, :num_points].copy_(labels_tensor)
            
            coords_view = self._point_coords[:, :num_points, :]
            labels_view = self._point_labels[:, :num_points]
            
            if mask_input is None:
                mask_tensor = self._mask_input
                self._has_mask_input.zero_()
            else:
                mask_tensor = torch.as_tensor(
                    mask_input, device=self.device, dtype=torch.float32
                )
                self._has_mask_input.fill_(1.0)
            
            mask_iou, low_res_mask = self.mask_decoder_engine(
                self.features,
                coords_view,
                labels_view,
                mask_tensor,
                self._has_mask_input,
            )
        
        torch.cuda.current_stream().wait_stream(self.stream)
        
        # 上采样掩码
        hi_res_mask = upscale_mask(
            low_res_mask, (self.image_height, self.image_width)
        )
        
        return hi_res_mask, mask_iou, low_res_mask

