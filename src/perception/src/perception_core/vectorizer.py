"""
VectorizerModule - 向量化模块

CLIP + DINOv3 特征提取
- CLIP: 用于文本→图像检索
- DINOv3: 用于图像→图像检索
"""

import asyncio
import logging
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Any, Optional

import cv2
import numpy as np

from .constants import (
    CLIP_EMBEDDING_DIM,
    DINO_EMBEDDING_DIM,
    CLIP_MODEL_NAME,
    DINO_MODEL_FILENAME,
)
from .error_codes import PerceptionError, ErrorCode

# 延迟导入（可能不可用）
TORCH_AVAILABLE = False
TRANSFORMERS_AVAILABLE = False

try:
    import torch
    import torch.nn as nn
    TORCH_AVAILABLE = True
except ImportError:
    pass

try:
    from transformers import CLIPModel, CLIPProcessor
    TRANSFORMERS_AVAILABLE = True
except ImportError:
    pass

logger = logging.getLogger(__name__)


@dataclass
class VectorizeResult:
    """向量化结果"""
    clip_embedding: np.ndarray      # CLIP 向量 (512,)
    dino_embedding: np.ndarray      # DINOv3 向量 (384,)


class VectorizerModule:
    """
    向量化模块
    
    使用 CLIP 和 DINOv3 提取图像特征向量
    """
    
    def __init__(self, config: Dict[str, Any]):
        """
        初始化配置
        
        Args:
            config: 配置字典，包含:
                clip:
                    model: CLIP 模型名称
                    device: 设备
                dino:
                    model_path: DINOv3 模型路径
                    device: 设备
                    feature_dim: 特征维度
        """
        self._config = config
        self._ready = False
        
        # CLIP 配置
        self._clip_config = config.get('clip', {})
        self._clip_model_name = self._clip_config.get('model', CLIP_MODEL_NAME)
        self._clip_device = self._clip_config.get('device', 'cuda')
        
        # DINOv3 配置
        self._dino_config = config.get('dino', {})
        self._dino_model_path = self._dino_config.get('model_path', DINO_MODEL_FILENAME)
        self._dino_device = self._dino_config.get('device', 'cuda')
        self._dino_feature_dim = self._dino_config.get('feature_dim', DINO_EMBEDDING_DIM)
        
        # 模型实例
        self._clip_model = None
        self._clip_processor = None
        self._dino_model = None
        
        # 预处理参数
        self._dino_image_size = 224
        self._dino_mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
        self._dino_std = np.array([0.229, 0.224, 0.225], dtype=np.float32)
        
        logger.info("VectorizerModule 配置已加载")
    
    @property
    def is_ready(self) -> bool:
        """检查模块是否就绪"""
        return self._ready
    
    async def initialize(self) -> bool:
        """
        异步初始化，加载 CLIP 和 DINOv3 模型
        
        Returns:
            bool: 是否初始化成功
        """
        try:
            # 检查依赖
            if not TORCH_AVAILABLE:
                raise PerceptionError(
                    ErrorCode.CLIP_MODEL_LOAD_FAILED,
                    "PyTorch 未安装"
                )
            
            if not TRANSFORMERS_AVAILABLE:
                raise PerceptionError(
                    ErrorCode.CLIP_MODEL_LOAD_FAILED,
                    "transformers 库未安装"
                )
            
            # 检查 CUDA
            if self._clip_device == 'cuda' and not torch.cuda.is_available():
                logger.warning("CUDA 不可用，回退到 CPU")
                self._clip_device = 'cpu'
                self._dino_device = 'cpu'
            
            # 在线程池中加载模型
            loop = asyncio.get_event_loop()
            
            # 并行加载 CLIP 和 DINOv3
            await asyncio.gather(
                loop.run_in_executor(None, self._load_clip),
                loop.run_in_executor(None, self._load_dino)
            )
            
            self._ready = True
            logger.info("VectorizerModule 初始化完成")
            return True
            
        except PerceptionError:
            raise
        except Exception as e:
            logger.error(f"VectorizerModule 初始化失败: {e}")
            raise PerceptionError(ErrorCode.INTERNAL_ERROR, str(e))
    
    def _load_clip(self):
        """加载 CLIP 模型（可选，失败时禁用文本检索功能）"""
        logger.info(f"加载 CLIP 模型: {self._clip_model_name}")
        
        try:
            self._clip_model = CLIPModel.from_pretrained(self._clip_model_name)
            self._clip_processor = CLIPProcessor.from_pretrained(self._clip_model_name)
            
            self._clip_model.to(self._clip_device)
            self._clip_model.eval()
            
            logger.info(f"CLIP 模型加载完成 (device={self._clip_device})")
            
        except Exception as e:
            # CLIP 加载失败不阻止服务启动，只是禁用文本检索功能
            logger.warning(f"CLIP 模型加载失败，文本检索功能将不可用: {e}")
            self._clip_model = None
            self._clip_processor = None
    
    def _load_dino(self):
        """加载 DINOv3 模型"""
        logger.info(f"加载 DINOv3 模型")
        
        try:
            # 解析模型路径
            model_path = self._resolve_model_path(self._dino_model_path)
            
            if model_path.exists():
                logger.info(f"从本地加载 DINOv3: {model_path}")
                self._dino_model = self._load_dino_from_checkpoint(model_path)
            else:
                logger.info("从 torch.hub 加载 DINOv3")
                self._dino_model = torch.hub.load(
                    'facebookresearch/dinov2',
                    'dinov2_vits14',
                    pretrained=True
                )
            
            self._dino_model.to(self._dino_device)
            self._dino_model.eval()
            
            logger.info(f"DINOv3 模型加载完成 (device={self._dino_device})")
            
        except Exception as e:
            raise PerceptionError(
                ErrorCode.DINO_MODEL_LOAD_FAILED,
                f"DINOv3 加载失败: {e}"
            )
    
    def _resolve_model_path(self, path_str: str) -> Path:
        """解析模型文件路径"""
        path = Path(path_str)
        if path.is_absolute():
            return path
        
        # 尝试相对于包目录
        package_dir = Path(__file__).parent.parent.parent
        
        # 直接路径
        candidate = package_dir / path_str
        if candidate.exists():
            return candidate
        
        # models 目录
        candidate = package_dir / 'models' / Path(path_str).name
        if candidate.exists():
            return candidate
        
        return path
    
    def _load_dino_from_checkpoint(self, model_path: Path):
        """从本地 checkpoint 加载 DINOv3"""
        checkpoint = torch.load(str(model_path), map_location=self._dino_device)
        
        # 尝试从 torch.hub 获取架构
        model = torch.hub.load(
            'facebookresearch/dinov2',
            'dinov2_vits14',
            pretrained=False
        )
        
        # 加载权重
        if 'model' in checkpoint:
            model.load_state_dict(checkpoint['model'], strict=False)
        elif 'teacher' in checkpoint:
            model.load_state_dict(checkpoint['teacher'], strict=False)
        else:
            model.load_state_dict(checkpoint, strict=False)
        
        return model
    
    async def extract(self, image: np.ndarray) -> VectorizeResult:
        """
        提取图像特征向量
        
        Args:
            image: RGB 图像 (H, W, 3)
            
        Returns:
            VectorizeResult: CLIP 和 DINOv3 向量
            
        Raises:
            PerceptionError: 提取失败时抛出
        """
        if not self._ready:
            raise PerceptionError(ErrorCode.NODE_NOT_READY, "向量化模块未就绪")
        
        if image is None or image.size == 0:
            raise PerceptionError(ErrorCode.IMAGE_EMPTY)
        
        loop = asyncio.get_event_loop()
        
        # 并行提取 CLIP 和 DINOv3 特征
        clip_task = loop.run_in_executor(None, self._extract_clip, image)
        dino_task = loop.run_in_executor(None, self._extract_dino, image)
        
        try:
            clip_embedding, dino_embedding = await asyncio.gather(clip_task, dino_task)
        except Exception as e:
            raise PerceptionError(ErrorCode.IMAGE_PREPROCESS_FAILED, str(e))
        
        return VectorizeResult(
            clip_embedding=clip_embedding,
            dino_embedding=dino_embedding
        )
    
    def _extract_clip(self, image: np.ndarray) -> np.ndarray:
        """提取 CLIP 图像特征"""
        # CLIP 不可用时返回零向量
        if self._clip_model is None or self._clip_processor is None:
            return np.zeros(CLIP_EMBEDDING_DIM, dtype=np.float32)
        
        try:
            # 预处理
            inputs = self._clip_processor(
                images=image,
                return_tensors="pt"
            )
            inputs = {k: v.to(self._clip_device) for k, v in inputs.items()}
            
            # 提取特征
            with torch.no_grad():
                image_features = self._clip_model.get_image_features(**inputs)
                # L2 归一化
                image_features = image_features / image_features.norm(dim=-1, keepdim=True)
            
            embedding = image_features.cpu().numpy().squeeze()
            
            # 验证维度
            if embedding.shape[0] != CLIP_EMBEDDING_DIM:
                logger.warning(
                    f"CLIP 维度不匹配: {embedding.shape[0]} != {CLIP_EMBEDDING_DIM}"
                )
            
            return embedding.astype(np.float32)
            
        except Exception as e:
            raise PerceptionError(ErrorCode.CLIP_ENCODE_FAILED, str(e))
    
    def _extract_dino(self, image: np.ndarray) -> np.ndarray:
        """提取 DINOv3 图像特征"""
        try:
            # 预处理
            image_tensor = self._preprocess_dino(image)
            
            # 提取特征
            with torch.no_grad():
                features = self._dino_model(image_tensor)
                
                # 处理不同输出格式
                if hasattr(features, 'last_hidden_state'):
                    # Transformer 输出
                    features = features.last_hidden_state[:, 0, :]
                elif isinstance(features, tuple):
                    features = features[0]
                
                if features.ndim == 3:
                    # 取 CLS token 或平均
                    features = features[:, 0, :]
                
                # L2 归一化
                features = nn.functional.normalize(features, p=2, dim=1)
            
            embedding = features.cpu().numpy().squeeze()
            
            return embedding.astype(np.float32)
            
        except Exception as e:
            raise PerceptionError(ErrorCode.DINO_ENCODE_FAILED, str(e))
    
    def _preprocess_dino(self, image: np.ndarray) -> torch.Tensor:
        """DINOv3 图像预处理"""
        # 缩放到标准尺寸
        if image.shape[0] != self._dino_image_size or image.shape[1] != self._dino_image_size:
            image = cv2.resize(
                image,
                (self._dino_image_size, self._dino_image_size),
                interpolation=cv2.INTER_CUBIC
            )
        
        # 归一化
        image = image.astype(np.float32) / 255.0
        image = (image - self._dino_mean) / self._dino_std
        
        # 转换为张量
        tensor = torch.from_numpy(image).permute(2, 0, 1).unsqueeze(0)
        tensor = tensor.to(self._dino_device)
        
        return tensor
    
    async def encode_text(self, text: str) -> np.ndarray:
        """
        CLIP 文本编码
        
        Args:
            text: 文本描述
            
        Returns:
            np.ndarray: CLIP 文本向量 (512,)
            
        Raises:
            PerceptionError: 编码失败时抛出
        """
        if not self._ready:
            raise PerceptionError(ErrorCode.NODE_NOT_READY, "向量化模块未就绪")
        
        if not text or not text.strip():
            raise PerceptionError(ErrorCode.TEXT_ENCODE_FAILED, "文本为空")
        
        loop = asyncio.get_event_loop()
        embedding = await loop.run_in_executor(None, self._encode_text_sync, text)
        
        return embedding
    
    def _encode_text_sync(self, text: str) -> np.ndarray:
        """同步文本编码"""
        # CLIP 不可用时无法进行文本编码
        if self._clip_model is None or self._clip_processor is None:
            raise PerceptionError(
                ErrorCode.CLIP_MODEL_LOAD_FAILED,
                "CLIP 模型不可用，文本检索功能已禁用"
            )
        
        try:
            # 预处理
            inputs = self._clip_processor(
                text=[text],
                return_tensors="pt",
                padding=True,
                truncation=True
            )
            inputs = {k: v.to(self._clip_device) for k, v in inputs.items()}
            
            # 提取特征
            with torch.no_grad():
                text_features = self._clip_model.get_text_features(**inputs)
                # L2 归一化
                text_features = text_features / text_features.norm(dim=-1, keepdim=True)
            
            embedding = text_features.cpu().numpy().squeeze()
            
            return embedding.astype(np.float32)
            
        except Exception as e:
            raise PerceptionError(ErrorCode.TEXT_ENCODE_FAILED, str(e))
    
    def compute_similarity(
        self,
        embedding1: np.ndarray,
        embedding2: np.ndarray
    ) -> float:
        """
        计算两个向量的余弦相似度
        
        Args:
            embedding1: 向量1
            embedding2: 向量2
            
        Returns:
            float: 相似度 (0.0-1.0)
        """
        dot = np.dot(embedding1, embedding2)
        norm1 = np.linalg.norm(embedding1)
        norm2 = np.linalg.norm(embedding2)
        
        if norm1 == 0 or norm2 == 0:
            return 0.0
        
        similarity = dot / (norm1 * norm2)
        # 确保在 [0, 1] 范围内
        return float(max(0.0, min(1.0, (similarity + 1) / 2)))

