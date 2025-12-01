"""
ObjectStorageManager - 物体存储管理模块

物体数据的增删改查（依赖 VectorManager）
"""

import logging
import time
import uuid
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Dict, Any, List, Optional, Tuple

import cv2
import numpy as np
import yaml

from .constants import (
    INDEX_FILENAME,
    IMAGES_DIR,
    MAX_OBJECTS,
    MAX_SAMPLES_PER_OBJECT,
    MAX_LABEL_LENGTH,
    MAX_DESCRIPTION_LENGTH,
    DEFAULT_TOP_K,
    DEFAULT_MIN_SIMILARITY,
)
from .error_codes import PerceptionError, ErrorCode
from .vector_manager import VectorManager
from .async_worker import AsyncWorker

logger = logging.getLogger(__name__)


@dataclass
class ObjectInfo:
    """物体信息"""
    object_id: str
    label: str
    description: str
    created_at: int          # 毫秒时间戳
    updated_at: int          # 毫秒时间戳
    sample_count: int
    image_path: str = ""


@dataclass
class SampleInfo:
    """样本信息"""
    sample_id: str
    object_id: str
    created_at: int          # 毫秒时间戳
    clip_index: int = -1
    dino_index: int = -1


class ObjectStorageManager:
    """
    物体存储管理器
    
    管理物体数据的增删改查，依赖 VectorManager 进行向量操作
    """
    
    def __init__(
        self,
        config: Dict[str, Any],
        storage_dir: str,
        vector_manager: VectorManager,
        worker: AsyncWorker
    ):
        """
        初始化
        
        Args:
            config: 配置字典
            storage_dir: 存储目录路径
            vector_manager: 向量管理器实例
        """
        if worker is None:
            raise ValueError("ObjectStorageManager 需要有效的 AsyncWorker 实例")
        self._config = config
        self._storage_dir = Path(storage_dir)
        self._vector_manager = vector_manager
        self._worker = worker
        self._ready = False
        
        # 存储路径
        self._index_path = self._storage_dir / INDEX_FILENAME
        self._images_dir = self._storage_dir / IMAGES_DIR
        
        # 内存中的物体数据
        self._objects: Dict[str, Dict[str, Any]] = {}
        self._metadata: Dict[str, Any] = {}
        
        logger.info(f"ObjectStorageManager 配置已加载: storage_dir={storage_dir}")
    
    @property
    def is_ready(self) -> bool:
        """检查模块是否就绪"""
        return self._ready
    
    async def initialize(self) -> bool:
        """
        异步初始化，加载 index.yaml
        
        Returns:
            bool: 是否初始化成功
        """
        try:
            # 创建目录
            self._storage_dir.mkdir(parents=True, exist_ok=True)
            self._images_dir.mkdir(parents=True, exist_ok=True)
            
            # 加载现有数据
            await self._worker.run_callable(self._load_index)
            
            self._ready = True
            logger.info(
                f"ObjectStorageManager 初始化完成: "
                f"objects={len(self._objects)}"
            )
            return True
            
        except Exception as e:
            logger.error(f"ObjectStorageManager 初始化失败: {e}")
            raise PerceptionError(ErrorCode.INTERNAL_ERROR, str(e))
    
    def _load_index(self):
        """加载 index.yaml"""
        if self._index_path.exists():
            try:
                with open(self._index_path, 'r', encoding='utf-8') as f:
                    data = yaml.safe_load(f) or {}
                
                self._objects = data.get('objects', {})
                self._metadata = data.get('metadata', {})
                logger.info(f"从磁盘加载 {len(self._objects)} 个物体")
            except Exception as e:
                logger.warning(f"加载 index.yaml 失败: {e}")
                self._objects = {}
                self._metadata = {}
        else:
            self._objects = {}
            self._metadata = {
                'total_objects': 0,
                'total_samples': 0,
                'last_updated': self._current_timestamp(),
            }
    
    def _save_index(self):
        """保存 index.yaml"""
        # 更新元数据
        total_samples = sum(
            obj.get('sample_count', 0) for obj in self._objects.values()
        )
        self._metadata.update({
            'total_objects': len(self._objects),
            'total_samples': total_samples,
            'last_updated': self._current_timestamp(),
        })
        
        data = {
            'objects': self._objects,
            'metadata': self._metadata,
        }
        
        # 原子写入
        tmp_path = self._index_path.with_suffix('.tmp')
        with open(tmp_path, 'w', encoding='utf-8') as f:
            yaml.dump(data, f, allow_unicode=True, default_flow_style=False)
        tmp_path.rename(self._index_path)
    
    def _current_timestamp(self) -> int:
        """获取当前毫秒时间戳"""
        return int(time.time() * 1000)
    
    def _generate_object_id(self) -> str:
        """生成物体 ID"""
        # 使用递增编号
        existing_ids = set(self._objects.keys())
        for i in range(1, MAX_OBJECTS + 1):
            obj_id = f"obj_{i:03d}"
            if obj_id not in existing_ids:
                return obj_id
        # 回退到 UUID
        return f"obj_{uuid.uuid4().hex[:8]}"
    
    def _generate_sample_id(self, object_id: str) -> str:
        """生成样本 ID"""
        obj = self._objects.get(object_id, {})
        samples = obj.get('samples', {})
        for i in range(1, MAX_SAMPLES_PER_OBJECT + 1):
            sample_id = f"s{i:03d}"
            if sample_id not in samples:
                return sample_id
        return f"s_{uuid.uuid4().hex[:6]}"
    
    # =========================================================================
    # 物体操作
    # =========================================================================
    
    def save_object(
        self,
        image: np.ndarray,
        clip_emb: np.ndarray,
        dino_emb: np.ndarray,
        label: str = "",
        description: str = ""
    ) -> Tuple[str, int]:
        """
        保存物体
        
        Args:
            image: 物体图像
            clip_emb: CLIP 向量
            dino_emb: DINOv3 向量
            label: 标签
            description: 描述
            
        Returns:
            Tuple[str, int]: (object_id, created_at)
        """
        if not self._ready:
            raise PerceptionError(ErrorCode.NODE_NOT_READY)
        
        # 验证
        if len(self._objects) >= MAX_OBJECTS:
            raise PerceptionError(
                ErrorCode.INTERNAL_ERROR,
                f"物体数量已达上限 {MAX_OBJECTS}"
            )
        
        label = label[:MAX_LABEL_LENGTH] if label else ""
        description = description[:MAX_DESCRIPTION_LENGTH] if description else ""
        
        # 生成 ID
        object_id = self._generate_object_id()
        sample_id = "s001"
        timestamp = self._current_timestamp()
        
        # 保存图像
        image_filename = f"{object_id}_cropped.png"
        image_path = self._images_dir / image_filename
        cv2.imwrite(str(image_path), cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
        
        # 添加向量到索引
        clip_idx, dino_idx = self._vector_manager.add_vectors(
            clip_emb, dino_emb, object_id, sample_id
        )
        
        # 创建物体记录
        self._objects[object_id] = {
            'label': label,
            'description': description,
            'created_at': timestamp,
            'updated_at': timestamp,
            'image_path': f"{IMAGES_DIR}/{image_filename}",
            'samples': {
                sample_id: {
                    'created_at': timestamp,
                    'clip_index': clip_idx,
                    'dino_index': dino_idx,
                }
            },
            'sample_count': 1,
        }
        
        # 持久化
        self._save_index()
        self._vector_manager.save()
        
        logger.info(f"保存物体: {object_id}, label={label}")
        return object_id, timestamp
    
    def add_sample(
        self,
        object_id: str,
        clip_emb: np.ndarray,
        dino_emb: np.ndarray
    ) -> Tuple[str, int]:
        """
        给物体增加样本
        
        Args:
            object_id: 物体 ID
            clip_emb: CLIP 向量
            dino_emb: DINOv3 向量
            
        Returns:
            Tuple[str, int]: (sample_id, total_samples)
        """
        if not self._ready:
            raise PerceptionError(ErrorCode.NODE_NOT_READY)
        
        if object_id not in self._objects:
            raise PerceptionError(ErrorCode.OBJECT_NOT_FOUND, object_id)
        
        obj = self._objects[object_id]
        if obj.get('sample_count', 0) >= MAX_SAMPLES_PER_OBJECT:
            raise PerceptionError(
                ErrorCode.INTERNAL_ERROR,
                f"样本数量已达上限 {MAX_SAMPLES_PER_OBJECT}"
            )
        
        # 生成样本 ID
        sample_id = self._generate_sample_id(object_id)
        timestamp = self._current_timestamp()
        
        # 添加向量
        clip_idx, dino_idx = self._vector_manager.add_vectors(
            clip_emb, dino_emb, object_id, sample_id
        )
        
        # 更新物体记录
        obj['samples'][sample_id] = {
            'created_at': timestamp,
            'clip_index': clip_idx,
            'dino_index': dino_idx,
        }
        obj['sample_count'] = len(obj['samples'])
        obj['updated_at'] = timestamp
        
        # 持久化
        self._save_index()
        self._vector_manager.save()
        
        logger.info(f"增加样本: {object_id}/{sample_id}")
        return sample_id, obj['sample_count']
    
    def update_object(
        self,
        object_id: str,
        label: Optional[str] = None,
        description: Optional[str] = None
    ) -> int:
        """
        更新物体标签/描述
        
        Args:
            object_id: 物体 ID
            label: 新标签（None 或空字符串表示不更新）
            description: 新描述（None 或空字符串表示不更新）
            
        Returns:
            int: 更新时间戳
        """
        if not self._ready:
            raise PerceptionError(ErrorCode.NODE_NOT_READY)
        
        if object_id not in self._objects:
            raise PerceptionError(ErrorCode.OBJECT_NOT_FOUND, object_id)
        
        obj = self._objects[object_id]
        timestamp = self._current_timestamp()
        
        if label:
            obj['label'] = label[:MAX_LABEL_LENGTH]
        if description:
            obj['description'] = description[:MAX_DESCRIPTION_LENGTH]
        
        obj['updated_at'] = timestamp
        
        # 持久化
        self._save_index()
        
        logger.info(f"更新物体: {object_id}")
        return timestamp
    
    def delete_object(self, object_id: str) -> int:
        """
        删除物体及其所有样本
        
        Args:
            object_id: 物体 ID
            
        Returns:
            int: 删除的样本数量
        """
        if not self._ready:
            raise PerceptionError(ErrorCode.NODE_NOT_READY)
        
        if object_id not in self._objects:
            raise PerceptionError(ErrorCode.OBJECT_NOT_FOUND, object_id)
        
        obj = self._objects[object_id]
        sample_count = obj.get('sample_count', 0)
        
        # 删除向量
        clip_indices, dino_indices = self._vector_manager.get_indices_by_object(object_id)
        if clip_indices or dino_indices:
            self._vector_manager.delete_vectors(clip_indices, dino_indices)
        
        # 删除图像文件
        image_path = self._storage_dir / obj.get('image_path', '')
        if image_path.exists():
            image_path.unlink()
        
        # 删除物体记录
        del self._objects[object_id]
        
        # 持久化
        self._save_index()
        self._vector_manager.save()
        
        logger.info(f"删除物体: {object_id}, samples={sample_count}")
        return sample_count
    
    def delete_sample(self, object_id: str, sample_id: str) -> int:
        """
        删除样本
        
        Args:
            object_id: 物体 ID
            sample_id: 样本 ID
            
        Returns:
            int: 剩余样本数量
        """
        if not self._ready:
            raise PerceptionError(ErrorCode.NODE_NOT_READY)
        
        if object_id not in self._objects:
            raise PerceptionError(ErrorCode.OBJECT_NOT_FOUND, object_id)
        
        obj = self._objects[object_id]
        if sample_id not in obj.get('samples', {}):
            raise PerceptionError(ErrorCode.SAMPLE_NOT_FOUND, sample_id)
        
        # 不能删除最后一个样本
        if obj.get('sample_count', 0) <= 1:
            raise PerceptionError(ErrorCode.CANNOT_DELETE_LAST_SAMPLE)
        
        # 获取向量索引
        sample = obj['samples'][sample_id]
        clip_idx = sample.get('clip_index', -1)
        dino_idx = sample.get('dino_index', -1)
        
        # 删除向量
        if clip_idx >= 0 or dino_idx >= 0:
            self._vector_manager.delete_vectors(
                [clip_idx] if clip_idx >= 0 else [],
                [dino_idx] if dino_idx >= 0 else []
            )
        
        # 删除样本记录
        del obj['samples'][sample_id]
        obj['sample_count'] = len(obj['samples'])
        obj['updated_at'] = self._current_timestamp()
        
        # 持久化
        self._save_index()
        self._vector_manager.save()
        
        logger.info(f"删除样本: {object_id}/{sample_id}")
        return obj['sample_count']
    
    # =========================================================================
    # 查询操作
    # =========================================================================
    
    def query_by_clip(
        self,
        emb: np.ndarray,
        top_k: int = DEFAULT_TOP_K,
        min_sim: float = DEFAULT_MIN_SIMILARITY
    ) -> List[Tuple[ObjectInfo, float]]:
        """
        CLIP 向量查询（文本描述查询用）
        
        Args:
            emb: CLIP 查询向量
            top_k: 返回前 K 个
            min_sim: 最小相似度
            
        Returns:
            List[Tuple[ObjectInfo, float]]: [(物体信息, 相似度), ...]
        """
        if not self._ready:
            raise PerceptionError(ErrorCode.NODE_NOT_READY)
        
        # 搜索向量
        results = self._vector_manager.search_by_clip(emb, top_k * 3)  # 多搜一些，去重后筛选
        
        # 按物体聚合，取最高分
        object_scores: Dict[str, float] = {}
        for idx, distance, object_id, sample_id in results:
            if not object_id:
                continue
            # L2 距离转相似度（简化处理）
            similarity = 1.0 / (1.0 + distance)
            if similarity >= min_sim:
                if object_id not in object_scores or similarity > object_scores[object_id]:
                    object_scores[object_id] = similarity
        
        # 排序并取 top_k
        sorted_objects = sorted(object_scores.items(), key=lambda x: -x[1])[:top_k]
        
        # 构建结果
        results_list = []
        for object_id, similarity in sorted_objects:
            obj = self._objects.get(object_id)
            if obj:
                info = ObjectInfo(
                    object_id=object_id,
                    label=obj.get('label', ''),
                    description=obj.get('description', ''),
                    created_at=obj.get('created_at', 0),
                    updated_at=obj.get('updated_at', 0),
                    sample_count=obj.get('sample_count', 0),
                    image_path=obj.get('image_path', ''),
                )
                results_list.append((info, similarity))
        
        return results_list
    
    def query_by_dino(
        self,
        emb: np.ndarray,
        top_k: int = DEFAULT_TOP_K,
        min_sim: float = DEFAULT_MIN_SIMILARITY
    ) -> List[Tuple[ObjectInfo, float]]:
        """
        DINOv3 向量查询（图像相似度查询用）
        
        Args:
            emb: DINOv3 查询向量
            top_k: 返回前 K 个
            min_sim: 最小相似度
            
        Returns:
            List[Tuple[ObjectInfo, float]]: [(物体信息, 相似度), ...]
        """
        if not self._ready:
            raise PerceptionError(ErrorCode.NODE_NOT_READY)
        
        # 搜索向量
        results = self._vector_manager.search_by_dino(emb, top_k * 3)
        
        # 按物体聚合
        object_scores: Dict[str, float] = {}
        for idx, distance, object_id, sample_id in results:
            if not object_id:
                continue
            similarity = 1.0 / (1.0 + distance)
            if similarity >= min_sim:
                if object_id not in object_scores or similarity > object_scores[object_id]:
                    object_scores[object_id] = similarity
        
        # 排序并取 top_k
        sorted_objects = sorted(object_scores.items(), key=lambda x: -x[1])[:top_k]
        
        # 构建结果
        results_list = []
        for object_id, similarity in sorted_objects:
            obj = self._objects.get(object_id)
            if obj:
                info = ObjectInfo(
                    object_id=object_id,
                    label=obj.get('label', ''),
                    description=obj.get('description', ''),
                    created_at=obj.get('created_at', 0),
                    updated_at=obj.get('updated_at', 0),
                    sample_count=obj.get('sample_count', 0),
                    image_path=obj.get('image_path', ''),
                )
                results_list.append((info, similarity))
        
        return results_list
    
    def query_by_label(self, label: str) -> List[ObjectInfo]:
        """
        标签精确查询
        
        Args:
            label: 标签
            
        Returns:
            List[ObjectInfo]: 匹配的物体列表
        """
        if not self._ready:
            raise PerceptionError(ErrorCode.NODE_NOT_READY)
        
        results = []
        for object_id, obj in self._objects.items():
            if obj.get('label', '') == label:
                results.append(ObjectInfo(
                    object_id=object_id,
                    label=obj.get('label', ''),
                    description=obj.get('description', ''),
                    created_at=obj.get('created_at', 0),
                    updated_at=obj.get('updated_at', 0),
                    sample_count=obj.get('sample_count', 0),
                    image_path=obj.get('image_path', ''),
                ))
        
        return results
    
    def query_by_id(self, object_id: str) -> Optional[ObjectInfo]:
        """
        ID 查询
        
        Args:
            object_id: 物体 ID
            
        Returns:
            Optional[ObjectInfo]: 物体信息，不存在则返回 None
        """
        if not self._ready:
            raise PerceptionError(ErrorCode.NODE_NOT_READY)
        
        obj = self._objects.get(object_id)
        if not obj:
            return None
        
        return ObjectInfo(
            object_id=object_id,
            label=obj.get('label', ''),
            description=obj.get('description', ''),
            created_at=obj.get('created_at', 0),
            updated_at=obj.get('updated_at', 0),
            sample_count=obj.get('sample_count', 0),
            image_path=obj.get('image_path', ''),
        )
    
    def get_object_image(self, object_id: str) -> Optional[np.ndarray]:
        """获取物体图像"""
        obj = self._objects.get(object_id)
        if not obj:
            return None
        
        image_path = self._storage_dir / obj.get('image_path', '')
        if not image_path.exists():
            return None
        
        image = cv2.imread(str(image_path))
        if image is not None:
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        return image
    
    # =========================================================================
    # 列表操作
    # =========================================================================
    
    def list_objects(
        self,
        label_filter: str = "",
        offset: int = 0,
        limit: int = 100
    ) -> Tuple[List[ObjectInfo], int]:
        """
        列出物体
        
        Args:
            label_filter: 按标签过滤（空=不过滤）
            offset: 分页偏移
            limit: 返回数量限制
            
        Returns:
            Tuple[List[ObjectInfo], int]: (物体列表, 总数量)
        """
        if not self._ready:
            raise PerceptionError(ErrorCode.NODE_NOT_READY)
        
        # 过滤
        if label_filter:
            filtered = {
                k: v for k, v in self._objects.items()
                if v.get('label', '') == label_filter
            }
        else:
            filtered = self._objects
        
        total = len(filtered)
        
        # 分页
        items = list(filtered.items())[offset:offset + limit]
        
        results = []
        for object_id, obj in items:
            results.append(ObjectInfo(
                object_id=object_id,
                label=obj.get('label', ''),
                description=obj.get('description', ''),
                created_at=obj.get('created_at', 0),
                updated_at=obj.get('updated_at', 0),
                sample_count=obj.get('sample_count', 0),
                image_path=obj.get('image_path', ''),
            ))
        
        return results, total
    
    def list_samples(self, object_id: str) -> List[SampleInfo]:
        """
        列出物体的样本
        
        Args:
            object_id: 物体 ID
            
        Returns:
            List[SampleInfo]: 样本列表
        """
        if not self._ready:
            raise PerceptionError(ErrorCode.NODE_NOT_READY)
        
        if object_id not in self._objects:
            raise PerceptionError(ErrorCode.OBJECT_NOT_FOUND, object_id)
        
        obj = self._objects[object_id]
        samples = obj.get('samples', {})
        
        results = []
        for sample_id, sample in samples.items():
            results.append(SampleInfo(
                sample_id=sample_id,
                object_id=object_id,
                created_at=sample.get('created_at', 0),
                clip_index=sample.get('clip_index', -1),
                dino_index=sample.get('dino_index', -1),
            ))
        
        return results
    
    def clear_all(self) -> Tuple[int, int]:
        """
        清空所有数据
        
        Returns:
            Tuple[int, int]: (删除的物体数, 删除的样本数)
        """
        if not self._ready:
            raise PerceptionError(ErrorCode.NODE_NOT_READY)
        
        # 统计
        object_count = len(self._objects)
        sample_count = sum(obj.get('sample_count', 0) for obj in self._objects.values())
        
        # 删除图像文件
        for obj in self._objects.values():
            image_path = self._storage_dir / obj.get('image_path', '')
            if image_path.exists():
                try:
                    image_path.unlink()
                except Exception:
                    pass
        
        # 清空向量索引
        self._vector_manager.clear()
        
        # 清空物体数据
        self._objects = {}
        self._metadata = {
            'total_objects': 0,
            'total_samples': 0,
            'last_updated': self._current_timestamp(),
        }
        
        # 持久化
        self._save_index()
        
        logger.info(f"清空所有数据: objects={object_count}, samples={sample_count}")
        return object_count, sample_count
    
    def get_stats(self) -> Dict[str, Any]:
        """获取存储统计"""
        return {
            'total_objects': len(self._objects),
            'total_samples': sum(obj.get('sample_count', 0) for obj in self._objects.values()),
            'last_updated': self._metadata.get('last_updated', 0),
        }
