"""
VectorManager - 向量索引管理模块

FAISS 向量索引的增删查
"""

import json
import logging
from pathlib import Path
from typing import Dict, Any, List, Tuple, Optional

import numpy as np

from .constants import (
    CLIP_EMBEDDING_DIM,
    DINO_EMBEDDING_DIM,
    FAISS_DIR,
    CLIP_INDEX_FILENAME,
    DINO_INDEX_FILENAME,
    ID_MAPPING_FILENAME,
)
from .error_codes import PerceptionError, ErrorCode
from .async_worker import AsyncWorker

# 延迟导入 FAISS
FAISS_AVAILABLE = False
try:
    import faiss
    FAISS_AVAILABLE = True
except ImportError:
    pass

logger = logging.getLogger(__name__)


class VectorManager:
    """
    向量索引管理器
    
    管理 FAISS 向量索引的增删查操作
    """
    
    def __init__(self, config: Dict[str, Any], storage_dir: str, worker: AsyncWorker):
        """
        初始化配置
        
        Args:
            config: 配置字典，包含:
                - index_type: FAISS 索引类型
                - clip_dimension: CLIP 向量维度
                - dino_dimension: DINOv3 向量维度
            storage_dir: 存储目录路径
        """
        if worker is None:
            raise ValueError("VectorManager 需要有效的 AsyncWorker 实例")
        self._config = config
        self._storage_dir = Path(storage_dir)
        self._worker = worker
        self._ready = False
        
        # 配置参数
        self._index_type = config.get('index_type', 'IndexFlatL2')
        self._clip_dim = config.get('clip_dimension', CLIP_EMBEDDING_DIM)
        self._dino_dim = config.get('dino_dimension', DINO_EMBEDDING_DIM)
        
        # FAISS 索引
        self._clip_index = None
        self._dino_index = None
        
        # ID 映射: {faiss_idx: {"object_id": str, "sample_id": str}}
        self._clip_id_mapping: Dict[int, Dict[str, str]] = {}
        self._dino_id_mapping: Dict[int, Dict[str, str]] = {}
        
        # 下一个可用的索引位置
        self._next_clip_idx = 0
        self._next_dino_idx = 0
        
        # 存储路径
        self._faiss_dir = self._storage_dir / FAISS_DIR
        self._clip_index_path = self._faiss_dir / CLIP_INDEX_FILENAME
        self._dino_index_path = self._faiss_dir / DINO_INDEX_FILENAME
        self._mapping_path = self._faiss_dir / ID_MAPPING_FILENAME
        
        logger.info(f"VectorManager 配置已加载: storage_dir={storage_dir}")
    
    @property
    def is_ready(self) -> bool:
        """检查模块是否就绪"""
        return self._ready
    
    async def initialize(self) -> bool:
        """
        异步初始化，加载或创建 FAISS 索引
        
        Returns:
            bool: 是否初始化成功
        """
        try:
            if not FAISS_AVAILABLE:
                raise PerceptionError(
                    ErrorCode.FAISS_INDEX_LOAD_FAILED,
                    "FAISS 未安装"
                )
            
            # 创建存储目录
            self._faiss_dir.mkdir(parents=True, exist_ok=True)
            
            await self._worker.run_callable(self._load_or_create_indices)
            
            self._ready = True
            logger.info(
                f"VectorManager 初始化完成: "
                f"clip_vectors={self._clip_index.ntotal}, "
                f"dino_vectors={self._dino_index.ntotal}"
            )
            return True
            
        except PerceptionError:
            raise
        except Exception as e:
            logger.error(f"VectorManager 初始化失败: {e}")
            raise PerceptionError(ErrorCode.FAISS_INDEX_LOAD_FAILED, str(e))
    
    def _load_or_create_indices(self):
        """加载或创建 FAISS 索引"""
        # 尝试加载现有索引
        if self._clip_index_path.exists() and self._dino_index_path.exists():
            try:
                self._clip_index = faiss.read_index(str(self._clip_index_path))
                self._dino_index = faiss.read_index(str(self._dino_index_path))
                self._load_id_mapping()
                
                # 设置下一个索引位置
                self._next_clip_idx = self._clip_index.ntotal
                self._next_dino_idx = self._dino_index.ntotal
                
                logger.info("从磁盘加载现有 FAISS 索引")
                return
            except Exception as e:
                logger.warning(f"加载索引失败，创建新索引: {e}")
        
        # 创建新索引
        self._create_new_indices()
    
    def _create_new_indices(self):
        """创建新的 FAISS 索引"""
        if self._index_type == 'IndexFlatL2':
            self._clip_index = faiss.IndexFlatL2(self._clip_dim)
            self._dino_index = faiss.IndexFlatL2(self._dino_dim)
        elif self._index_type == 'IndexFlatIP':
            # 内积（用于归一化向量的余弦相似度）
            self._clip_index = faiss.IndexFlatIP(self._clip_dim)
            self._dino_index = faiss.IndexFlatIP(self._dino_dim)
        else:
            # 默认使用 L2
            self._clip_index = faiss.IndexFlatL2(self._clip_dim)
            self._dino_index = faiss.IndexFlatL2(self._dino_dim)
        
        self._clip_id_mapping = {}
        self._dino_id_mapping = {}
        self._next_clip_idx = 0
        self._next_dino_idx = 0
        
        logger.info(f"创建新 FAISS 索引: {self._index_type}")
    
    def _load_id_mapping(self):
        """加载 ID 映射"""
        if self._mapping_path.exists():
            with open(self._mapping_path, 'r') as f:
                data = json.load(f)
            
            # 转换键为整数
            self._clip_id_mapping = {
                int(k): v for k, v in data.get('clip', {}).items()
            }
            self._dino_id_mapping = {
                int(k): v for k, v in data.get('dino', {}).items()
            }
    
    def _save_id_mapping(self):
        """保存 ID 映射"""
        data = {
            'clip': {str(k): v for k, v in self._clip_id_mapping.items()},
            'dino': {str(k): v for k, v in self._dino_id_mapping.items()}
        }
        
        # 原子写入
        tmp_path = self._mapping_path.with_suffix('.tmp')
        with open(tmp_path, 'w') as f:
            json.dump(data, f, indent=2)
        tmp_path.rename(self._mapping_path)
    
    def add_vectors(
        self,
        clip_emb: np.ndarray,
        dino_emb: np.ndarray,
        object_id: str,
        sample_id: str
    ) -> Tuple[int, int]:
        """
        添加向量到索引
        
        Args:
            clip_emb: CLIP 向量 (512,)
            dino_emb: DINOv3 向量 (384,)
            object_id: 物体 ID
            sample_id: 样本 ID
            
        Returns:
            Tuple[int, int]: (clip_index, dino_index) 在 FAISS 中的位置
        """
        if not self._ready:
            raise PerceptionError(ErrorCode.NODE_NOT_READY)
        
        # 验证维度
        if clip_emb.shape[0] != self._clip_dim:
            raise PerceptionError(
                ErrorCode.VECTOR_DIM_MISMATCH,
                f"CLIP 维度 {clip_emb.shape[0]} != {self._clip_dim}"
            )
        if dino_emb.shape[0] != self._dino_dim:
            raise PerceptionError(
                ErrorCode.VECTOR_DIM_MISMATCH,
                f"DINO 维度 {dino_emb.shape[0]} != {self._dino_dim}"
            )
        
        # 添加到 FAISS
        clip_emb = clip_emb.reshape(1, -1).astype(np.float32)
        dino_emb = dino_emb.reshape(1, -1).astype(np.float32)
        
        self._clip_index.add(clip_emb)
        self._dino_index.add(dino_emb)
        
        # 记录映射
        clip_idx = self._next_clip_idx
        dino_idx = self._next_dino_idx
        
        self._clip_id_mapping[clip_idx] = {
            'object_id': object_id,
            'sample_id': sample_id
        }
        self._dino_id_mapping[dino_idx] = {
            'object_id': object_id,
            'sample_id': sample_id
        }
        
        self._next_clip_idx += 1
        self._next_dino_idx += 1
        
        logger.debug(
            f"添加向量: object_id={object_id}, sample_id={sample_id}, "
            f"clip_idx={clip_idx}, dino_idx={dino_idx}"
        )
        
        return clip_idx, dino_idx
    
    def search_by_clip(
        self,
        query_emb: np.ndarray,
        top_k: int = 5
    ) -> List[Tuple[int, float, str, str]]:
        """
        CLIP 向量检索
        
        Args:
            query_emb: 查询向量 (512,)
            top_k: 返回前 K 个结果
            
        Returns:
            List[Tuple[int, float, str, str]]: [(index, distance, object_id, sample_id), ...]
        """
        if not self._ready:
            raise PerceptionError(ErrorCode.NODE_NOT_READY)
        
        if self._clip_index.ntotal == 0:
            return []
        
        query = query_emb.reshape(1, -1).astype(np.float32)
        k = min(top_k, self._clip_index.ntotal)
        
        distances, indices = self._clip_index.search(query, k)
        
        results = []
        for i, idx in enumerate(indices[0]):
            if idx < 0:
                continue
            mapping = self._clip_id_mapping.get(int(idx), {})
            results.append((
                int(idx),
                float(distances[0][i]),
                mapping.get('object_id', ''),
                mapping.get('sample_id', '')
            ))
        
        return results
    
    def search_by_dino(
        self,
        query_emb: np.ndarray,
        top_k: int = 5
    ) -> List[Tuple[int, float, str, str]]:
        """
        DINOv3 向量检索
        
        Args:
            query_emb: 查询向量 (384,)
            top_k: 返回前 K 个结果
            
        Returns:
            List[Tuple[int, float, str, str]]: [(index, distance, object_id, sample_id), ...]
        """
        if not self._ready:
            raise PerceptionError(ErrorCode.NODE_NOT_READY)
        
        if self._dino_index.ntotal == 0:
            return []
        
        query = query_emb.reshape(1, -1).astype(np.float32)
        k = min(top_k, self._dino_index.ntotal)
        
        distances, indices = self._dino_index.search(query, k)
        
        results = []
        for i, idx in enumerate(indices[0]):
            if idx < 0:
                continue
            mapping = self._dino_id_mapping.get(int(idx), {})
            results.append((
                int(idx),
                float(distances[0][i]),
                mapping.get('object_id', ''),
                mapping.get('sample_id', '')
            ))
        
        return results
    
    def delete_vectors(
        self,
        clip_indices: List[int],
        dino_indices: List[int]
    ) -> bool:
        """
        删除向量（标记删除，通过重建索引实现完全删除）
        
        注意：FAISS IndexFlat 不支持真正的删除，这里只删除映射
        如需完全删除，需要调用 rebuild_indices()
        
        Args:
            clip_indices: 要删除的 CLIP 索引位置
            dino_indices: 要删除的 DINO 索引位置
            
        Returns:
            bool: 是否成功
        """
        if not self._ready:
            raise PerceptionError(ErrorCode.NODE_NOT_READY)
        
        # 从映射中删除
        for idx in clip_indices:
            self._clip_id_mapping.pop(idx, None)
        
        for idx in dino_indices:
            self._dino_id_mapping.pop(idx, None)
        
        logger.debug(f"标记删除向量: clip={clip_indices}, dino={dino_indices}")
        return True
    
    def get_indices_by_object(self, object_id: str) -> Tuple[List[int], List[int]]:
        """
        获取物体的所有向量索引
        
        Args:
            object_id: 物体 ID
            
        Returns:
            Tuple[List[int], List[int]]: (clip_indices, dino_indices)
        """
        clip_indices = [
            idx for idx, mapping in self._clip_id_mapping.items()
            if mapping.get('object_id') == object_id
        ]
        dino_indices = [
            idx for idx, mapping in self._dino_id_mapping.items()
            if mapping.get('object_id') == object_id
        ]
        return clip_indices, dino_indices
    
    def get_indices_by_sample(
        self,
        object_id: str,
        sample_id: str
    ) -> Tuple[Optional[int], Optional[int]]:
        """
        获取样本的向量索引
        
        Args:
            object_id: 物体 ID
            sample_id: 样本 ID
            
        Returns:
            Tuple[Optional[int], Optional[int]]: (clip_index, dino_index)
        """
        clip_idx = None
        dino_idx = None
        
        for idx, mapping in self._clip_id_mapping.items():
            if mapping.get('object_id') == object_id and mapping.get('sample_id') == sample_id:
                clip_idx = idx
                break
        
        for idx, mapping in self._dino_id_mapping.items():
            if mapping.get('object_id') == object_id and mapping.get('sample_id') == sample_id:
                dino_idx = idx
                break
        
        return clip_idx, dino_idx
    
    def save(self) -> bool:
        """
        持久化索引到磁盘
        
        Returns:
            bool: 是否成功
        """
        if not self._ready:
            raise PerceptionError(ErrorCode.NODE_NOT_READY)
        
        try:
            # 保存 FAISS 索引
            faiss.write_index(self._clip_index, str(self._clip_index_path))
            faiss.write_index(self._dino_index, str(self._dino_index_path))
            
            # 保存 ID 映射
            self._save_id_mapping()
            
            logger.info(
                f"FAISS 索引已保存: "
                f"clip={self._clip_index.ntotal}, dino={self._dino_index.ntotal}"
            )
            return True
            
        except Exception as e:
            raise PerceptionError(ErrorCode.FAISS_SAVE_FAILED, str(e))
    
    def clear(self) -> bool:
        """
        清空索引
        
        Returns:
            bool: 是否成功
        """
        if not self._ready:
            raise PerceptionError(ErrorCode.NODE_NOT_READY)
        
        # 重建空索引
        self._create_new_indices()
        
        # 删除磁盘文件
        if self._clip_index_path.exists():
            self._clip_index_path.unlink()
        if self._dino_index_path.exists():
            self._dino_index_path.unlink()
        if self._mapping_path.exists():
            self._mapping_path.unlink()
        
        logger.info("FAISS 索引已清空")
        return True
    
    def get_stats(self) -> Dict[str, Any]:
        """获取索引统计信息"""
        return {
            'clip_vectors': self._clip_index.ntotal if self._clip_index else 0,
            'dino_vectors': self._dino_index.ntotal if self._dino_index else 0,
            'clip_mappings': len(self._clip_id_mapping),
            'dino_mappings': len(self._dino_id_mapping),
            'index_type': self._index_type,
        }
