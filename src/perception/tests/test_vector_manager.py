"""
VectorManager 单元测试

测试向量索引管理模块的基本功能
"""

import pytest
import numpy as np
import tempfile
from pathlib import Path


class TestVectorManager:
    """测试 VectorManager"""
    
    def test_module_init(self, async_worker):
        """测试模块初始化"""
        from perception_core.vector_manager import VectorManager
        
        with tempfile.TemporaryDirectory() as tmpdir:
            config = {
                'index_type': 'IndexFlatL2',
                'clip_dimension': 512,
                'dino_dimension': 384,
            }
            
            manager = VectorManager(config, tmpdir, async_worker)
            assert not manager.is_ready
    
    @pytest.mark.asyncio
    async def test_initialize(self, async_worker):
        """测试异步初始化"""
        pytest.importorskip("faiss")
        from perception_core.vector_manager import VectorManager
        
        with tempfile.TemporaryDirectory() as tmpdir:
            config = {}
            manager = VectorManager(config, tmpdir, async_worker)
            
            result = await manager.initialize()
            assert result is True
            assert manager.is_ready
    
    @pytest.mark.asyncio
    async def test_add_and_search(self, async_worker):
        """测试添加和搜索"""
        pytest.importorskip("faiss")
        from perception_core.vector_manager import VectorManager
        
        with tempfile.TemporaryDirectory() as tmpdir:
            manager = VectorManager({}, tmpdir, async_worker)
            await manager.initialize()
            
            # 添加向量
            clip_emb = np.random.randn(512).astype(np.float32)
            dino_emb = np.random.randn(384).astype(np.float32)
            
            clip_idx, dino_idx = manager.add_vectors(
                clip_emb, dino_emb, "obj_001", "s001"
            )
            
            assert clip_idx == 0
            assert dino_idx == 0
            
            # 搜索
            results = manager.search_by_clip(clip_emb, top_k=5)
            assert len(results) == 1
            assert results[0][2] == "obj_001"  # object_id
            assert results[0][3] == "s001"     # sample_id
    
    @pytest.mark.asyncio
    async def test_multiple_samples(self, async_worker):
        """测试多样本"""
        pytest.importorskip("faiss")
        from perception_core.vector_manager import VectorManager
        
        with tempfile.TemporaryDirectory() as tmpdir:
            manager = VectorManager({}, tmpdir, async_worker)
            await manager.initialize()
            
            # 添加多个样本
            for i in range(3):
                clip_emb = np.random.randn(512).astype(np.float32)
                dino_emb = np.random.randn(384).astype(np.float32)
                manager.add_vectors(clip_emb, dino_emb, "obj_001", f"s00{i+1}")
            
            # 获取物体的所有索引
            clip_indices, dino_indices = manager.get_indices_by_object("obj_001")
            assert len(clip_indices) == 3
            assert len(dino_indices) == 3
    
    @pytest.mark.asyncio
    async def test_save_and_load(self, async_worker):
        """测试保存和加载"""
        pytest.importorskip("faiss")
        from perception_core.vector_manager import VectorManager
        
        with tempfile.TemporaryDirectory() as tmpdir:
            # 创建并添加数据
            manager1 = VectorManager({}, tmpdir, async_worker)
            await manager1.initialize()
            
            clip_emb = np.random.randn(512).astype(np.float32)
            dino_emb = np.random.randn(384).astype(np.float32)
            manager1.add_vectors(clip_emb, dino_emb, "obj_001", "s001")
            manager1.save()
            
            # 重新加载
            manager2 = VectorManager({}, tmpdir, async_worker)
            await manager2.initialize()
            
            stats = manager2.get_stats()
            assert stats['clip_vectors'] == 1
            assert stats['dino_vectors'] == 1
    
    @pytest.mark.asyncio
    async def test_clear(self, async_worker):
        """测试清空索引"""
        pytest.importorskip("faiss")
        from perception_core.vector_manager import VectorManager
        
        with tempfile.TemporaryDirectory() as tmpdir:
            manager = VectorManager({}, tmpdir, async_worker)
            await manager.initialize()
            
            # 添加数据
            clip_emb = np.random.randn(512).astype(np.float32)
            dino_emb = np.random.randn(384).astype(np.float32)
            manager.add_vectors(clip_emb, dino_emb, "obj_001", "s001")
            
            # 清空
            manager.clear()
            
            stats = manager.get_stats()
            assert stats['clip_vectors'] == 0
            assert stats['dino_vectors'] == 0
