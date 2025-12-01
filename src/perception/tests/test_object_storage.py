"""
ObjectStorageManager 单元测试

测试物体存储管理模块的基本功能
"""

import pytest
import numpy as np
import tempfile
from pathlib import Path


class TestObjectInfo:
    """测试 ObjectInfo 数据类"""
    
    def test_info_creation(self):
        """测试 ObjectInfo 创建"""
        from perception_core.object_storage_manager import ObjectInfo
        
        info = ObjectInfo(
            object_id="obj_001",
            label="cup",
            description="红色杯子",
            created_at=1732704625000,
            updated_at=1732704625000,
            sample_count=1,
        )
        
        assert info.object_id == "obj_001"
        assert info.label == "cup"
        assert info.sample_count == 1


class TestObjectStorageManager:
    """测试 ObjectStorageManager"""
    
    @pytest.fixture
    async def manager(self, async_worker):
        """创建测试用的管理器"""
        pytest.importorskip("faiss")
        from perception_core.vector_manager import VectorManager
        from perception_core.object_storage_manager import ObjectStorageManager
        
        with tempfile.TemporaryDirectory() as tmpdir:
            vector_manager = VectorManager({}, tmpdir, async_worker)
            await vector_manager.initialize()
            
            storage_manager = ObjectStorageManager({}, tmpdir, vector_manager, async_worker)
            await storage_manager.initialize()
            
            yield storage_manager
    
    @pytest.mark.asyncio
    async def test_save_object(self, manager):
        """测试保存物体"""
        image = np.random.randint(0, 255, (100, 100, 3), dtype=np.uint8)
        clip_emb = np.random.randn(512).astype(np.float32)
        dino_emb = np.random.randn(384).astype(np.float32)
        
        object_id, created_at = manager.save_object(
            image, clip_emb, dino_emb,
            label="cup",
            description="红色杯子"
        )
        
        assert object_id == "obj_001"
        assert created_at > 0
        
        # 验证查询
        info = manager.query_by_id(object_id)
        assert info is not None
        assert info.label == "cup"
        assert info.sample_count == 1
    
    @pytest.mark.asyncio
    async def test_add_sample(self, manager):
        """测试增加样本"""
        image = np.random.randint(0, 255, (100, 100, 3), dtype=np.uint8)
        clip_emb = np.random.randn(512).astype(np.float32)
        dino_emb = np.random.randn(384).astype(np.float32)
        
        # 保存物体
        object_id, _ = manager.save_object(image, clip_emb, dino_emb)
        
        # 增加样本
        clip_emb2 = np.random.randn(512).astype(np.float32)
        dino_emb2 = np.random.randn(384).astype(np.float32)
        
        sample_id, total = manager.add_sample(object_id, clip_emb2, dino_emb2)
        
        assert sample_id == "s002"
        assert total == 2
    
    @pytest.mark.asyncio
    async def test_update_object(self, manager):
        """测试更新物体"""
        image = np.random.randint(0, 255, (100, 100, 3), dtype=np.uint8)
        clip_emb = np.random.randn(512).astype(np.float32)
        dino_emb = np.random.randn(384).astype(np.float32)
        
        object_id, _ = manager.save_object(image, clip_emb, dino_emb, label="cup")
        
        # 更新
        manager.update_object(object_id, label="mug", description="新描述")
        
        info = manager.query_by_id(object_id)
        assert info.label == "mug"
        assert info.description == "新描述"
    
    @pytest.mark.asyncio
    async def test_delete_object(self, manager):
        """测试删除物体"""
        image = np.random.randint(0, 255, (100, 100, 3), dtype=np.uint8)
        clip_emb = np.random.randn(512).astype(np.float32)
        dino_emb = np.random.randn(384).astype(np.float32)
        
        object_id, _ = manager.save_object(image, clip_emb, dino_emb)
        
        # 删除
        deleted = manager.delete_object(object_id)
        assert deleted == 1
        
        # 验证已删除
        info = manager.query_by_id(object_id)
        assert info is None
    
    @pytest.mark.asyncio
    async def test_query_by_label(self, manager):
        """测试按标签查询"""
        image = np.random.randint(0, 255, (100, 100, 3), dtype=np.uint8)
        
        # 保存多个物体
        for label in ["cup", "cup", "bottle"]:
            clip_emb = np.random.randn(512).astype(np.float32)
            dino_emb = np.random.randn(384).astype(np.float32)
            manager.save_object(image, clip_emb, dino_emb, label=label)
        
        # 按标签查询
        results = manager.query_by_label("cup")
        assert len(results) == 2
    
    @pytest.mark.asyncio
    async def test_list_objects(self, manager):
        """测试列出物体"""
        image = np.random.randint(0, 255, (100, 100, 3), dtype=np.uint8)
        
        # 保存多个物体
        for i in range(5):
            clip_emb = np.random.randn(512).astype(np.float32)
            dino_emb = np.random.randn(384).astype(np.float32)
            manager.save_object(image, clip_emb, dino_emb)
        
        # 列出
        objects, total = manager.list_objects(limit=3)
        assert len(objects) == 3
        assert total == 5
    
    @pytest.mark.asyncio
    async def test_clear_all(self, manager):
        """测试清空所有"""
        image = np.random.randint(0, 255, (100, 100, 3), dtype=np.uint8)
        
        # 保存物体
        for i in range(3):
            clip_emb = np.random.randn(512).astype(np.float32)
            dino_emb = np.random.randn(384).astype(np.float32)
            manager.save_object(image, clip_emb, dino_emb)
        
        # 清空
        obj_count, sample_count = manager.clear_all()
        assert obj_count == 3
        assert sample_count == 3
        
        # 验证
        objects, total = manager.list_objects()
        assert total == 0
