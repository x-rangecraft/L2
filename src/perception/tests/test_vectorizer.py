"""
VectorizerModule 单元测试

测试向量化模块的基本功能
"""

import pytest
import numpy as np


class TestVectorizeResult:
    """测试 VectorizeResult 数据类"""
    
    def test_result_creation(self):
        """测试 VectorizeResult 创建"""
        from perception_core.vectorizer import VectorizeResult
        
        clip_emb = np.random.randn(512).astype(np.float32)
        dino_emb = np.random.randn(384).astype(np.float32)
        
        result = VectorizeResult(
            clip_embedding=clip_emb,
            dino_embedding=dino_emb
        )
        
        assert result.clip_embedding.shape == (512,)
        assert result.dino_embedding.shape == (384,)


class TestVectorizerModule:
    """测试 VectorizerModule"""
    
    def test_module_init(self):
        """测试模块初始化"""
        from perception_core.vectorizer import VectorizerModule
        
        config = {
            'clip': {
                'model': 'openai/clip-vit-base-patch32',
                'device': 'cuda',
            },
            'dino': {
                'model_path': 'models/dinov2_vits14.pth',
                'device': 'cuda',
                'feature_dim': 384,
            }
        }
        
        module = VectorizerModule(config)
        assert not module.is_ready
    
    def test_compute_similarity(self):
        """测试相似度计算"""
        from perception_core.vectorizer import VectorizerModule
        
        module = VectorizerModule({})
        
        # 相同向量
        v1 = np.array([1.0, 0.0, 0.0], dtype=np.float32)
        v2 = np.array([1.0, 0.0, 0.0], dtype=np.float32)
        
        sim = module.compute_similarity(v1, v2)
        assert sim == pytest.approx(1.0, abs=0.01)
        
        # 正交向量
        v3 = np.array([0.0, 1.0, 0.0], dtype=np.float32)
        sim = module.compute_similarity(v1, v3)
        assert sim == pytest.approx(0.5, abs=0.01)  # (0 + 1) / 2 = 0.5
        
        # 相反向量
        v4 = np.array([-1.0, 0.0, 0.0], dtype=np.float32)
        sim = module.compute_similarity(v1, v4)
        assert sim == pytest.approx(0.0, abs=0.01)  # (-1 + 1) / 2 = 0


class TestVectorizerIntegration:
    """集成测试（需要模型）"""
    
    @pytest.mark.skip(reason="需要下载模型")
    @pytest.mark.asyncio
    async def test_extract_basic(self):
        """测试基本向量提取"""
        from perception_core.vectorizer import VectorizerModule
        
        config = {
            'clip': {'model': 'openai/clip-vit-base-patch32'},
            'dino': {}
        }
        
        module = VectorizerModule(config)
        await module.initialize()
        
        assert module.is_ready
        
        # 创建测试图像
        image = np.random.randint(0, 255, (224, 224, 3), dtype=np.uint8)
        
        # 提取向量
        result = await module.extract(image)
        
        assert result.clip_embedding.shape == (512,)
        assert result.dino_embedding.shape == (384,)
    
    @pytest.mark.skip(reason="需要下载模型")
    @pytest.mark.asyncio
    async def test_encode_text(self):
        """测试文本编码"""
        from perception_core.vectorizer import VectorizerModule
        
        config = {'clip': {'model': 'openai/clip-vit-base-patch32'}}
        
        module = VectorizerModule(config)
        await module.initialize()
        
        # 编码文本
        embedding = await module.encode_text("红色杯子")
        
        assert embedding.shape == (512,)
        assert np.linalg.norm(embedding) == pytest.approx(1.0, abs=0.01)

