"""
SegmentationModule 单元测试

测试分割模块的基本功能
"""

import pytest
import numpy as np

# 标记为需要 GPU 的测试
pytestmark = pytest.mark.skipif(
    not pytest.importorskip("torch").cuda.is_available(),
    reason="需要 CUDA"
)


class TestSegmentResult:
    """测试 SegmentResult 数据类"""
    
    def test_segment_result_creation(self):
        """测试 SegmentResult 创建"""
        from perception_core.segmentation import SegmentResult
        
        mask = np.zeros((100, 100), dtype=np.uint8)
        cropped = np.zeros((50, 50, 3), dtype=np.uint8)
        
        result = SegmentResult(
            mask=mask,
            cropped_image=cropped,
            confidence=0.9,
            mask_area=1000,
            bbox=(10, 10, 50, 50)
        )
        
        assert result.confidence == 0.9
        assert result.mask_area == 1000
        assert result.bbox == (10, 10, 50, 50)


class TestSegmentationModule:
    """测试 SegmentationModule"""
    
    def test_module_init(self):
        """测试模块初始化"""
        from perception_core.segmentation import SegmentationModule
        
        config = {
            'engine_path': 'models/nanosam_image_encoder.engine',
            'decoder_path': 'models/nanosam_mask_decoder.engine',
            'min_confidence': 0.3,
            'min_mask_area': 100,
        }
        
        module = SegmentationModule(config)
        assert not module.is_ready
    
    def test_create_visualization(self):
        """测试可视化生成"""
        from perception_core.segmentation import SegmentationModule
        
        config = {}
        module = SegmentationModule(config)
        
        # 创建测试数据
        image = np.zeros((100, 100, 3), dtype=np.uint8)
        image[30:70, 30:70] = [255, 255, 255]  # 白色方块
        
        mask = np.zeros((100, 100), dtype=np.uint8)
        mask[30:70, 30:70] = 255
        
        click_point = (50.0, 50.0)
        
        # 生成可视化
        vis = module.create_visualization(image, mask, click_point)
        
        assert vis.shape == image.shape
        assert vis.dtype == np.uint8


class TestSegmentationIntegration:
    """集成测试（需要模型文件）"""
    
    @pytest.mark.skip(reason="需要模型文件")
    async def test_segment_basic(self):
        """测试基本分割功能"""
        from perception_core.segmentation import SegmentationModule
        
        config = {
            'engine_path': 'models/nanosam_image_encoder.engine',
            'decoder_path': 'models/nanosam_mask_decoder.engine',
        }
        
        module = SegmentationModule(config)
        await module.initialize()
        
        assert module.is_ready
        
        # 创建测试图像
        image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        
        # 执行分割
        result = await module.segment(image, 320.0, 240.0)
        
        assert result.mask.shape == (480, 640)
        assert 0.0 <= result.confidence <= 1.0
        assert result.mask_area > 0

