"""
PointCloudModule 单元测试

测试点云计算模块的基本功能
"""

import pytest
import numpy as np

from sensor_msgs.msg import CameraInfo


class TestPointCloudResult:
    """测试 PointCloudResult 数据类"""
    
    def test_result_creation(self):
        """测试 PointCloudResult 创建"""
        from perception_core.pointcloud import PointCloudResult
        from sensor_msgs.msg import PointCloud2
        
        pc = PointCloud2()
        result = PointCloudResult(
            point_cloud=pc,
            center_3d=(0.1, 0.2, 0.5),
            bbox_min=(0.0, 0.0, 0.3),
            bbox_max=(0.2, 0.4, 0.7),
            point_count=100
        )
        
        assert result.center_3d == (0.1, 0.2, 0.5)
        assert result.point_count == 100


class TestPointCloudModule:
    """测试 PointCloudModule"""
    
    def test_module_init(self):
        """测试模块初始化"""
        from perception_core.pointcloud import PointCloudModule
        
        config = {
            'output_frame_id': 'camera_color_optical_frame',
            'min_point_count': 10,
            'min_depth_mm': 100,
            'max_depth_mm': 10000,
        }
        
        module = PointCloudModule(config)
        assert not module.is_ready
    
    @pytest.mark.asyncio
    async def test_initialize(self):
        """测试异步初始化"""
        from perception_core.pointcloud import PointCloudModule
        
        config = {}
        module = PointCloudModule(config)
        
        result = await module.initialize()
        assert result is True
        assert module.is_ready
    
    def test_pointcloud2_conversion(self):
        """测试 PointCloud2 转换"""
        from perception_core.pointcloud import PointCloudModule
        
        # 创建测试点云
        points = np.array([
            [0.1, 0.2, 0.5],
            [0.2, 0.3, 0.6],
            [0.3, 0.4, 0.7],
        ], dtype=np.float32)
        
        module = PointCloudModule({})
        pc_msg = module._create_pointcloud2(points)
        
        # 验证消息属性
        assert pc_msg.width == 3
        assert pc_msg.height == 1
        assert len(pc_msg.fields) == 3
        
        # 反向转换
        recovered = PointCloudModule.pointcloud2_to_array(pc_msg)
        np.testing.assert_array_almost_equal(points, recovered)
    
    @pytest.mark.asyncio
    async def test_compute_basic(self):
        """测试基本点云计算"""
        from perception_core.pointcloud import PointCloudModule
        
        config = {
            'min_point_count': 5,
            'min_depth_mm': 100,
            'max_depth_mm': 10000,
        }
        module = PointCloudModule(config)
        await module.initialize()
        
        # 创建测试数据
        h, w = 100, 100
        
        # 掩码：中心 20x20 区域
        mask = np.zeros((h, w), dtype=np.uint8)
        mask[40:60, 40:60] = 255
        
        # 深度图：500mm
        depth = np.zeros((h, w), dtype=np.uint16)
        depth[40:60, 40:60] = 500  # 500mm = 0.5m
        
        # 相机内参
        camera_info = CameraInfo()
        camera_info.k = [
            500.0, 0.0, 50.0,  # fx, 0, cx
            0.0, 500.0, 50.0,  # 0, fy, cy
            0.0, 0.0, 1.0      # 0, 0, 1
        ]
        
        # 计算点云
        result = await module.compute(mask, depth, camera_info)
        
        # 验证结果
        assert result.point_count == 400  # 20x20
        assert result.center_3d[2] == pytest.approx(0.5, abs=0.01)  # Z = 0.5m
        assert result.bbox_min[2] == pytest.approx(0.5, abs=0.01)
        assert result.bbox_max[2] == pytest.approx(0.5, abs=0.01)


class TestPointCloudErrors:
    """测试错误处理"""
    
    @pytest.mark.asyncio
    async def test_empty_depth(self):
        """测试空深度图错误"""
        from perception_core.pointcloud import PointCloudModule
        from perception_core.error_codes import PerceptionError, ErrorCode
        
        module = PointCloudModule({})
        await module.initialize()
        
        mask = np.zeros((100, 100), dtype=np.uint8)
        depth = np.array([])
        camera_info = CameraInfo()
        
        with pytest.raises(PerceptionError) as exc_info:
            await module.compute(mask, depth, camera_info)
        
        assert exc_info.value.code == ErrorCode.DEPTH_IMAGE_EMPTY
    
    @pytest.mark.asyncio
    async def test_size_mismatch(self):
        """测试尺寸不匹配错误"""
        from perception_core.pointcloud import PointCloudModule
        from perception_core.error_codes import PerceptionError, ErrorCode
        
        module = PointCloudModule({})
        await module.initialize()
        
        mask = np.zeros((100, 100), dtype=np.uint8)
        depth = np.zeros((200, 200), dtype=np.uint16)
        camera_info = CameraInfo()
        
        with pytest.raises(PerceptionError) as exc_info:
            await module.compute(mask, depth, camera_info)
        
        assert exc_info.value.code == ErrorCode.MASK_DEPTH_SIZE_MISMATCH

