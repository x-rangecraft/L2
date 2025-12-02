"""
Perception 错误码定义

错误码格式: [EXXX] 错误描述
- 1xxx: 初始化错误
- 2xxx: 分割错误
- 3xxx: 点云错误
- 4xxx: 向量化错误
- 5xxx: 存储错误
- 6xxx: 查询错误
- 9xxx: 系统错误
"""

from enum import IntEnum
from typing import Optional


class ErrorCode(IntEnum):
    """错误码枚举"""
    
    # =========================================================================
    # 1xxx - 初始化错误
    # =========================================================================
    NANOSAM_ENGINE_LOAD_FAILED = 1001       # NanoSAM 引擎加载失败
    NANOSAM_DECODER_LOAD_FAILED = 1002      # NanoSAM 解码器加载失败
    CLIP_MODEL_LOAD_FAILED = 1003           # CLIP 模型加载失败
    DINO_MODEL_LOAD_FAILED = 1004           # DINOv3 模型加载失败
    FAISS_INDEX_LOAD_FAILED = 1005          # FAISS 索引加载失败
    STORAGE_DIR_CREATE_FAILED = 1006        # 存储目录创建失败
    CONFIG_READ_FAILED = 1007               # 配置文件读取失败
    GPU_MEMORY_INSUFFICIENT = 1010          # GPU 内存不足
    CUDA_NOT_AVAILABLE = 1011               # CUDA 不可用
    
    # =========================================================================
    # 2xxx - 分割错误
    # =========================================================================
    IMAGE_EMPTY = 2001                      # 输入图像为空
    IMAGE_FORMAT_UNSUPPORTED = 2002         # 图像格式不支持
    CLICK_OUT_OF_RANGE = 2003               # 点击坐标超出图像范围
    SEGMENT_RESULT_EMPTY = 2004             # 分割结果为空（未检测到目标）
    CONFIDENCE_TOO_LOW = 2005               # 分割置信度过低
    MASK_AREA_TOO_SMALL = 2006              # 掩码面积过小
    NANOSAM_INFERENCE_TIMEOUT = 2007        # NanoSAM 推理超时
    NANOSAM_INFERENCE_ERROR = 2008          # NanoSAM 推理异常
    MASK_AREA_TOO_LARGE = 2009              # 掩码面积过大
    
    # =========================================================================
    # 3xxx - 点云错误
    # =========================================================================
    DEPTH_IMAGE_EMPTY = 3001                # 深度图为空
    DEPTH_INVALID_RATIO_HIGH = 3002         # 深度图无效点过多
    CAMERA_INFO_INVALID = 3003              # 相机内参无效
    MASK_DEPTH_SIZE_MISMATCH = 3004         # 掩码与深度图尺寸不匹配
    POINT_COUNT_INSUFFICIENT = 3005         # 有效点数量不足
    POINTCLOUD_COMPUTE_TIMEOUT = 3006       # 点云计算超时
    
    # =========================================================================
    # 7xxx - 抓取错误
    # =========================================================================
    GRASP_MODEL_LOAD_FAILED = 7001          # 抓取模型加载失败
    GRASP_INFERENCE_FAILED = 7002           # 抓取推理失败
    GRASP_INFERENCE_TIMEOUT = 7003          # 抓取推理超时
    GRASP_NO_CANDIDATES = 7004              # 无有效抓取候选
    
    # =========================================================================
    # 4xxx - 向量化错误
    # =========================================================================
    CLIP_ENCODE_FAILED = 4001               # CLIP 编码失败
    DINO_ENCODE_FAILED = 4002               # DINOv3 编码失败
    IMAGE_PREPROCESS_FAILED = 4003          # 图像预处理失败
    TEXT_ENCODE_FAILED = 4004               # 文本编码失败（空文本）
    VECTORIZE_TIMEOUT = 4005                # 向量化超时
    
    # =========================================================================
    # 5xxx - 存储错误
    # =========================================================================
    OBJECT_NOT_FOUND = 5001                 # 物体 ID 不存在
    SAMPLE_NOT_FOUND = 5002                 # 样本 ID 不存在
    IMAGE_SAVE_FAILED = 5003                # 图像保存失败
    INDEX_WRITE_FAILED = 5004               # index.yaml 写入失败
    FAISS_SAVE_FAILED = 5005                # FAISS 索引保存失败
    VECTOR_DIM_MISMATCH = 5006              # 向量维度不匹配
    OBJECT_ALREADY_EXISTS = 5007            # 物体已存在（重复 ID）
    CANNOT_DELETE_LAST_SAMPLE = 5008        # 删除最后一个样本（物体至少保留一个样本）
    
    # =========================================================================
    # 6xxx - 查询错误
    # =========================================================================
    QUERY_TEXT_EMPTY = 6001                 # 查询文本为空
    QUERY_IMAGE_EMPTY = 6002                # 查询图像为空
    NO_MATCH_FOUND = 6003                   # 无匹配结果
    LABEL_NOT_FOUND = 6004                  # 标签不存在
    
    # =========================================================================
    # 9xxx - 系统错误
    # =========================================================================
    NODE_NOT_READY = 9001                   # 节点未就绪
    ACTION_CANCELLED = 9002                 # Action 被取消
    INTERNAL_ERROR = 9003                   # 内部异常
    RESOURCE_BUSY = 9004                    # 资源繁忙（排队中）


# 错误码描述映射
ERROR_MESSAGES = {
    # 1xxx - 初始化错误
    ErrorCode.NANOSAM_ENGINE_LOAD_FAILED: "NanoSAM 引擎加载失败",
    ErrorCode.NANOSAM_DECODER_LOAD_FAILED: "NanoSAM 解码器加载失败",
    ErrorCode.CLIP_MODEL_LOAD_FAILED: "CLIP 模型加载失败",
    ErrorCode.DINO_MODEL_LOAD_FAILED: "DINOv3 模型加载失败",
    ErrorCode.FAISS_INDEX_LOAD_FAILED: "FAISS 索引加载失败",
    ErrorCode.STORAGE_DIR_CREATE_FAILED: "存储目录创建失败",
    ErrorCode.CONFIG_READ_FAILED: "配置文件读取失败",
    ErrorCode.GPU_MEMORY_INSUFFICIENT: "GPU 内存不足",
    ErrorCode.CUDA_NOT_AVAILABLE: "CUDA 不可用",
    
    # 2xxx - 分割错误
    ErrorCode.IMAGE_EMPTY: "输入图像为空",
    ErrorCode.IMAGE_FORMAT_UNSUPPORTED: "图像格式不支持",
    ErrorCode.CLICK_OUT_OF_RANGE: "点击坐标超出图像范围",
    ErrorCode.SEGMENT_RESULT_EMPTY: "分割结果为空（未检测到目标）",
    ErrorCode.CONFIDENCE_TOO_LOW: "分割置信度过低",
    ErrorCode.MASK_AREA_TOO_SMALL: "掩码面积过小",
    ErrorCode.NANOSAM_INFERENCE_TIMEOUT: "NanoSAM 推理超时",
    ErrorCode.NANOSAM_INFERENCE_ERROR: "NanoSAM 推理异常",
    ErrorCode.MASK_AREA_TOO_LARGE: "掩码面积过大",
    
    # 3xxx - 点云错误
    ErrorCode.DEPTH_IMAGE_EMPTY: "深度图为空",
    ErrorCode.DEPTH_INVALID_RATIO_HIGH: "深度图无效点过多",
    ErrorCode.CAMERA_INFO_INVALID: "相机内参无效",
    ErrorCode.MASK_DEPTH_SIZE_MISMATCH: "掩码与深度图尺寸不匹配",
    ErrorCode.POINT_COUNT_INSUFFICIENT: "有效点数量不足",
    ErrorCode.POINTCLOUD_COMPUTE_TIMEOUT: "点云计算超时",
    
    # 7xxx - 抓取错误
    ErrorCode.GRASP_MODEL_LOAD_FAILED: "抓取模型加载失败",
    ErrorCode.GRASP_INFERENCE_FAILED: "抓取推理失败",
    ErrorCode.GRASP_INFERENCE_TIMEOUT: "抓取推理超时",
    ErrorCode.GRASP_NO_CANDIDATES: "无有效抓取候选",
    
    # 4xxx - 向量化错误
    ErrorCode.CLIP_ENCODE_FAILED: "CLIP 编码失败",
    ErrorCode.DINO_ENCODE_FAILED: "DINOv3 编码失败",
    ErrorCode.IMAGE_PREPROCESS_FAILED: "图像预处理失败",
    ErrorCode.TEXT_ENCODE_FAILED: "文本编码失败（空文本）",
    ErrorCode.VECTORIZE_TIMEOUT: "向量化超时",
    
    # 5xxx - 存储错误
    ErrorCode.OBJECT_NOT_FOUND: "物体 ID 不存在",
    ErrorCode.SAMPLE_NOT_FOUND: "样本 ID 不存在",
    ErrorCode.IMAGE_SAVE_FAILED: "图像保存失败",
    ErrorCode.INDEX_WRITE_FAILED: "index.yaml 写入失败",
    ErrorCode.FAISS_SAVE_FAILED: "FAISS 索引保存失败",
    ErrorCode.VECTOR_DIM_MISMATCH: "向量维度不匹配",
    ErrorCode.OBJECT_ALREADY_EXISTS: "物体已存在（重复 ID）",
    ErrorCode.CANNOT_DELETE_LAST_SAMPLE: "不能删除最后一个样本",
    
    # 6xxx - 查询错误
    ErrorCode.QUERY_TEXT_EMPTY: "查询文本为空",
    ErrorCode.QUERY_IMAGE_EMPTY: "查询图像为空",
    ErrorCode.NO_MATCH_FOUND: "无匹配结果",
    ErrorCode.LABEL_NOT_FOUND: "标签不存在",
    
    # 9xxx - 系统错误
    ErrorCode.NODE_NOT_READY: "节点未就绪",
    ErrorCode.ACTION_CANCELLED: "Action 被取消",
    ErrorCode.INTERNAL_ERROR: "内部异常",
    ErrorCode.RESOURCE_BUSY: "资源繁忙（排队中）",
}


class PerceptionError(Exception):
    """Perception 统一异常类"""
    
    def __init__(self, code: ErrorCode, detail: Optional[str] = None):
        """
        初始化异常
        
        Args:
            code: 错误码
            detail: 额外的错误详情
        """
        self.code = code
        self.detail = detail
        self.message = self._format_message()
        super().__init__(self.message)
    
    def _format_message(self) -> str:
        """格式化错误消息: [EXXX] 错误描述"""
        base_msg = ERROR_MESSAGES.get(self.code, "未知错误")
        formatted = f"[E{self.code.value}] {base_msg}"
        if self.detail:
            formatted += f": {self.detail}"
        return formatted
    
    def to_error_message(self) -> str:
        """返回用于 ROS 消息的错误字符串"""
        return self.message
    
    @classmethod
    def from_code(cls, code: int, detail: Optional[str] = None) -> 'PerceptionError':
        """从错误码整数创建异常"""
        try:
            error_code = ErrorCode(code)
        except ValueError:
            error_code = ErrorCode.INTERNAL_ERROR
            detail = f"未知错误码: {code}"
        return cls(error_code, detail)
