"""
Perception 全局常量定义

所有边界条件和默认值统一在此定义
"""

# =============================================================================
# 图像相关
# =============================================================================
MAX_IMAGE_SIZE = 4096 * 4096          # 最大图像像素数（16MP）
MIN_IMAGE_SIZE = 64 * 64              # 最小图像像素数
SUPPORTED_IMAGE_FORMATS = ['rgb8', 'bgr8', 'rgba8', 'bgra8']

# =============================================================================
# 分割相关
# =============================================================================
MIN_CONFIDENCE = 0.3                  # 最小分割置信度
MIN_MASK_AREA = 100                   # 最小掩码面积（像素）
MAX_MASK_AREA_RATIO = 0.9             # 最大掩码面积占比

# =============================================================================
# 点云相关
# =============================================================================
MIN_POINT_COUNT = 10                  # 最小有效点数
MAX_INVALID_DEPTH_RATIO = 0.8         # 最大深度无效点占比
MIN_DEPTH_MM = 100                    # 最小有效深度（mm）
MAX_DEPTH_MM = 10000                  # 最大有效深度（mm）

# 边缘深度过滤（解决 RealSense 边缘 flying pixels 问题）
# 三重过滤：Mask 腐蚀 → 深度梯度过滤 → 中位数+MAD 统计过滤
EDGE_FILTER_ENABLED = True            # 是否启用边缘过滤（总开关）

# Step 1: Mask 腐蚀（排除边缘像素）
MASK_EROSION_PIXELS = 1               # Mask 腐蚀像素数（排除边缘 N 像素）

# Step 2: 深度梯度过滤（检测 2D 深度图中的深度跳变）
DEPTH_GRADIENT_FILTER_ENABLED = True  # 是否启用深度梯度过滤
DEPTH_GRADIENT_THRESHOLD = 50.0       # 深度梯度阈值（mm）

# Step 3: 中位数+MAD 统计过滤（排除深度异常值）
DEPTH_OUTLIER_FILTER_ENABLED = True   # 是否启用深度异常值过滤
DEPTH_OUTLIER_MAD_RATIO = 3.5         # MAD 倍数阈值

# 深度图预处理（在反投影前）
DEPTH_MEDIAN_FILTER_ENABLED = True    # 是否启用深度图中值滤波
DEPTH_MEDIAN_FILTER_SIZE = 5         # 中值滤波核大小（必须是奇数，3/5/7）

# 3D空间统计过滤（增强版MAD过滤）
DEPTH_3D_MAD_FILTER_ENABLED = True    # 是否启用3D空间MAD过滤（替代仅Z轴过滤）
DEPTH_3D_MAD_RATIO = 3.0              # 3D空间MAD倍数阈值（通常比Z轴更严格）

# 加权质心计算
WEIGHTED_CENTER_ENABLED = True        # 是否启用加权质心（距离加权）

# =============================================================================
# 向量相关
# =============================================================================
CLIP_EMBEDDING_DIM = 512              # CLIP 向量维度
DINO_EMBEDDING_DIM = 384              # DINOv3 向量维度

# =============================================================================
# 存储相关
# =============================================================================
MAX_OBJECTS = 10000                   # 最大物体数量
MAX_SAMPLES_PER_OBJECT = 100          # 单物体最大样本数
MAX_LABEL_LENGTH = 64                 # 标签最大长度
MAX_DESCRIPTION_LENGTH = 256          # 描述最大长度

# =============================================================================
# 查询相关
# =============================================================================
MAX_TOP_K = 100                       # 最大返回数量
DEFAULT_TOP_K = 5                     # 默认返回数量
DEFAULT_MIN_SIMILARITY = 0.5          # 默认最小相似度

# =============================================================================
# 超时配置（秒）
# =============================================================================
TIMEOUT_SEGMENT = 5.0                 # 分割超时
TIMEOUT_POINTCLOUD = 2.0              # 点云超时
TIMEOUT_VECTORIZE = 6.0               # 向量化超时
TIMEOUT_OBJECT_TARGET = 10.0          # 组合 Action: 分割+点云
TIMEOUT_OBJECT_RECORD = 10.0          # 组合 Action: 向量化+保存
TIMEOUT_OBJECT_PROCESS = 20.0         # 组合 Action: 完整流程
TIMEOUT_FAISS_SEARCH = 1.0            # FAISS 检索
TIMEOUT_FILE_WRITE = 2.0              # 文件写入

# =============================================================================
# 并发配置
# =============================================================================
MAX_QUEUE_SIZE = 10                   # 最大队列长度
QUEUE_TIMEOUT = 30.0                  # 排队超时（秒）

# =============================================================================
# 文件路径
# =============================================================================
DEFAULT_STORAGE_DIR = 'data'          # 默认存储目录（相对于包路径）
DEFAULT_MODEL_DIR = 'models'          # 默认模型目录（相对于包路径）
INDEX_FILENAME = 'index.yaml'         # 物体索引文件名
FAISS_DIR = 'faiss'                   # FAISS 索引目录
IMAGES_DIR = 'images'                 # 图像存储目录
CLIP_INDEX_FILENAME = 'clip_index.faiss'
DINO_INDEX_FILENAME = 'dino_index.faiss'
ID_MAPPING_FILENAME = 'id_mapping.json'

# =============================================================================
# 模型配置
# =============================================================================
CLIP_MODEL_NAME = 'openai/clip-vit-base-patch32'
DINO_MODEL_FILENAME = 'dinov3_vits16_pretrain_lvd1689m-08c60483.pth'

# SAM2 配置
SAM2_CONFIG_FILENAME = 'sam2_config.yaml'
SAM2_MODEL_FILENAME = 'sam2.1_hiera_small.pt'
SAM2_ENGINE_FILENAME = 'sam2_encoder_multi_1024_fp16.engine'

# =============================================================================
# 抓取相关
# =============================================================================
GRASP_MODEL_FILENAME = 'model.pt'
GRASP_CONFIG_FILENAME = 'config.yaml'
DEFAULT_MAX_CANDIDATES = 50           # 默认最大抓取候选数量
DEFAULT_MIN_GRASP_CONFIDENCE = 0.1    # 默认最小抓取置信度（Contact-GraspNet 模型输出范围约 0-0.2）
DEFAULT_NUM_INPUT_POINTS = 20000      # 点云输入点数
TIMEOUT_GRASP = 10.0                  # 抓取推理超时（秒）- 增加到10秒以应对内存不足时的慢速推理
Z_RANGE_MIN = 0.2                     # 点云 Z 轴最小距离（米）
Z_RANGE_MAX = 1.0                     # 点云 Z 轴最大距离（米）

# =============================================================================
# 坐标系
# =============================================================================
DEFAULT_OUTPUT_FRAME_ID = 'camera_color_optical_frame'

