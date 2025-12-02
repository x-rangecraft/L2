# Grasp Service 设计文档

## 概述

本文档描述 `/perception/service/grasp` 的设计方案，用于基于点云生成抓取位姿候选。

| 项目 | 内容 |
|------|------|
| **Service 名称** | `/perception/service/grasp` |
| **接口类型** | Service（同步调用，~200ms 响应） |
| **核心网络** | Contact-GraspNet |
| **输入** | 点云数据（相机坐标系） |
| **输出** | 抓取候选数组（位置 + 四元数，相机坐标系） |

### 为什么选择 Service 而非 Action

| 考量点 | 结论 |
|-------|------|
| 推理时间 | ~150-200ms，Service 可接受 |
| 进度反馈 | 不需要（单次推理，无中间状态） |
| 取消功能 | 不需要（推理很快完成） |
| 实现复杂度 | Service 更简单 |
| 调用复杂度 | Service 更简单 |

---

## 1. 关键技术选型

### 1.1 方案总结

| 决策点 | 最终选择 | 备选方案 |
|-------|---------|---------|
| 抓取网络 | **Contact-GraspNet** | GraspNet-1Billion, VGN, GPD |
| 部署方式 | **PyTorch + CUDA** | TensorRT（需自定义插件） |
| 深度学习框架 | **PyTorch** | TensorFlow |
| 推理精度 | **FP32**（可升级 FP16） | INT8 |
| 模型权重 | **官方预训练** | 自定义微调 |
| 输出坐标系 | **相机坐标系** | base_link |
| 接口类型 | **Service** | Action |

### 1.2 为什么选择 Contact-GraspNet

| 优势 | 说明 |
|------|------|
| 端到端推理 | 直接从点云生成 6-DoF 抓取位姿 |
| 多候选输出 | 单次推理输出 50-100 个候选，按置信度排序 |
| 接触点预测 | 预测最佳接触区域，提高抓取稳定性 |
| NVIDIA 官方 | 有预训练权重和完整代码 |

### 1.3 为什么选择 PyTorch 而非 TensorRT

| 对比项 | TensorRT | PyTorch + CUDA |
|-------|----------|----------------|
| 标准算子支持 | ✅ 完整 | ✅ 完整 |
| PointNet++ 算子 | ❌ 需自定义插件 | ✅ pointnet2_ops 提供 |
| 部署复杂度 | 高（需写 4-6 个 CUDA 插件） | 低（编译现成扩展） |
| 推理性能 | 快 ~1.5-2x | 中等 |
| 开发周期 | 长（2-4 周） | 短（1-2 天） |

**结论**：TensorRT 标准库不包含 PointNet++ 所需的点云专用算子（ball_query、group_points、furthest_point_sampling 等），需自行编写 CUDA 插件。PyTorch 方案可直接使用社区的 `pointnet2_ops` 扩展，开发成本低。

---

## 2. 技术选型过程与局限性

### 2.1 TensorRT 方案的障碍

Contact-GraspNet 使用 PointNet++ 作为骨干网络，依赖以下 **点云专用算子**：

| 算子 | 作用 | TensorRT 支持 |
|------|------|--------------|
| `ball_query` | 球形邻域搜索 | ❌ 不支持 |
| `group_points` | 点分组 | ❌ 不支持 |
| `furthest_point_sampling` | 最远点采样 | ❌ 不支持 |
| `three_nn` | K 近邻搜索 | ❌ 不支持 |
| `three_interpolate` | 特征插值 | ❌ 不支持 |

这些算子：
- **不是 TensorRT 版本问题**：从 TensorRT 7.x 到 10.x 均不包含
- **是设计范围问题**：TensorRT 针对 CNN/Transformer 优化，点云网络超出其设计范围
- **需自定义插件**：每个算子需编写 CUDA kernel + TensorRT Plugin 封装

### 2.2 PyTorch 方案的可行性

```
PyTorch 生态提供了这些算子的 CUDA 实现：

pointnet2_ops/
├── ball_query.cu        # CUDA 实现
├── group_points.cu      # CUDA 实现
├── sampling.cu          # FPS 实现
├── three_nn.cu          # 近邻搜索
└── three_interpolate.cu # 插值
```

在 Jetson aarch64 上编译后可直接使用，无需开发插件。

### 2.3 性能对比

| 部署方式 | 推理延迟（估算） | 开发成本 |
|---------|----------------|---------|
| PyTorch FP32 | ~150-200ms | 低 |
| PyTorch FP16 | ~80-120ms | 低 |
| PyTorch + torch.compile | ~60-100ms | 低 |
| TensorRT（假设完成） | ~40-60ms | **高** |

### 2.4 深度相机有效区域

当前设备：**Intel RealSense D435I**

| 参数 | 数值 |
|------|------|
| 最小距离 | 0.11m |
| 最大距离 | 10m |
| **推荐工作范围** | **0.3m - 3m** |
| **最佳精度范围（黄金区域）** | **0.4m - 0.8m** |

#### 黄金区域定义

```
距离 0.4m - 0.8m：
├── 深度精度：±2-4mm
├── 点云密度：适合抓取识别
├── 视野覆盖：典型桌面工作区
└── Contact-GraspNet 训练数据分布匹配
```

#### 深度精度 vs 距离

| 距离 | 深度误差 | 适用性 |
|------|---------|--------|
| 0.3m | ±1-2mm | ✅ 精度高，视野小 |
| 0.5m | ±2-3mm | ✅ **最佳平衡点** |
| 0.8m | ±3-5mm | ✅ 视野大，精度可接受 |
| 1.0m | ±4-6mm | ⚠️ 精度下降 |
| 2.0m | ±10-15mm | ❌ 不推荐用于精细抓取 |

### 2.5 精度误差来源

| 环节 | 典型误差 | 可控性 |
|------|---------|--------|
| D435I 深度噪声 | ±2-5mm | 硬件限制 |
| 相机外参标定 | ±1-3mm | 可优化 |
| Contact-GraspNet 预测 | ±5-10mm | 网络精度 |
| 坐标转换计算 | <0.01mm | 可忽略 |
| **综合误差** | **±5-15mm** | - |

### 2.6 坐标转换策略

**选择：先推理，后转位姿**

| 方案 | 转换数据量 | 计算量 |
|------|-----------|--------|
| 先转点云，再推理 | ~20000 点 × 3 | 大 |
| **先推理，再转位姿** | ~100 位姿 × 7 | **小（200x）** |

两种方案精度相同，但计算量差距 200-400 倍。因此：
- **perception 输出**：相机坐标系下的抓取位姿
- **调用方（skill）**：通过 TF_tools 转换到 base_link

---

## 3. 整体结构设计

### 3.1 模块架构

```
┌─────────────────────────────────────────────────────────────┐
│                    Perception Node                          │
│                                                             │
│  ┌─────────────────────────────────────────────────────┐   │
│  │                   核心模块                           │   │
│  │  ┌──────────┐ ┌──────────┐ ┌──────────┐            │   │
│  │  │ NanoSAM  │ │PointCloud│ │Contact-  │            │   │
│  │  │  分割    │ │   计算   │ │GraspNet  │  ← 新增    │   │
│  │  └──────────┘ └──────────┘ └──────────┘            │   │
│  │                                                      │   │
│  │  ┌──────────┐ ┌──────────┐ ┌──────────┐            │   │
│  │  │   CLIP   │ │ DINOv3   │ │  FAISS   │            │   │
│  │  │  向量化  │ │  向量化  │ │ 向量索引 │            │   │
│  │  └──────────┘ └──────────┘ └──────────┘            │   │
│  └─────────────────────────────────────────────────────┘   │
│                           │                                 │
│                           ▼                                 │
│  ┌─────────────────────────────────────────────────────┐   │
│  │                    对外接口                          │   │
│  │                                                      │   │
│  │  Action:                                             │   │
│  │    /perception/action/segment                        │   │
│  │    /perception/action/pointcloud                     │   │
│  │    /perception/action/vectorize                      │   │
│  │    /perception/action/object_target                  │   │
│  │    /perception/action/object_record                  │   │
│  │                                                      │   │
│  │  Service:                                            │   │
│  │    /perception/service/grasp        ← 新增          │   │
│  │    /perception/service/save_object                   │   │
│  │    /perception/service/query_by_*                    │   │
│  └─────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
```

### 3.2 数据流设计

```
┌─────────────────┐          ┌─────────────────────────────────┐
│    调用方       │          │      Perception Node            │
│  (robot_skill)  │          │                                 │
└────────┬────────┘          └────────────────┬────────────────┘
         │                                    │
         │  ═══════ Grasp Service ══════════ │
         │                                    │
         │  Request:                          │
         │    point_cloud (相机坐标系)        │
         │    [可选] mask                     │
         │    [可选] max_candidates           │
         │    [可选] min_confidence           │
         │ ─────────────────────────────────► │
         │                                    │  Contact-GraspNet
         │                                    │  推理 (~200ms)
         │                                    │
         │  Response:                         │
         │    success: true                   │
         │    candidates: [                   │
         │      {pos, quat, width, conf},     │
         │      {pos, quat, width, conf},     │
         │      ...                           │
         │    ]                               │
         │ ◄───────────────────────────────── │
         │                                    │
         │  ══════════ TF 转换 ══════════════ │
         │                                    │
         │  调用 /tf_tools/transform_points   │
         │  camera_frame → base_link          │
         │                                    │
         │  ══════════ 执行抓取 ═════════════ │
         │                                    │
         │  依次尝试候选，IK 求解             │
         │  发送 /robot_driver/action/robot   │
         │                                    │
```

### 3.3 Service 定义

```yaml
# srv/Grasp.srv
# Service: /perception/service/grasp

#==================== Request ====================
sensor_msgs/PointCloud2 point_cloud    # [必填] 点云数据（相机坐标系）
sensor_msgs/Image mask                 # [可选] 分割掩码，限定目标区域
int32 max_candidates                   # [可选] 最大候选数量，默认 50
float32 min_confidence                 # [可选] 最小置信度阈值，默认 0.5
---
#==================== Response ====================
bool success                           # 是否成功
string error_message                   # 错误信息（失败时）

GraspCandidate[] candidates            # 抓取候选数组（按置信度降序）
```

### 3.4 GraspCandidate 消息定义

```yaml
# msg/GraspCandidate.msg

# 位置（相机坐标系）
geometry_msgs/Point position           # 夹爪中心位置 (x, y, z)

# 姿态（相机坐标系）
geometry_msgs/Quaternion orientation   # 夹爪朝向四元数 (x, y, z, w)

# 抓取参数
float32 width                          # 建议夹爪开口宽度（米）
float32 confidence                     # 置信度 (0.0 - 1.0)

# 可选：接触点信息
geometry_msgs/Point[] contact_points   # 预测的接触点（如果有）
```

### 3.5 GraspModule 实现结构

```python
# src/perception_core/grasp_module.py

class GraspModule:
    """
    Contact-GraspNet 抓取推理模块
    
    参考 SegmentationModule 的实现模式
    """
    
    def __init__(self, config: Dict[str, Any], worker: AsyncWorker):
        """
        初始化
        
        Args:
            config: 配置字典
                - model_path: 模型权重路径
                - min_confidence: 最小置信度
                - max_candidates: 最大候选数
            worker: AsyncWorker 实例
        """
        self._config = config
        self._worker = worker
        self._model = None
        self._ready = False
    
    async def initialize(self) -> bool:
        """异步初始化，加载模型"""
        return await self._worker.run_callable(self._load_model)
    
    def _load_model(self) -> bool:
        """加载 Contact-GraspNet 模型（在工作线程中执行）"""
        # 加载 PyTorch 模型
        # 编译 pointnet2_ops
        # Warmup
        pass
    
    async def predict(
        self,
        point_cloud: np.ndarray,
        mask: Optional[np.ndarray] = None,
        max_candidates: int = 50,
        min_confidence: float = 0.5
    ) -> List[GraspCandidate]:
        """
        执行抓取推理
        
        Args:
            point_cloud: (N, 3) 点云坐标
            mask: 可选的分割掩码
            max_candidates: 最大候选数
            min_confidence: 最小置信度
            
        Returns:
            抓取候选列表，按置信度降序
        """
        return await self._worker.run_callable(
            self._predict_sync,
            point_cloud, mask, max_candidates, min_confidence
        )
```

### 3.6 Service 回调实现

```python
# 在 PerceptionNode 中注册 Service

def __init__(self):
    # ...
    self._grasp_srv = self.create_service(
        Grasp,
        '/perception/service/grasp',
        self._grasp_callback
    )

async def _grasp_callback(
    self,
    request: Grasp.Request,
    response: Grasp.Response
) -> Grasp.Response:
    """
    Grasp Service 回调
    """
    try:
        # 解析点云
        point_cloud = self._parse_pointcloud(request.point_cloud)
        
        # 解析可选参数
        mask = self._parse_mask(request.mask) if request.mask else None
        max_candidates = request.max_candidates or 50
        min_confidence = request.min_confidence or 0.5
        
        # 执行推理
        candidates = await self._grasp_module.predict(
            point_cloud, mask, max_candidates, min_confidence
        )
        
        # 构造响应
        response.success = True
        response.candidates = [self._to_msg(c) for c in candidates]
        
    except Exception as e:
        response.success = False
        response.error_message = str(e)
        response.candidates = []
    
    return response
```

---

## 4. 后续落地工作

### 4.1 开发任务

| # | 任务 | 说明 | 预计工时 |
|---|------|------|---------|
| 1 | 定义消息类型 | Grasp.srv, GraspCandidate.msg | 0.5 天 |
| 2 | 编译 pointnet2_ops | aarch64 CUDA 扩展 | 0.5 天 |
| 3 | 获取模型 | PyTorch 版 + 预训练权重 | 0.5 天 |
| 4 | 实现 GraspModule | 参考 SegmentationModule | 2 天 |
| 5 | 注册 Service | 集成到 PerceptionNode | 0.5 天 |
| 6 | 测试验证 | 单元测试 + 集成测试 | 1 天 |

### 4.2 依赖项

| 依赖 | 版本 | 说明 |
|------|------|------|
| PyTorch | 与现有 NanoSAM 统一 | 已有 |
| pointnet2_ops | 需编译 | CUDA 扩展 |
| Contact-GraspNet | PyTorch 版 | 社区移植 |
| 预训练权重 | scene_test_2048_bs3_hor_sigma_001 | 官方提供 |

### 4.3 配置项（perception_config.yaml）

```yaml
grasp:
  enabled: true
  model_path: "models/contact_graspnet.pth"
  min_confidence: 0.5
  max_candidates: 50
  use_fp16: false  # 后续可开启
  timeout: 5.0     # 推理超时（秒）
```

---

## 5. 参考资料

- [Contact-GraspNet 论文](https://arxiv.org/abs/2103.14127)
- [Contact-GraspNet GitHub (NVlabs)](https://github.com/NVlabs/Contact-GraspNet)
- [PointNet++ PyTorch](https://github.com/erikwijmans/Pointnet2_PyTorch)
- [RealSense D435I 规格](https://www.intelrealsense.com/depth-camera-d435i/)

---

## 6. 类关系结构图

### 6.1 模块层次结构

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         PerceptionNode                                   │
│                    (src/perception_core/node.py)                         │
│                                                                          │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐    │
│  │ Segmentation│  │ PointCloud  │  │   Grasp     │  │  Vectorizer │    │
│  │   Module    │  │   Module    │  │   Module    │  │   Module    │    │
│  │  (NanoSAM)  │  │  (Open3D)   │  │(ContactGN)  │  │ (CLIP/DINO) │    │
│  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘    │
│         │                │                │                │            │
│         └────────────────┴────────────────┴────────────────┘            │
│                                   │                                      │
│                                   ▼                                      │
│                          ┌───────────────┐                              │
│                          │  AsyncWorker  │                              │
│                          │ (线程池执行)   │                              │
│                          └───────────────┘                              │
└─────────────────────────────────────────────────────────────────────────┘
```

### 6.2 GraspModule 内部结构

```
┌─────────────────────────────────────────────────────────────────────────┐
│                           GraspModule                                    │
│                  (src/perception_core/grasp_module.py)                   │
├─────────────────────────────────────────────────────────────────────────┤
│  属性:                                                                   │
│    _config: Dict[str, Any]          # 配置字典                          │
│    _worker: AsyncWorker             # 异步工作线程                       │
│    _model: ContactGraspNet          # PyTorch 模型                       │
│    _ready: bool                     # 初始化状态                         │
│    _device: torch.device            # GPU 设备                          │
├─────────────────────────────────────────────────────────────────────────┤
│  公开方法:                                                               │
│    initialize() -> bool             # 异步初始化                         │
│    predict(point_cloud, ...) -> List[GraspCandidate]  # 推理入口        │
│    is_ready: bool                   # 就绪状态属性                       │
├─────────────────────────────────────────────────────────────────────────┤
│  内部方法:                                                               │
│    _load_model() -> bool            # 加载模型（同步）                   │
│    _warmup() -> None                # GPU 预热                          │
│    _predict_sync(...) -> List       # 同步推理（在线程池执行）           │
│    _preprocess(pc) -> Tensor        # 点云预处理                         │
│    _postprocess(output) -> List     # 后处理（NMS、排序）                │
└─────────────────────────────────────────────────────────────────────────┘
```

### 6.3 Contact-GraspNet 网络结构

```
┌─────────────────────────────────────────────────────────────────────────┐
│                       ContactGraspNet (PyTorch)                          │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  输入: point_cloud (N, 3)                                                │
│         │                                                                │
│         ▼                                                                │
│  ┌─────────────────────────────────────────────────────────┐            │
│  │              PointNet++ Encoder                          │            │
│  │  ┌─────────┐  ┌─────────┐  ┌─────────┐  ┌─────────┐    │            │
│  │  │  SA1    │→ │  SA2    │→ │  SA3    │→ │  SA4    │    │            │
│  │  │ 2048 pt │  │ 512 pt  │  │ 128 pt  │  │  32 pt  │    │            │
│  │  └─────────┘  └─────────┘  └─────────┘  └─────────┘    │            │
│  │                                                          │            │
│  │  SA = Set Abstraction (FPS + BallQuery + MLP)           │            │
│  └─────────────────────────────────────────────────────────┘            │
│         │                                                                │
│         ▼                                                                │
│  ┌─────────────────────────────────────────────────────────┐            │
│  │              PointNet++ Decoder                          │            │
│  │  ┌─────────┐  ┌─────────┐  ┌─────────┐  ┌─────────┐    │            │
│  │  │  FP4    │→ │  FP3    │→ │  FP2    │→ │  FP1    │    │            │
│  │  │  32→128 │  │ 128→512 │  │512→2048 │  │2048→N   │    │            │
│  │  └─────────┘  └─────────┘  └─────────┘  └─────────┘    │            │
│  │                                                          │            │
│  │  FP = Feature Propagation (ThreeNN + Interpolate)       │            │
│  └─────────────────────────────────────────────────────────┘            │
│         │                                                                │
│         ▼                                                                │
│  ┌─────────────────────────────────────────────────────────┐            │
│  │                   Grasp Prediction Head                   │            │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐      │            │
│  │  │ Contact     │  │ Grasp Pose  │  │ Grasp Width │      │            │
│  │  │ Prediction  │  │ Prediction  │  │ Prediction  │      │            │
│  │  │ (per-point) │  │ (6-DoF)     │  │ (scalar)    │      │            │
│  │  └─────────────┘  └─────────────┘  └─────────────┘      │            │
│  └─────────────────────────────────────────────────────────┘            │
│         │                                                                │
│         ▼                                                                │
│  输出: grasp_poses (K, 4, 4), scores (K,), widths (K,)                  │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

### 6.4 类依赖关系

```
┌─────────────────┐      ┌─────────────────┐
│ PerceptionNode  │─────►│   GraspModule   │
└─────────────────┘      └────────┬────────┘
                                  │
                    ┌─────────────┼─────────────┐
                    │             │             │
                    ▼             ▼             ▼
          ┌─────────────┐ ┌─────────────┐ ┌─────────────┐
          │ AsyncWorker │ │ContactGrasp │ │  pointnet2  │
          │             │ │    Net      │ │    _ops     │
          └─────────────┘ └──────┬──────┘ └─────────────┘
                                 │
                    ┌────────────┼────────────┐
                    │            │            │
                    ▼            ▼            ▼
              ┌──────────┐ ┌──────────┐ ┌──────────┐
              │PointNet++│ │PointNet++│ │  Grasp   │
              │ Encoder  │ │ Decoder  │ │   Head   │
              └──────────┘ └──────────┘ └──────────┘

依赖说明:
├── PerceptionNode: ROS2 节点，管理所有模块
├── GraspModule: 抓取推理模块，封装 Contact-GraspNet
├── AsyncWorker: 异步执行器，GPU 推理在独立线程
├── ContactGraspNet: PyTorch 网络定义
├── pointnet2_ops: PointNet++ CUDA 算子
└── PointNet++ Encoder/Decoder: 点云特征提取
```

### 6.5 消息与服务关系

```
┌─────────────────────────────────────────────────────────────────────────┐
│                          ROS2 接口层                                     │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  ┌─────────────────────────────────────────────────────────────────┐    │
│  │                    /perception/service/grasp                     │    │
│  │                         (Grasp.srv)                              │    │
│  ├─────────────────────────────────────────────────────────────────┤    │
│  │  Request:                        Response:                       │    │
│  │  ├─ PointCloud2 point_cloud     ├─ bool success                 │    │
│  │  ├─ Image mask (可选)            ├─ string error_message         │    │
│  │  ├─ int32 max_candidates        └─ GraspCandidate[] candidates  │    │
│  │  └─ float32 min_confidence                                      │    │
│  └─────────────────────────────────────────────────────────────────┘    │
│                                   │                                      │
│                                   ▼                                      │
│  ┌─────────────────────────────────────────────────────────────────┐    │
│  │                     GraspCandidate.msg                           │    │
│  ├─────────────────────────────────────────────────────────────────┤    │
│  │  geometry_msgs/Point position        # 夹爪中心 (x, y, z)        │    │
│  │  geometry_msgs/Quaternion orientation # 姿态四元数               │    │
│  │  float32 width                       # 建议开口宽度              │    │
│  │  float32 confidence                  # 置信度                    │    │
│  │  geometry_msgs/Point[] contact_points # 接触点（可选）           │    │
│  └─────────────────────────────────────────────────────────────────┘    │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## 7. 落地执行步骤清单

### Phase 1: 环境准备

| Step | 任务 | 具体操作 | 验收标准 | 预计耗时 |
|------|------|---------|---------|---------|
| 1.1 | 确认 PyTorch 环境 | 检查现有 NanoSAM 使用的 PyTorch 版本 | `python -c "import torch; print(torch.__version__)"` 成功 | 5 min |
| 1.2 | 确认 CUDA 环境 | 验证 CUDA 可用 | `python -c "import torch; print(torch.cuda.is_available())"` 返回 True | 5 min |
| 1.3 | 克隆 Contact-GraspNet | `git clone https://github.com/NVlabs/Contact-GraspNet.git` | 仓库下载完成 | 10 min |
| 1.4 | 克隆 pointnet2_ops | `git clone https://github.com/erikwijmans/Pointnet2_PyTorch.git` | 仓库下载完成 | 5 min |

### Phase 2: 编译 PointNet++ CUDA 扩展

| Step | 任务 | 具体操作 | 验收标准 | 预计耗时 |
|------|------|---------|---------|---------|
| 2.1 | 安装编译依赖 | `pip3 install ninja` | ninja 可用 | 5 min |
| 2.2 | 编译 pointnet2_ops | `cd pointnet2_ops_lib && pip3 install .` | 无编译错误 | 15 min |
| 2.3 | 验证编译结果 | `python -c "from pointnet2_ops import pointnet2_utils"` | 导入成功，无报错 | 5 min |
| 2.4 | 测试 CUDA 算子 | 运行简单的 ball_query 测试 | 返回正确结果 | 10 min |

### Phase 3: 模型准备

| Step | 任务 | 具体操作 | 验收标准 | 预计耗时 |
|------|------|---------|---------|---------|
| 3.1 | 下载官方权重 | 从 Contact-GraspNet releases 下载 | `scene_test_2048_bs3_hor_sigma_001` 文件存在 | 30 min |
| 3.2 | 获取 PyTorch 版代码 | 查找社区 PyTorch 移植版或自行转换 | PyTorch 模型定义文件就绪 | 2 hr |
| 3.3 | 转换权重格式 | TensorFlow checkpoint → PyTorch state_dict | `.pth` 文件生成 | 1 hr |
| 3.4 | 验证模型加载 | 加载模型并打印结构 | 模型加载无报错 | 15 min |
| 3.5 | 拷贝到 models 目录 | `cp contact_graspnet.pth src/perception/models/` | 文件就位 | 5 min |

### Phase 4: 消息定义

| Step | 任务 | 具体操作 | 验收标准 | 预计耗时 |
|------|------|---------|---------|---------|
| 4.1 | 创建 GraspCandidate.msg | 在 `src/perception/msg/` 创建消息定义 | 文件创建完成 | 15 min |
| 4.2 | 创建 Grasp.srv | 在 `src/perception/srv/` 创建服务定义 | 文件创建完成 | 15 min |
| 4.3 | 更新 CMakeLists.txt | 添加消息和服务生成 | 编译配置更新 | 10 min |
| 4.4 | 编译消息 | `colcon build --packages-select perception` | 消息生成成功 | 5 min |
| 4.5 | 验证消息导入 | `from perception.msg import GraspCandidate` | 导入成功 | 5 min |

### Phase 5: GraspModule 实现

| Step | 任务 | 具体操作 | 验收标准 | 预计耗时 |
|------|------|---------|---------|---------|
| 5.1 | 创建 grasp_module.py | 在 `src/perception_core/` 创建模块文件 | 文件创建完成 | 15 min |
| 5.2 | 实现 `__init__` | 配置解析、成员变量初始化、**创建线程锁** | 类可实例化 | 30 min |
| 5.3 | 实现 `_load_model` | 加载 PyTorch 模型、**设置 CUDA 设备**、pointnet2_ops | 模型加载成功 | 1 hr |
| 5.4 | 实现 warmup | 虚拟输入预热 GPU、**创建 CUDA Stream** | warmup 完成无报错 | 30 min |
| 5.5 | 实现 `_predict_sync` | **加锁保护**、点云预处理 → 推理 → 后处理 | 返回候选列表 | 2 hr |
| 5.6 | 实现 `predict` | 异步包装，使用 **共享的 AsyncWorker** | 异步调用成功 | 30 min |
| 5.7 | 添加错误处理 | 异常捕获、错误码定义 | 错误情况有明确返回 | 30 min |

#### ⚠️ Phase 5 关键注意事项：线程与 CUDA

**问题背景**：PyTorch CUDA context 是线程绑定的，模型加载和推理必须在同一线程执行。

**必须遵循的模式**（参考 SegmentationModule）：

```python
class GraspModule:
    def __init__(self, config: Dict[str, Any], worker: AsyncWorker):
        self._config = config
        self._worker = worker           # 复用现有 AsyncWorker（与 SegmentationModule 共享）
        self._model = None
        self._ready = False
        self._device = None
        self._stream = None
        self._model_lock = threading.Lock()  # ⚠️ 必须：保护模型访问
    
    def _load_model_sync(self) -> bool:
        """在 AsyncWorker 线程池中执行，确保 CUDA context 一致"""
        # ⚠️ 必须：显式设置 CUDA 设备
        self._device = torch.device("cuda:0")
        torch.cuda.set_device(self._device)
        
        # ⚠️ 必须：创建 CUDA Stream（与 SegmentationModule 模式一致）
        self._stream = torch.cuda.Stream(device=self._device)
        
        # 加载模型
        self._model = ContactGraspNet().to(self._device)
        self._model.load_state_dict(torch.load(...))
        self._model.eval()
        
        return True
    
    def _predict_sync(self, point_cloud, ...):
        """同步推理，在 AsyncWorker 线程中执行"""
        # ⚠️ 必须：加锁保护
        with self._model_lock:
            with torch.cuda.stream(self._stream):
                # 推理逻辑
                ...
            torch.cuda.current_stream().wait_stream(self._stream)
        
        return candidates
    
    async def predict(self, point_cloud, ...):
        """异步入口，通过 AsyncWorker 调度"""
        # ⚠️ 必须：使用 run_callable，不要直接调用同步方法
        return await self._worker.run_callable(
            self._predict_sync,
            point_cloud, ...
        )
```

**关键检查点**：

| 检查项 | 正确做法 | 错误做法 |
|-------|---------|---------|
| AsyncWorker | 复用 PerceptionNode 的实例 | 每个模块创建独立 Worker |
| CUDA 设备 | 显式 `torch.cuda.set_device()` | 依赖默认设备 |
| 线程锁 | `threading.Lock()` 保护模型 | 无锁直接访问 |
| CUDA Stream | 使用独立 Stream | 使用默认 Stream |
| 初始化 | 通过 `run_callable` 执行 | 在主线程直接加载 |

### Phase 6: Service 集成

| Step | 任务 | 具体操作 | 验收标准 | 预计耗时 |
|------|------|---------|---------|---------|
| 6.1 | 更新 constants.py | 添加 grasp 相关常量 | 常量定义完成 | 15 min |
| 6.2 | 更新 error_codes.py | 添加 grasp 相关错误码 | 错误码定义完成 | 15 min |
| 6.3 | 更新 node.py | 初始化 GraspModule、**共享 AsyncWorker** | 模块初始化成功 | 30 min |
| 6.4 | 注册 Service | 创建 `/perception/service/grasp`、**使用 callback_group** | Service 注册成功 | 30 min |
| 6.5 | 实现回调函数 | `_grasp_callback` 实现、**处理同步/异步转换** | 回调逻辑完成 | 1 hr |
| 6.6 | 更新配置文件 | `perception_config.yaml` 添加 grasp 配置 | 配置项就绪 | 15 min |

#### ⚠️ Phase 6 关键注意事项：Service 回调模式

**问题背景**：ROS2 Service 默认是同步回调，但 GraspModule.predict() 是异步的。

**正确的集成模式**：

```python
# node.py 中的集成

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        
        # ⚠️ 关键：创建单一 AsyncWorker，所有模块共享
        self._async_worker = AsyncWorker(name="perception_worker", max_workers=1)
        self._async_worker.start()
        
        # 初始化模块时传入同一个 worker
        self._segment_module = SegmentationModule(config, self._async_worker)
        self._grasp_module = GraspModule(config, self._async_worker)  # 共享同一个
        
        # ⚠️ 关键：使用 MutuallyExclusiveCallbackGroup 避免回调冲突
        self._service_callback_group = MutuallyExclusiveCallbackGroup()
        
        # 注册 Service
        self._grasp_srv = self.create_service(
            Grasp,
            '/perception/service/grasp',
            self._grasp_callback,
            callback_group=self._service_callback_group  # ⚠️ 必须指定
        )
    
    def _grasp_callback(
        self,
        request: Grasp.Request,
        response: Grasp.Response
    ) -> Grasp.Response:
        """
        Service 回调 - 同步版本
        
        ⚠️ 注意：ROS2 Service 回调默认是同步的
        需要在回调内部等待异步结果
        """
        try:
            # 解析请求
            point_cloud = self._parse_pointcloud(request.point_cloud)
            mask = self._parse_mask(request.mask) if request.mask.data else None
            max_candidates = request.max_candidates if request.max_candidates > 0 else 50
            min_confidence = request.min_confidence if request.min_confidence > 0 else 0.5
            
            # ⚠️ 关键：同步等待异步结果
            # 方式1：使用 AsyncWorker 的 awaitable（推荐）
            awaitable = self._grasp_module.predict(
                point_cloud, mask, max_candidates, min_confidence
            )
            # 直接获取结果（会阻塞当前回调线程）
            candidates = self._wait_for_result(awaitable, timeout=5.0)
            
            # 构造响应
            response.success = True
            response.candidates = [self._candidate_to_msg(c) for c in candidates]
            
        except Exception as e:
            self.get_logger().error(f"Grasp service error: {e}")
            response.success = False
            response.error_message = str(e)
            response.candidates = []
        
        return response
    
    def _wait_for_result(self, awaitable, timeout: float):
        """
        同步等待 AsyncWorker 的结果
        
        ⚠️ 这是在同步回调中获取异步结果的桥接方法
        """
        import time
        start = time.time()
        while True:
            try:
                # 尝试获取结果
                return awaitable._future.result(timeout=0.01)
            except TimeoutError:
                if time.time() - start > timeout:
                    raise TimeoutError(f"Grasp prediction timeout after {timeout}s")
                continue
```

**关键检查点**：

| 检查项 | 正确做法 | 错误做法 |
|-------|---------|---------|
| AsyncWorker | 所有模块共享一个 | 每个模块独立创建 |
| max_workers | 设为 1（保证线程一致） | 默认多线程 |
| callback_group | 使用 MutuallyExclusiveCallbackGroup | 使用默认组 |
| 异步等待 | 同步回调内阻塞等待 | 直接 await（会报错） |
| 超时处理 | 设置合理超时 | 无限等待 |

### Phase 7: 测试验证

| Step | 任务 | 具体操作 | 验收标准 | 预计耗时 |
|------|------|---------|---------|---------|
| 7.1 | 单元测试：模型加载 | 测试 GraspModule 初始化 | 测试通过 | 30 min |
| 7.2 | 单元测试：推理 | 使用模拟点云测试推理 | 返回有效候选 | 1 hr |
| 7.3 | 集成测试：Service | ros2 service call 测试 | 响应正确 | 30 min |
| 7.4 | 端到端测试 | segment → pointcloud → grasp 完整流程 | 流程跑通 | 1 hr |
| 7.5 | 性能测试 | 测量推理延迟 | 延迟 < 300ms | 30 min |
| 7.6 | 真实场景测试 | 使用真实相机和物体测试 | 抓取位姿合理 | 2 hr |

### Phase 8: 文档与收尾

| Step | 任务 | 具体操作 | 验收标准 | 预计耗时 |
|------|------|---------|---------|---------|
| 8.1 | 更新 perception.md | 添加 grasp service 文档 | 文档更新完成 | 30 min |
| 8.2 | 添加使用示例 | 代码示例和调用说明 | 示例可运行 | 30 min |
| 8.3 | 代码审查 | 检查代码风格、注释 | 符合项目规范 | 1 hr |
| 8.4 | 提交代码 | git commit & push | 代码入库 | 15 min |

---

### 执行总结

| Phase | 内容 | 预计耗时 |
|-------|------|---------|
| Phase 1 | 环境准备 | 0.5 hr |
| Phase 2 | 编译 PointNet++ | 0.5 hr |
| Phase 3 | 模型准备 | 4 hr |
| Phase 4 | 消息定义 | 1 hr |
| Phase 5 | GraspModule 实现 | 5.5 hr |
| Phase 6 | Service 集成 | 3 hr |
| Phase 7 | 测试验证 | 6 hr |
| Phase 8 | 文档与收尾 | 2.5 hr |
| **合计** | | **~23 hr（约 3 天）** |

---

### 风险点与应对

| 风险 | 可能性 | 影响 | 应对措施 |
|------|-------|------|---------|
| pointnet2_ops 编译失败 | 中 | 阻塞 | 检查 CUDA 版本匹配，参考 issue |
| PyTorch 版本不兼容 | 低 | 阻塞 | 使用与 NanoSAM 相同的环境 |
| 权重转换失败 | 中 | 延期 | 寻找已有 PyTorch 移植版 |
| 推理延迟过高 | 低 | 需优化 | 开启 FP16，减少点云采样 |
| 抓取精度不足 | 中 | 需调优 | 调整置信度阈值，微调模型 |
| **CUDA context 线程错误** | **高** | **崩溃** | **严格遵循 Phase 5 的线程模式** |
| **并发调用冲突** | **中** | **结果错误** | **使用 threading.Lock 保护模型** |
| **Service 回调死锁** | **中** | **无响应** | **使用 MutuallyExclusiveCallbackGroup** |
| **AsyncWorker 未启动** | **低** | **阻塞** | **在 node init 时显式 start()** |

---

### ⚠️ 线程问题速查表

如果遇到以下错误，检查对应问题：

| 错误现象 | 可能原因 | 解决方法 |
|---------|---------|---------|
| `CUDA error: invalid device ordinal` | 未设置 CUDA 设备 | 添加 `torch.cuda.set_device(0)` |
| `RuntimeError: CUDA error: initialization error` | 线程 context 不一致 | 确保加载和推理在同一线程 |
| 推理结果随机错误 | 无锁并发访问 | 添加 `threading.Lock` |
| Service 调用无响应 | 回调死锁 | 检查 callback_group 配置 |
| `Future is not done` | await 使用错误 | 在同步回调中不能直接 await |
| 内存持续增长 | CUDA tensor 未释放 | 使用 `with torch.no_grad()` |
