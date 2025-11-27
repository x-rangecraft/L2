# Perception 视觉分析节点规划

## 概述

本节点是一个**纯服务节点**，提供以下能力：
- **目标分割**：基于点击坐标分割目标物体（NanoSAM）
- **点云计算**：计算目标的 3D 位置和边界框
- **物体记录**：向量化存储物体特征（CLIP + DINOv3 + FAISS）
- **物体检索**：支持文本、图片、标签多种查询方式

**特点**：
- 不订阅任何话题，完全按需处理
- 所有输入通过 Action/Service 传入
- 空闲时无资源占用（除模型内存）

---

## 架构方案 ✅

**选择：单一服务节点**

```
┌─────────────────────────────────────────────────────────────┐
│                  Perception Node（服务节点）                 │
│                                                             │
│  ┌─────────────────────────────────────────────────────┐   │
│  │                   核心模块                           │   │
│  │  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌────────┐  │   │
│  │  │ NanoSAM  │ │PointCloud│ │   CLIP   │ │ DINOv3 │  │   │
│  │  │  分割    │ │   计算   │ │  向量化  │ │ 向量化 │  │   │
│  │  └──────────┘ └──────────┘ └──────────┘ └────────┘  │   │
│  │                                                      │   │
│  │  ┌──────────┐ ┌──────────┐                          │   │
│  │  │  FAISS   │ │ Storage  │                          │   │
│  │  │ 向量索引 │ │ 文件存储 │                          │   │
│  │  └──────────┘ └──────────┘                          │   │
│  └─────────────────────────────────────────────────────┘   │
│                           │                                 │
│                           ▼                                 │
│  ┌─────────────────────────────────────────────────────┐   │
│  │              对外接口（Action + Service）            │   │
│  │                                                      │   │
│  │  Action:                                             │   │
│  │    segment, pointcloud, vectorize                    │   │
│  │    object_target, object_record, object_process      │   │
│  │                                                      │   │
│  │  Service:                                            │   │
│  │    save_object, add_sample, update_object, delete_*  │   │
│  │    query_by_desc, query_by_image, query_by_label, ...│   │
│  └─────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
```

**优点**：
- 完全按需，不占用持续资源
- 接口清晰，易于测试
- 可独立部署

---

## 数据流设计

### 订阅话题
**无**（纯服务节点，不订阅任何话题）

### 交互方式

```
┌─────────────────┐          ┌─────────────────────────────────┐
│    调用方       │          │      Perception Node            │
│  (上层节点)     │          │                                 │
└────────┬────────┘          └────────────────┬────────────────┘
         │                                    │
         │  ══════════ Action 调用 ══════════ │
         │                                    │
         │  object_target(图像+深度+点击)     │
         │ ─────────────────────────────────► │
         │                                    │  segment()
         │            Feedback: "segmenting"  │  pointcloud()
         │ ◄───────────────────────────────── │
         │                                    │
         │            Result: 点云+位置       │
         │ ◄───────────────────────────────── │
         │                                    │
         │  object_record(裁剪图+标签)        │
         │ ─────────────────────────────────► │
         │                                    │  vectorize()
         │            Feedback: "vectorizing" │  save_object()
         │ ◄───────────────────────────────── │
         │                                    │
         │            Result: object_id       │
         │ ◄───────────────────────────────── │
         │                                    │
         │  ══════════ Service 调用 ═════════ │
         │                                    │
         │  query_by_desc("红色杯子")         │
         │ ─────────────────────────────────► │
         │                                    │  CLIP检索
         │            Response: 物体列表      │
         │ ◄───────────────────────────────── │
         │                                    │
```

### 数据传输
- **图像数据**：通过 Action Goal 传入，使用 DDS 共享内存（零拷贝）
- **查询请求**：通过 Service Request 传入
- **所有结果**：通过 Action Result / Service Response 返回

---

## 交互模式 ✅

**选择：Action + Service**

| 类型 | 适用场景 | 特点 |
|------|----------|------|
| **Action** | 需要 GPU 计算（分割/点云/向量化） | 异步、可取消、有进度反馈 |
| **Service** | 纯数据操作（增删改查） | 同步、快速响应 |

**节点特点**：
- 不订阅话题，完全按需处理
- 图像通过 Action Goal 传入（共享内存零拷贝）
- 空闲时仅占用模型内存，无 CPU 占用

---

## Action 定义

### 原子 Action

#### 1. Segment（目标分割）

**功能**：基于点击坐标，使用 NanoSAM 分割目标物体

```
# action/Segment.action
# Action: /perception/action/segment

#==================== Goal ====================
sensor_msgs/Image color_image         # [必填] RGB 图像（共享内存传输）
float64 click_x                       # [必填] 点击坐标 X（像素坐标）
float64 click_y                       # [必填] 点击坐标 Y（像素坐标）

---

#==================== Result ====================
bool success                          # 是否成功
string error_message                  # 错误信息（失败时）

sensor_msgs/Image mask                # 分割掩码（mono8，255=目标，0=背景）
sensor_msgs/Image cropped_image       # 裁剪后的目标图像（RGB）
sensor_msgs/Image visualization       # 可视化叠加图（原图+掩码+点击点）
float32 confidence                    # 分割置信度（0.0-1.0）
int32 mask_area_pixels                # 掩码面积（像素数）

---

#==================== Feedback ====================
string status                         # 当前状态："encoding" / "decoding"
float32 progress                      # 进度（0.0-1.0）
```

#### 2. PointCloud（点云计算）

**功能**：根据分割掩码和深度图，计算目标物体的 3D 点云

```
# action/PointCloud.action
# Action: /perception/action/pointcloud

#==================== Goal ====================
sensor_msgs/Image mask                # [必填] 分割掩码（来自 segment）
sensor_msgs/Image depth_image         # [必填] 深度图（对齐到 RGB，共享内存传输）
sensor_msgs/CameraInfo camera_info    # [必填] 相机内参（fx, fy, cx, cy）

---

#==================== Result ====================
bool success                          # 是否成功
string error_message                  # 错误信息（失败时）

sensor_msgs/PointCloud2 point_cloud   # 目标区域点云（XYZ）
geometry_msgs/Point center_3d         # 点云质心坐标（相机坐标系）
geometry_msgs/Vector3 bbox_min        # 3D 边界框最小点 [x_min, y_min, z_min]
geometry_msgs/Vector3 bbox_max        # 3D 边界框最大点 [x_max, y_max, z_max]
int32 point_count                     # 有效点数量

---

#==================== Feedback ====================
string status                         # 当前状态："computing"
float32 progress                      # 进度（0.0-1.0）
```

#### 3. Vectorize（向量化）

**功能**：使用 CLIP 和 DINOv3 提取图像特征向量

```
# action/Vectorize.action
# Action: /perception/action/vectorize

#==================== Goal ====================
sensor_msgs/Image cropped_image       # [必填] 裁剪后的目标图像（来自 segment）

---

#==================== Result ====================
bool success                          # 是否成功
string error_message                  # 错误信息（失败时）

float32[] clip_embedding              # CLIP 特征向量（512 维，用于文本检索）
float32[] dino_embedding              # DINOv3 特征向量（384 维，用于图像检索）

---

#==================== Feedback ====================
string status                         # 当前状态："clip_encoding" / "dino_encoding"
float32 progress                      # 进度（0.0-1.0）
```

---

### 组合 Action

#### 4. ObjectTarget（抓取定位）

**功能**：分割 → 点云计算（用于抓取场景，不保存）

```
# action/ObjectTarget.action
# Action: /perception/action/object_target

#==================== Goal ====================
sensor_msgs/Image color_image         # [必填] RGB 图像
sensor_msgs/Image depth_image         # [必填] 深度图（对齐到 RGB）
sensor_msgs/CameraInfo camera_info    # [必填] 相机内参
float64 click_x                       # [必填] 点击坐标 X（像素）
float64 click_y                       # [必填] 点击坐标 Y（像素）

---

#==================== Result ====================
bool success                          # 是否成功
string error_message                  # 错误信息（失败时）

# 分割结果
sensor_msgs/Image mask                # 分割掩码
sensor_msgs/Image cropped_image       # 裁剪后的目标图像
sensor_msgs/Image visualization       # 可视化叠加图
float32 confidence                    # 分割置信度
int32 mask_area_pixels                # 掩码面积

# 点云结果（用于抓取）
sensor_msgs/PointCloud2 point_cloud   # 目标点云
geometry_msgs/Point center_3d         # 点云质心（抓取目标位置）
geometry_msgs/Vector3 bbox_min        # 3D 边界框最小点
geometry_msgs/Vector3 bbox_max        # 3D 边界框最大点

---

#==================== Feedback ====================
string status                         # "segmenting" / "computing_pointcloud"
float32 progress                      # 进度（0.0-1.0）
```

#### 5. ObjectRecord（记录物体）

**功能**：向量化 → 保存（记录已分割的物体）

```
# action/ObjectRecord.action
# Action: /perception/action/object_record

#==================== Goal ====================
sensor_msgs/Image cropped_image       # [必填] 裁剪后的目标图像（来自 object_target）
string label                          # [可选] 语义标签（如 "cup"）
string description                    # [可选] 文本描述（如 "红色陶瓷杯子"）

---

#==================== Result ====================
bool success                          # 是否成功
string error_message                  # 错误信息（失败时）
string object_id                      # 保存后的物体 ID（如 "obj_001"）
int64 created_at                      # 创建时间（毫秒时间戳）

# 向量化结果
float32[] clip_embedding              # CLIP 向量（512d）
float32[] dino_embedding              # DINOv3 向量（384d）

---

#==================== Feedback ====================
string status                         # "vectorizing" / "saving"
float32 progress                      # 进度（0.0-1.0）
```

#### 6. ObjectProcess（完整流程）

**功能**：分割 → 点云计算 → 向量化 → 保存（一步到位）

**等价于**：`object_target` + `object_record`

```
# action/ObjectProcess.action
# Action: /perception/action/object_process

#==================== Goal ====================
sensor_msgs/Image color_image         # [必填] RGB 图像
sensor_msgs/Image depth_image         # [必填] 深度图（对齐到 RGB）
sensor_msgs/CameraInfo camera_info    # [必填] 相机内参
float64 click_x                       # [必填] 点击坐标 X（像素）
float64 click_y                       # [必填] 点击坐标 Y（像素）
string label                          # [可选] 语义标签（如 "cup"）
string description                    # [可选] 文本描述（如 "红色陶瓷杯子"）

---

#==================== Result ====================
bool success                          # 是否成功
string error_message                  # 错误信息（失败时）
string object_id                      # 保存后的物体 ID（如 "obj_001"）
int64 created_at                      # 创建时间（毫秒时间戳）

# 分割结果
sensor_msgs/Image mask                # 分割掩码
sensor_msgs/Image cropped_image       # 裁剪后的目标图像
sensor_msgs/Image visualization       # 可视化叠加图
float32 confidence                    # 分割置信度
int32 mask_area_pixels                # 掩码面积

# 点云结果
sensor_msgs/PointCloud2 point_cloud   # 目标点云
geometry_msgs/Point center_3d         # 点云质心（抓取参考位置）
geometry_msgs/Vector3 bbox_min        # 3D 边界框最小点
geometry_msgs/Vector3 bbox_max        # 3D 边界框最大点

# 向量化结果
float32[] clip_embedding              # CLIP 向量（512d）
float32[] dino_embedding              # DINOv3 向量（384d）

---

#==================== Feedback ====================
string status                         # "segmenting" / "computing_pointcloud" / "vectorizing" / "saving"
float32 progress                      # 进度（0.0-1.0）
```

---

## Service 定义

### 存储服务

#### 1. SaveObject（保存物体）

**功能**：将物体信息保存到索引（向量 + 元数据）

```
# srv/SaveObject.srv
# Service: /perception/service/save_object

#==================== Request ====================
sensor_msgs/Image cropped_image       # [必填] 物体图像（已裁剪）
float32[] clip_embedding              # [必填] CLIP 向量（512d）
float32[] dino_embedding              # [必填] DINOv3 向量（384d）
string label                          # [可选] 标签（如 "cup"）
string description                    # [可选] 描述（如 "红色陶瓷杯子"）

---

#==================== Response ====================
bool success                          # 是否成功
string error_message                  # 错误信息（失败时）
string object_id                      # 自动生成的物体 ID
int64 created_at                      # 创建时间（毫秒时间戳）
```

#### 2. AddSample（增加样本）

**功能**：给已有物体增加新的向量样本（多角度增强）

```
# srv/AddSample.srv
# Service: /perception/service/add_sample

#==================== Request ====================
string object_id                      # [必填] 目标物体 ID
float32[] clip_embedding              # [必填] 新的 CLIP 向量
float32[] dino_embedding              # [必填] 新的 DINOv3 向量

---

#==================== Response ====================
bool success                          # 是否成功
string error_message                  # 错误信息（失败时）
string sample_id                      # 新样本 ID
int32 total_samples                   # 该物体当前样本总数
```

#### 3. UpdateObject（更新物体）

**功能**：更新物体的标签或描述

```
# srv/UpdateObject.srv
# Service: /perception/service/update_object

#==================== Request ====================
string object_id                      # [必填] 物体 ID
string label                          # [可选] 新标签（空字符串表示不更新）
string description                    # [可选] 新描述（空字符串表示不更新）

---

#==================== Response ====================
bool success                          # 是否成功
string error_message                  # 错误信息（失败时）
int64 updated_at                      # 更新时间（毫秒时间戳）
```

#### 4. DeleteObject（删除物体）

**功能**：删除物体及其所有样本

```
# srv/DeleteObject.srv
# Service: /perception/service/delete_object

#==================== Request ====================
string object_id                      # [必填] 要删除的物体 ID

---

#==================== Response ====================
bool success                          # 是否成功
string error_message                  # 错误信息（失败时）
int32 deleted_samples                 # 删除的样本数量
```

#### 5. DeleteSample（删除样本）

**功能**：删除物体的指定样本

```
# srv/DeleteSample.srv
# Service: /perception/service/delete_sample

#==================== Request ====================
string object_id                      # [必填] 物体 ID
string sample_id                      # [必填] 要删除的样本 ID

---

#==================== Response ====================
bool success                          # 是否成功
string error_message                  # 错误信息（失败时）
int32 remaining_samples               # 剩余样本数量
```

---

### 查询服务

#### 6. QueryByDesc（文本描述查询）

**功能**：通过文本描述查找物体（CLIP 语义匹配）

```
# srv/QueryByDesc.srv
# Service: /perception/service/query_by_desc

#==================== Request ====================
string description                    # [必填] 文本描述（如 "红色杯子"、"水果"）
int32 top_k                           # [可选] 返回前 K 个结果（默认 5）
float32 min_similarity                # [可选] 最小相似度阈值（默认 0.5）

---

#==================== Response ====================
bool success                          # 是否成功
string error_message                  # 错误信息（失败时）
ObjectInfo[] objects                  # 匹配的物体列表
float32[] similarities                # 对应的相似度分数（0.0-1.0）
```

#### 7. QueryByImage（图片查询）

**功能**：通过参考图片查找相似物体（DINOv3 相似度匹配）

```
# srv/QueryByImage.srv
# Service: /perception/service/query_by_image

#==================== Request ====================
sensor_msgs/Image query_image         # [必填] 查询图片
int32 top_k                           # [可选] 返回前 K 个结果（默认 5）
float32 min_similarity                # [可选] 最小相似度阈值（默认 0.5）

---

#==================== Response ====================
bool success                          # 是否成功
string error_message                  # 错误信息（失败时）
ObjectInfo[] objects                  # 相似的物体列表
float32[] similarities                # 对应的相似度分数（0.0-1.0）
```

#### 8. QueryByLabel（标签查询）

**功能**：通过标签精确查找物体

```
# srv/QueryByLabel.srv
# Service: /perception/service/query_by_label

#==================== Request ====================
string label                          # [必填] 标签（精确匹配，如 "cup"）

---

#==================== Response ====================
bool success                          # 是否成功
string error_message                  # 错误信息（失败时）
ObjectInfo[] objects                  # 匹配的物体列表
```

#### 9. QueryById（ID 查询）

**功能**：通过 ID 获取物体详细信息

```
# srv/QueryById.srv
# Service: /perception/service/query_by_id

#==================== Request ====================
string object_id                      # [必填] 物体 ID（如 "obj_001"）
bool include_image                    # [可选] 是否返回图像（默认 false）

---

#==================== Response ====================
bool success                          # 是否成功
string error_message                  # 错误信息（失败时）
ObjectInfo object                     # 物体完整信息
sensor_msgs/Image cropped_image       # 物体图像（当 include_image=true）
int32 sample_count                    # 样本数量
```

---

### 列表服务

#### 10. ListObjects（列出物体）

**功能**：列出所有已保存的物体

```
# srv/ListObjects.srv
# Service: /perception/service/list_objects

#==================== Request ====================
string label_filter                   # [可选] 按标签过滤（空=不过滤）
int32 offset                          # [可选] 分页偏移（默认 0）
int32 limit                           # [可选] 返回数量限制（默认 100）

---

#==================== Response ====================
bool success                          # 是否成功
string error_message                  # 错误信息（失败时）
ObjectInfo[] objects                  # 物体列表
int32 total_count                     # 总数量（不受 limit 限制）
```

#### 11. ListSamples（列出样本）

**功能**：列出物体的所有样本

```
# srv/ListSamples.srv
# Service: /perception/service/list_samples

#==================== Request ====================
string object_id                      # [必填] 物体 ID

---

#==================== Response ====================
bool success                          # 是否成功
string error_message                  # 错误信息（失败时）
SampleInfo[] samples                  # 样本列表
int32 total_count                     # 样本总数
```

#### 12. ClearAll（清空索引）

**功能**：清空所有物体和样本

```
# srv/ClearAll.srv（或使用 std_srvs/srv/Trigger）
# Service: /perception/service/clear_all

#==================== Request ====================
bool confirm                          # [必填] 确认删除（必须为 true）

---

#==================== Response ====================
bool success                          # 是否成功
string error_message                  # 错误信息（失败时）
int32 deleted_objects                 # 删除的物体数量
int32 deleted_samples                 # 删除的样本数量
```

---

## 消息类型定义

### ObjectInfo（物体信息）

```
# msg/ObjectInfo.msg

string object_id                      # 物体唯一 ID
string label                          # 标签
string description                    # 描述
int64 created_at                      # 创建时间（毫秒时间戳）
int64 updated_at                      # 更新时间（毫秒时间戳）
int32 sample_count                    # 样本数量
```

### SampleInfo（样本信息）

```
# msg/SampleInfo.msg

string sample_id                      # 样本唯一 ID
string object_id                      # 所属物体 ID
int64 created_at                      # 创建时间（毫秒时间戳）
```

---

## 共享内存配置

图像数据通过 DDS 共享内存传输，实现零拷贝：

### Fast DDS 配置（可选，默认已启用）

```xml
<!-- config/fastdds.xml -->
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>shm_transport</transport_id>
            <type>SHM</type>
            <segment_size>10485760</segment_size>  <!-- 10MB -->
        </transport_descriptor>
    </transport_descriptors>
</profiles>
```

### 启用方式

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/path/to/fastdds.xml
```

**注意**：Fast DDS 默认已启用共享内存，通常无需额外配置。

---

## 设计决策（已确认）

### 1. 分割方案 ✅

**选择: NanoSAM（参考 L1 实现）**

- 参考 L1 中的 NanoSAM TensorRT 引擎实现
- 提供高精度的语义级分割
- 支持基于点击的交互式分割

参考代码：`/home/jetson/L1/l1_stage2_segmentation/l1_stage2_segmentation/perception/nanosam_engine.py`

### 2. 点云输出 ✅

**选择: 两者都提供（通过 Action Result 返回）**

| 输出 | Result 字段 | 用途 |
|------|-------------|------|
| 完整点云 | `point_cloud` | 估计物体尺寸、方向、最佳抓取角度 |
| 中心坐标 | `center_3d` | 抓取的参考目标位置 |
| 3D 边界框 | `bbox_min`, `bbox_max` | 物体尺寸范围 |

### 3. 图像保存策略 ✅

**选择: 保存分割图像 + 向量化 + 语义标签**

- 每次成功分割后保存图像
- 图像向量化存储（CLIP + DINOv3）
- 支持通过文本描述或相似图片检索

### 4. 向量化模型 ✅

**选择: CLIP + DINOv3 双模型方案**

| 模型 | 用途 | 参考 |
|------|------|------|
| **CLIP** | 文本 → 图像检索（如"红色杯子"） | 新增实现 |
| **DINOv3** | 图像 → 图像检索（相似物体匹配） | 参考 L1 实现 |

DINOv3 参考代码：`/home/jetson/L1/l1_stage2_segmentation/l1_stage2_segmentation/perception/dino_feature_extractor.py`

### 5. 向量存储 ✅

**选择: FAISS**

- 高效向量检索库
- 支持增量添加、删除
- 支持持久化保存

### 6. 附加功能 ✅

| 功能 | 状态 | 说明 |
|------|------|------|
| 实时可视化 | ✅ 加入 | 分割结果发布给 Web GUI / RViz |
| 删除/清理 | ✅ 加入 | 从索引中删除旧物体记录 |

### 7. CLIP 实现 ✅

**选择: transformers 库的 CLIPModel**

- 兼容性好，支持多种预训练权重
- 使用 `openai/clip-vit-base-patch32`
- 首次运行自动下载模型

### 8. 并发策略 ✅

**选择: 排队处理（FIFO）**

- 同一时刻只处理一个 Action 请求
- 后续请求进入队列等待
- 避免 GPU 资源竞争

### 9. 坐标系输出 ✅

**选择: 相机坐标系**

- 所有 3D 坐标输出为相机坐标系（camera_color_optical_frame）
- TF 转换由调用方处理

### 10. 持久化策略 ✅

**选择: 立即写入**

- 每次 save/delete 操作后立即同步写入磁盘
- 使用临时文件 + 重命名保证原子性

---

## 向量化存储与检索设计

### 双模型向量化流程

```
分割完成后的目标图像
        ↓
┌───────────────────────────────────┐
│          向量化处理               │
│  ┌─────────────┐ ┌─────────────┐  │
│  │    CLIP     │ │   DINOv3    │  │
│  │  文本对齐   │ │  视觉特征   │  │
│  │  向量 512d  │ │  向量 384d  │  │
│  └─────────────┘ └─────────────┘  │
└───────────────────────────────────┘
        ↓
┌───────────────────────────────────┐
│         FAISS 索引存储            │
│  clip_index.faiss (文本检索)      │
│  dino_index.faiss (图像检索)      │
└───────────────────────────────────┘
```

### 检索方式

| 检索类型 | 输入 | 模型 | 示例 |
|----------|------|------|------|
| 文本 → 图像 | 文本描述 | CLIP | "找一个红色杯子" |
| 图像 → 图像 | 参考图片 | DINOv3 | 上传一张杯子图片，找相似物体 |
| 图像 → 文本 | 查询图片 | CLIP | 返回该物体的描述/标签 |

### 存储结构

```
src/perception/data/                 # 数据存储目录（在 perception 包内）
├── index.yaml                       # 物体元数据索引
├── faiss/
│   ├── clip_index.faiss             # CLIP 向量索引
│   ├── dino_index.faiss             # DINOv3 向量索引
│   └── id_mapping.json              # 索引位置 → sample_id 映射
└── images/
    ├── obj_001_cropped.png          # 物体裁剪图
    └── ...
```

### 物体存储表（index.yaml）

```yaml
# 物体表（支持多样本）
objects:
  obj_001:                            # object_id 作为 key
    label: "cup"                      # 分类标签（可选）
    description: "红色陶瓷杯子"        # 文本描述（可选）
    created_at: 1732704625000         # 创建时间（毫秒时间戳）
    updated_at: 1732704800000         # 更新时间（毫秒时间戳）
    image_path: "images/obj_001_cropped.png"
    
    # 多样本（支持多角度增强）
    samples:
      s001:
        created_at: 1732704625000
        clip_index: 0                 # FAISS clip_index 中的位置
        dino_index: 0                 # FAISS dino_index 中的位置
      s002:
        created_at: 1732704700000
        clip_index: 1
        dino_index: 1
      s003:
        created_at: 1732704800000
        clip_index: 2
        dino_index: 2
    
    sample_count: 3

  obj_002:
    label: "bottle"
    description: "绿色塑料瓶"
    created_at: 1732704670000
    updated_at: 1732704670000
    image_path: "images/obj_002_cropped.png"
    samples:
      s001:
        created_at: 1732704670000
        clip_index: 3
        dino_index: 3
    sample_count: 1

# 元信息
metadata:
  total_objects: 2
  total_samples: 4
  last_updated: 1732704800000
  clip_dimension: 512
  dino_dimension: 384
```

### FAISS 索引映射（id_mapping.json）

```json
{
  "clip": {
    "0": {"object_id": "obj_001", "sample_id": "s001"},
    "1": {"object_id": "obj_001", "sample_id": "s002"},
    "2": {"object_id": "obj_001", "sample_id": "s003"},
    "3": {"object_id": "obj_002", "sample_id": "s001"}
  },
  "dino": {
    "0": {"object_id": "obj_001", "sample_id": "s001"},
    "1": {"object_id": "obj_001", "sample_id": "s002"},
    "2": {"object_id": "obj_001", "sample_id": "s003"},
    "3": {"object_id": "obj_002", "sample_id": "s001"}
  }
}
```

### 查询流程（多样本）

```
输入: "红色杯子"
    ↓ CLIP 编码文本
    ↓ FAISS 检索 clip_index
相似索引: [0, 2, 1]（按相似度排序）
    ↓ 通过 id_mapping 映射
    ↓ 相同 object_id 的样本取最高分
    
结果:
  - obj_001: 最高相似度 0.91（来自 sample s002）
    label: "cup"
    description: "红色陶瓷杯子"
```

---

## 参数配置

```yaml
perception_node:
  ros__parameters:
    # ===== 输出坐标系 =====
    output_frame_id: "camera_color_optical_frame"
    
    # ===== 保存配置 =====
    save_directory: "~/perception_output"
    save_pointcloud_pcd: false       # 是否额外保存 PCD 文件
    
    # ===== NanoSAM 配置 =====
    nanosam_config: "config/nanosam_config.yaml"
    
    # ===== CLIP 配置 =====
    clip_model: "ViT-B/32"           # 或 ViT-L/14
    clip_device: "cuda"
    
    # ===== DINOv3 配置 =====
    dino_config: "config/dino_config.yaml"
    dino_device: "cuda"
    
    # ===== FAISS 配置 =====
    faiss_index_type: "IndexFlatL2"  # 或 IndexIVFFlat
    faiss_nprobe: 10                 # IVF 检索参数
    
    # ===== 检索配置 =====
    similarity_threshold: 0.7        # 检索相似度阈值
    top_k: 5                         # 返回前 K 个结果
```

---

## Action 与 Service 接口汇总

### Action（需要 GPU 计算）

#### 原子 Action（3个）

| Action 名 | 说明 |
|-----------|------|
| `/perception/action/segment` | 目标分割（NanoSAM） |
| `/perception/action/pointcloud` | 点云计算 |
| `/perception/action/vectorize` | 向量化（CLIP + DINOv3） |

#### 组合 Action（3个）

| Action 名 | 流程 | 说明 |
|-----------|------|------|
| `/perception/action/object_target` | 分割→点云 | 抓取定位 |
| `/perception/action/object_record` | 向量化→保存 | 记录物体 |
| `/perception/action/object_process` | 分割→点云→向量化→保存 | 完整流程 |

#### 组合关系

```
object_process = object_target + object_record
object_target  = segment + pointcloud
object_record  = vectorize + save_object(service)
```

### Service - 存储操作

| 服务名 | 说明 |
|--------|------|
| `/perception/service/save_object` | 保存物体（向量 + 元数据） |
| `/perception/service/add_sample` | 给物体增加样本 |
| `/perception/service/update_object` | 更新标签/描述 |
| `/perception/service/delete_object` | 删除物体 |
| `/perception/service/delete_sample` | 删除样本 |

### Service - 查询操作

| 服务名 | 查询方式 | 说明 |
|--------|----------|------|
| `/perception/service/query_by_desc` | CLIP 语义 | 文本描述 → 物体列表 |
| `/perception/service/query_by_image` | DINOv3 相似度 | 参考图片 → 相似物体 |
| `/perception/service/query_by_label` | 精确匹配 | 标签 → 物体列表 |
| `/perception/service/query_by_id` | 直接读取 | ID → 详细信息 |

### Service - 列表操作

| 服务名 | 说明 |
|--------|------|
| `/perception/service/list_objects` | 列出所有物体 |
| `/perception/service/list_samples` | 列出物体的样本 |
| `/perception/service/clear_all` | 清空所有数据 |

### Service - 系统操作

| 服务名 | 说明 |
|--------|------|
| `/perception/service/get_status` | 健康检查（模块状态、资源使用、统计信息） |

---

## 节点结构设计

```
src/perception/                      # ROS 2 包（参照 robot_skill 结构）
├── perception.md                    # 本文档
├── start_perception.sh              # 启动/停止脚本
├── package.xml
├── setup.py
├── setup.cfg
├── CMakeLists.txt
├── resource/
│   └── perception
├── data/                            # 数据存储目录
│   ├── index.yaml                   # 物体元数据索引
│   ├── faiss/                       # FAISS 索引
│   └── images/                      # 物体图像
├── models/                          # 模型文件（从 L1 拷贝）
│   ├── nanosam_image_encoder.engine
│   ├── nanosam_mask_decoder.engine
│   └── dinov2_vits14.pth
├── action/                          # Action 定义
│   ├── Segment.action               # 原子：目标分割
│   ├── PointCloud.action            # 原子：点云计算
│   ├── Vectorize.action             # 原子：向量化
│   ├── ObjectTarget.action          # 组合：抓取定位（分割→点云）
│   ├── ObjectRecord.action          # 组合：记录物体（向量化→保存）
│   └── ObjectProcess.action         # 组合：完整流程
├── srv/                             # Service 定义（13个）
│   ├── SaveObject.srv
│   ├── AddSample.srv
│   ├── UpdateObject.srv
│   ├── DeleteObject.srv
│   ├── DeleteSample.srv
│   ├── QueryByDesc.srv
│   ├── QueryByImage.srv
│   ├── QueryByLabel.srv
│   ├── QueryById.srv
│   ├── ListObjects.srv
│   ├── ListSamples.srv
│   ├── ClearAll.srv
│   └── GetStatus.srv                # 健康检查
├── msg/                             # 消息定义
│   ├── ObjectInfo.msg
│   └── SampleInfo.msg
├── config/                          # 配置文件
│   └── perception_config.yaml       # 统一配置
├── launch/
│   └── perception.launch.py
├── scripts/
│   └── perception_node.py           # 入口脚本
├── src/
│   └── perception_core/
│       ├── __init__.py
│       ├── constants.py             # 全局常量
│       ├── error_codes.py           # 错误码定义
│       ├── node.py                  # PerceptionNode（协调层）
│       ├── segmentation.py          # SegmentationModule
│       ├── pointcloud.py            # PointCloudModule
│       ├── vectorizer.py            # VectorizerModule
│       ├── vector_manager.py        # VectorManager（FAISS）
│       └── object_storage_manager.py # ObjectStorageManager
└── tests/
```

---

## 类接口设计

### 1. SegmentationModule（分割模块）

**文件**：`src/perception_core/segmentation.py`

**职责**：封装 NanoSAM，提供图像分割能力

| 方法 | 类型 | 入参 | 出参 | 说明 |
|------|------|------|------|------|
| `__init__` | 同步 | config: dict | - | 初始化配置 |
| `initialize` | 异步 | - | bool | 加载 NanoSAM 引擎 |
| `is_ready` | 同步 | - | bool | 检查是否就绪 |
| `segment` | 异步 | image: ndarray, click_x: float, click_y: float | SegmentResult | 执行分割 |
| `create_visualization` | 同步 | image: ndarray, mask: ndarray, click_point: tuple | ndarray | 生成可视化图 |

**SegmentResult 结构**：
- `mask`: ndarray - 分割掩码 (H, W)
- `cropped_image`: ndarray - 裁剪后的目标图像
- `confidence`: float - 置信度
- `mask_area`: int - 掩码面积（像素数）

---

### 2. PointCloudModule（点云模块）

**文件**：`src/perception_core/pointcloud.py`

**职责**：深度图 → 3D 点云计算

| 方法 | 类型 | 入参 | 出参 | 说明 |
|------|------|------|------|------|
| `__init__` | 同步 | config: dict | - | 初始化配置 |
| `initialize` | 异步 | - | bool | 初始化模块 |
| `is_ready` | 同步 | - | bool | 检查是否就绪 |
| `compute` | 异步 | mask: ndarray, depth_image: ndarray, camera_info: CameraInfo | PointCloudResult | 计算点云 |

**PointCloudResult 结构**：
- `point_cloud`: PointCloud2 - 目标点云
- `center_3d`: tuple(float, float, float) - 质心坐标
- `bbox_min`: tuple(float, float, float) - 边界框最小点
- `bbox_max`: tuple(float, float, float) - 边界框最大点
- `point_count`: int - 有效点数量

---

### 3. VectorizerModule（向量化模块）

**文件**：`src/perception_core/vectorizer.py`

**职责**：CLIP + DINOv3 特征提取

| 方法 | 类型 | 入参 | 出参 | 说明 |
|------|------|------|------|------|
| `__init__` | 同步 | config: dict | - | 初始化配置 |
| `initialize` | 异步 | - | bool | 加载 CLIP 和 DINOv3 模型 |
| `is_ready` | 同步 | - | bool | 检查是否就绪 |
| `extract` | 异步 | image: ndarray | VectorizeResult | 提取特征向量 |
| `encode_text` | 异步 | text: str | ndarray | CLIP 文本编码 |

**VectorizeResult 结构**：
- `clip_embedding`: ndarray - CLIP 向量 (512,)
- `dino_embedding`: ndarray - DINOv3 向量 (384,)

---

### 4. VectorManager（向量索引管理）

**文件**：`src/perception_core/vector_manager.py`

**职责**：FAISS 向量索引的增删查

| 方法 | 类型 | 入参 | 出参 | 说明 |
|------|------|------|------|------|
| `__init__` | 同步 | config: dict, storage_dir: str | - | 初始化配置 |
| `initialize` | 异步 | - | bool | 加载或创建 FAISS 索引 |
| `is_ready` | 同步 | - | bool | 检查是否就绪 |
| `add_vectors` | 同步 | clip_emb: ndarray, dino_emb: ndarray | tuple(int, int) | 添加向量，返回索引位置 |
| `search_by_clip` | 同步 | query_emb: ndarray, top_k: int | list[tuple(int, float)] | CLIP 检索 |
| `search_by_dino` | 同步 | query_emb: ndarray, top_k: int | list[tuple(int, float)] | DINOv3 检索 |
| `delete_vectors` | 同步 | clip_indices: list, dino_indices: list | bool | 删除向量 |
| `save` | 同步 | - | bool | 持久化到磁盘 |
| `clear` | 同步 | - | bool | 清空索引 |

---

### 5. ObjectStorageManager（物体存储管理）

**文件**：`src/perception_core/object_storage_manager.py`

**职责**：物体数据的增删改查（依赖 VectorManager）

| 方法 | 类型 | 入参 | 出参 | 说明 |
|------|------|------|------|------|
| `__init__` | 同步 | config: dict, storage_dir: str, vector_manager: VectorManager | - | 初始化 |
| `initialize` | 异步 | - | bool | 加载 index.yaml |
| `is_ready` | 同步 | - | bool | 检查是否就绪 |

**物体操作**：

| 方法 | 入参 | 出参 | 说明 |
|------|------|------|------|
| `save_object` | image: ndarray, clip_emb: ndarray, dino_emb: ndarray, label: str, desc: str | str | 保存物体，返回 object_id |
| `add_sample` | object_id: str, clip_emb: ndarray, dino_emb: ndarray | str | 增加样本，返回 sample_id |
| `update_object` | object_id: str, label: str, desc: str | bool | 更新标签/描述 |
| `delete_object` | object_id: str | int | 删除物体，返回删除的样本数 |
| `delete_sample` | object_id: str, sample_id: str | int | 删除样本，返回剩余样本数 |

**查询操作**：

| 方法 | 入参 | 出参 | 说明 |
|------|------|------|------|
| `query_by_clip` | emb: ndarray, top_k: int, min_sim: float | list[ObjectInfo] | 文本向量查询 |
| `query_by_dino` | emb: ndarray, top_k: int, min_sim: float | list[ObjectInfo] | 图像向量查询 |
| `query_by_label` | label: str | list[ObjectInfo] | 标签精确查询 |
| `query_by_id` | object_id: str | ObjectInfo | ID 查询 |

**列表操作**：

| 方法 | 入参 | 出参 | 说明 |
|------|------|------|------|
| `list_objects` | label_filter: str, offset: int, limit: int | list[ObjectInfo] | 列出物体 |
| `list_samples` | object_id: str | list[SampleInfo] | 列出样本 |
| `clear_all` | - | tuple(int, int) | 清空，返回 (物体数, 样本数) |

---

### 6. PerceptionNode（主节点 - 协调层）

**文件**：`src/perception_core/node.py`

**职责**：ROS 2 节点入口，协调各模块，注册 Action/Service

| 属性 | 类型 | 说明 |
|------|------|------|
| `_segmentation` | SegmentationModule | 分割模块 |
| `_pointcloud` | PointCloudModule | 点云模块 |
| `_vectorizer` | VectorizerModule | 向量化模块 |
| `_vector_manager` | VectorManager | 向量索引管理 |
| `_storage` | ObjectStorageManager | 物体存储管理 |
| `_ready` | bool | 节点是否就绪 |

| 方法 | 类型 | 说明 |
|------|------|------|
| `__init__` | 同步 | 初始化节点，创建模块实例，启动初始化定时器 |
| `_initialize_modules` | 异步 | 并行初始化所有模块，等待全部就绪后注册服务 |
| `_register_actions` | 同步 | 注册 6 个 Action Server |
| `_register_services` | 同步 | 注册 13 个 Service Server |

**Action 回调（协调各模块）**：

| 回调 | 调用模块 | 说明 |
|------|----------|------|
| `_segment_execute` | segmentation | 目标分割 |
| `_pointcloud_execute` | pointcloud | 点云计算 |
| `_vectorize_execute` | vectorizer | 向量化 |
| `_object_target_execute` | segmentation → pointcloud | 抓取定位 |
| `_object_record_execute` | vectorizer → storage | 记录物体 |
| `_object_process_execute` | segmentation → pointcloud → vectorizer → storage | 完整流程 |

**Service 回调**：

| 回调 | 调用模块 | 说明 |
|------|----------|------|
| `_save_object_callback` | storage | 保存物体 |
| `_add_sample_callback` | storage | 增加样本 |
| `_update_object_callback` | storage | 更新物体 |
| `_delete_object_callback` | storage | 删除物体 |
| `_delete_sample_callback` | storage | 删除样本 |
| `_query_by_desc_callback` | vectorizer → storage | 文本查询 |
| `_query_by_image_callback` | vectorizer → storage | 图片查询 |
| `_query_by_label_callback` | storage | 标签查询 |
| `_query_by_id_callback` | storage | ID 查询 |
| `_list_objects_callback` | storage | 列出物体 |
| `_list_samples_callback` | storage | 列出样本 |
| `_clear_all_callback` | storage | 清空索引 |
| `_get_status_callback` | all modules | 健康检查 |

---

## 配置文件

**文件**：`config/perception_config.yaml`

```yaml
perception:
  # ===== 存储配置 =====
  storage_dir: "data"               # 相对于 perception 包目录
  log_dir: "L2/log/perception"      # 日志目录（绝对路径）
  
  # ===== NanoSAM 配置 =====
  segmentation:
    engine_path: "models/nanosam_image_encoder.engine"   # 相对于 perception 包
    decoder_path: "models/nanosam_mask_decoder.engine"
    min_confidence: 0.3
    min_mask_area: 100
    
  # ===== 点云配置 =====
  pointcloud:
    output_frame_id: "camera_color_optical_frame"
    min_point_count: 10
    min_depth_mm: 100
    max_depth_mm: 10000
    
  # ===== CLIP 配置 =====
  clip:
    model: "openai/clip-vit-base-patch32"
    device: "cuda"
    
  # ===== DINOv3 配置 =====
  dino:
    model_path: "models/dinov2_vits14.pth"               # 相对于 perception 包
    device: "cuda"
    feature_dim: 384
    
  # ===== FAISS 配置 =====
  vector:
    index_type: "IndexFlatL2"
    clip_dimension: 512
    dino_dimension: 384
    
  # ===== 查询配置 =====
  query:
    default_top_k: 5
    default_min_similarity: 0.5
    
  # ===== 超时配置 =====
  timeout:
    segment: 5.0
    pointcloud: 2.0
    vectorize: 6.0
    object_target: 10.0
    object_record: 10.0
    object_process: 20.0
    faiss_search: 1.0
    file_write: 2.0
    
  # ===== 并发配置 =====
  concurrency:
    max_queue_size: 10
    queue_timeout: 30.0
```

---

## 启动流程

```
start_perception.sh --start
       │
       ▼
perception_node.py
       │
       ▼
PerceptionNode.__init__()
       ├── 加载配置
       ├── 创建模块实例（不加载模型）
       └── 启动初始化定时器
       
       ▼
_initialize_modules() [异步]
       │
       ├── 并行初始化（耗时模块）:
       │   ├── SegmentationModule.initialize()  → NanoSAM
       │   ├── VectorizerModule.initialize()    → CLIP + DINOv3
       │   ├── PointCloudModule.initialize()    → 轻量
       │   └── VectorManager.initialize()       → FAISS
       │
       ├── 串行初始化（有依赖）:
       │   └── ObjectStorageManager.initialize() → 依赖 VectorManager
       │
       ├── 检查所有模块 is_ready()
       │
       ├── 注册 Action Server (6个)
       ├── 注册 Service Server (12个)
       │
       └── self._ready = True
              │
              ▼
         "✅ Perception Node 服务就绪"
```

---

## 处理流程

```
1. 初始化
   ├── 加载 NanoSAM 引擎
   ├── 加载 CLIP 模型
   ├── 加载 DINOv3 模型
   ├── 初始化 FAISS 索引（从磁盘加载或创建新的）
   ├── 创建 Action Server（6个）/ Service Server
   └── 初始化存储目录

2. 原子 Action 执行

   segment（目标分割）
   ├── 接收 color_image, click_x, click_y
   ├── 调用 NanoSAM 分割 → mask
   ├── 裁剪目标图像 → cropped_image
   ├── 生成可视化图 → visualization
   └── 返回 mask, cropped_image, visualization, confidence

   pointcloud（点云计算）
   ├── 接收 mask, depth_image, camera_info
   ├── 应用 mask 到深度图
   ├── 反投影到 3D → point_cloud
   ├── 计算中心 → center_3d
   ├── 计算边界框 → bbox_min, bbox_max
   └── 返回 point_cloud, center_3d, bbox

   vectorize（向量化）
   ├── 接收 cropped_image
   ├── CLIP 编码 → clip_embedding (512d)
   ├── DINOv3 编码 → dino_embedding (384d)
   └── 返回 clip_embedding, dino_embedding

   save_object（保存到索引）
   ├── 接收 cropped_image, mask, embeddings, center_3d, bbox, label, description
   ├── 添加到 FAISS 索引
   ├── 保存图像文件
   ├── 更新 index.yaml
   └── 返回 object_id

3. 组合 Action 执行

   object_process（完整流程）
   ├── 调用 segment()
   ├── 调用 pointcloud()
   ├── 调用 vectorize()
   ├── 调用 save_object()
   └── 返回完整结果

   target_object（抓取定位）
   ├── 调用 segment()
   ├── 调用 pointcloud()
   └── 返回分割+点云结果（不保存）

4. 查询服务处理
   ├── query_by_text: CLIP 编码文本 → FAISS 检索 → 返回匹配物体
   ├── query_by_image: DINOv3 编码图像 → FAISS 检索 → 返回相似物体
   └── query_by_id: 从 index.yaml 读取 → 返回完整信息

5. 管理服务处理
   ├── delete_object: 从 FAISS 删除 + 更新 index.yaml
   ├── clear_all: 重置所有索引
   └── list_objects: 返回所有物体列表
```

---

## 3D 反投影公式

从像素坐标 (u, v) 和深度 d 计算 3D 点 (X, Y, Z)：

```
Z = d
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
```

其中 fx, fy, cx, cy 来自 CameraInfo 消息。

---

## 技术依赖

| 依赖 | 版本 | 说明 |
|------|------|------|
| `torch` | >= 2.0 | PyTorch |
| `torchvision` | >= 0.15 | 图像处理 |
| `transformers` | >= 4.30 | CLIP 模型（CLIPModel） |
| `faiss-cpu` / `faiss-gpu` | >= 1.7 | 向量检索 |
| `opencv-python` | >= 4.5 | 图像处理 |
| `open3d` | >= 0.17 | 点云处理（可选） |
| `tensorrt` | >= 8.5 | NanoSAM 推理 |

---

## 启动脚本规格

### start_perception.sh

**功能**：启动/停止 Perception 节点

**用法**：
```bash
./start_perception.sh --start   # 启动节点
./start_perception.sh --stop    # 停止节点
```

**特点**：
- 启动完成后脚本退出，不常驻
- 停止完成后脚本退出
- 日志输出到 `L2/log/perception/`

**启动流程**：
```
--start
    ├── 检查节点是否已运行
    ├── source ROS 2 环境
    ├── 启动 perception_node（后台）
    ├── 等待节点就绪（检查 /perception/service/get_status）
    ├── 输出启动日志
    └── 脚本退出
```

**停止流程**：
```
--stop
    ├── 查找 perception_node 进程
    ├── 发送 SIGTERM
    ├── 等待进程退出（超时 10s）
    ├── 超时则 SIGKILL
    └── 脚本退出
```

---

## 日志规范

### 日志目录

```
/home/jetson/L2/log/perception/  # 日志统一放到 L2/log 下
├── build.log                    # 编译日志
├── startup_20251127.log         # 启动日志（按日期）
├── runtime_20251127.log         # 运行日志（按日期）
└── runtime_20251126.log         # 历史日志
```

### 日志格式

```
[2025-11-27 14:30:25.123] [INFO] [perception_node] 模块初始化完成
[2025-11-27 14:30:26.456] [WARN] [segmentation] 分割置信度较低: 0.45
[2025-11-27 14:30:27.789] [ERROR] [pointcloud] 深度图无效点过多: 2001 (code: 3002)
```

### 日志级别

| 级别 | 用途 |
|------|------|
| DEBUG | 调试信息（开发时启用） |
| INFO | 正常运行信息 |
| WARN | 警告（不影响功能） |
| ERROR | 错误（带错误码） |

---

## 错误码体系

### 错误码格式

`error_message` 格式：`[EXXX] 错误描述`

### 错误码定义

#### 1xxx - 初始化错误

| 错误码 | 说明 |
|--------|------|
| 1001 | NanoSAM 引擎加载失败 |
| 1002 | NanoSAM 解码器加载失败 |
| 1003 | CLIP 模型加载失败 |
| 1004 | DINOv3 模型加载失败 |
| 1005 | FAISS 索引加载失败 |
| 1006 | 存储目录创建失败 |
| 1007 | 配置文件读取失败 |
| 1010 | GPU 内存不足 |
| 1011 | CUDA 不可用 |

#### 2xxx - 分割错误

| 错误码 | 说明 |
|--------|------|
| 2001 | 输入图像为空 |
| 2002 | 图像格式不支持 |
| 2003 | 点击坐标超出图像范围 |
| 2004 | 分割结果为空（未检测到目标） |
| 2005 | 分割置信度过低（< MIN_CONFIDENCE） |
| 2006 | 掩码面积过小（< MIN_MASK_AREA） |
| 2007 | NanoSAM 推理超时 |
| 2008 | NanoSAM 推理异常 |

#### 3xxx - 点云错误

| 错误码 | 说明 |
|--------|------|
| 3001 | 深度图为空 |
| 3002 | 深度图无效点过多（> MAX_INVALID_DEPTH_RATIO） |
| 3003 | 相机内参无效 |
| 3004 | 掩码与深度图尺寸不匹配 |
| 3005 | 有效点数量不足（< MIN_POINT_COUNT） |
| 3006 | 点云计算超时 |

#### 4xxx - 向量化错误

| 错误码 | 说明 |
|--------|------|
| 4001 | CLIP 编码失败 |
| 4002 | DINOv3 编码失败 |
| 4003 | 图像预处理失败 |
| 4004 | 文本编码失败（空文本） |
| 4005 | 向量化超时 |

#### 5xxx - 存储错误

| 错误码 | 说明 |
|--------|------|
| 5001 | 物体 ID 不存在 |
| 5002 | 样本 ID 不存在 |
| 5003 | 图像保存失败 |
| 5004 | index.yaml 写入失败 |
| 5005 | FAISS 索引保存失败 |
| 5006 | 向量维度不匹配 |
| 5007 | 物体已存在（重复 ID） |
| 5008 | 删除最后一个样本（物体至少保留一个样本） |

#### 6xxx - 查询错误

| 错误码 | 说明 |
|--------|------|
| 6001 | 查询文本为空 |
| 6002 | 查询图像为空 |
| 6003 | 无匹配结果 |
| 6004 | 标签不存在 |

#### 9xxx - 系统错误

| 错误码 | 说明 |
|--------|------|
| 9001 | 节点未就绪 |
| 9002 | Action 被取消 |
| 9003 | 内部异常 |
| 9004 | 资源繁忙（排队中） |

---

## 超时与性能指标

### 预期耗时（Jetson Orin）

| 操作 | 预期耗时 | 超时设置 |
|------|----------|----------|
| NanoSAM 分割 | 200-500ms | 5s |
| 点云计算 | 50-100ms | 2s |
| CLIP 编码（图像） | 100-200ms | 3s |
| CLIP 编码（文本） | 20-50ms | 1s |
| DINOv3 编码 | 150-300ms | 3s |
| 向量化（CLIP+DINOv3） | 300-600ms | 6s |
| FAISS 检索（10k 向量） | 5-20ms | 1s |
| 文件写入 | 10-50ms | 2s |

### 组合 Action 超时

| Action | 预期耗时 | 超时设置 |
|--------|----------|----------|
| object_target | 300-700ms | 10s |
| object_record | 350-700ms | 10s |
| object_process | 600-1200ms | 20s |

### 配置项

```yaml
perception:
  timeout:
    segment: 5.0          # 分割超时（秒）
    pointcloud: 2.0       # 点云超时
    vectorize: 6.0        # 向量化超时
    object_target: 10.0   # 组合 Action
    object_record: 10.0
    object_process: 20.0
    faiss_search: 1.0     # FAISS 检索
    file_write: 2.0       # 文件写入
```

---

## 边界条件与常量

### 全局常量（constants.py）

```python
# ===== 图像相关 =====
MAX_IMAGE_SIZE = 4096 * 4096      # 最大图像像素数（16MP）
MIN_IMAGE_SIZE = 64 * 64          # 最小图像像素数
SUPPORTED_IMAGE_FORMATS = ['rgb8', 'bgr8', 'rgba8', 'bgra8']

# ===== 分割相关 =====
MIN_CONFIDENCE = 0.3              # 最小分割置信度
MIN_MASK_AREA = 100               # 最小掩码面积（像素）
MAX_MASK_AREA_RATIO = 0.9         # 最大掩码面积占比

# ===== 点云相关 =====
MIN_POINT_COUNT = 10              # 最小有效点数
MAX_INVALID_DEPTH_RATIO = 0.8     # 最大深度无效点占比
MIN_DEPTH_MM = 100                # 最小有效深度（mm）
MAX_DEPTH_MM = 10000              # 最大有效深度（mm）

# ===== 向量相关 =====
CLIP_EMBEDDING_DIM = 512          # CLIP 向量维度
DINO_EMBEDDING_DIM = 384          # DINOv3 向量维度

# ===== 存储相关 =====
MAX_OBJECTS = 10000               # 最大物体数量
MAX_SAMPLES_PER_OBJECT = 100      # 单物体最大样本数
MAX_LABEL_LENGTH = 64             # 标签最大长度
MAX_DESCRIPTION_LENGTH = 256      # 描述最大长度

# ===== 查询相关 =====
MAX_TOP_K = 100                   # 最大返回数量
DEFAULT_TOP_K = 5                 # 默认返回数量
DEFAULT_MIN_SIMILARITY = 0.5     # 默认最小相似度
```

### 可配置项（perception_config.yaml）

```yaml
perception:
  # 分割阈值（可调）
  segmentation:
    min_confidence: 0.3
    min_mask_area: 100
    
  # 点云阈值（可调）
  pointcloud:
    min_point_count: 10
    min_depth_mm: 100
    max_depth_mm: 10000
    
  # 查询默认值（可调）
  query:
    default_top_k: 5
    default_min_similarity: 0.5
```

---

## 健康检查接口

### GetStatus Service

```
# srv/GetStatus.srv
# Service: /perception/service/get_status

#==================== Request ====================
# 无参数

---

#==================== Response ====================
bool ready                        # 节点是否就绪
string node_status                # 节点状态: "initializing" / "ready" / "error"

# 各模块状态
bool segmentation_ready           # 分割模块就绪
bool pointcloud_ready             # 点云模块就绪
bool vectorizer_ready             # 向量化模块就绪
bool storage_ready                # 存储模块就绪

# 资源使用
int64 gpu_memory_used_mb          # GPU 内存使用（MB）
int64 gpu_memory_total_mb         # GPU 内存总量（MB）

# 索引统计
int32 total_objects               # 物体总数
int32 total_samples               # 样本总数

# 运行统计
int64 uptime_seconds              # 运行时长（秒）
int32 requests_processed          # 已处理请求数
int32 requests_failed             # 失败请求数
```

### 节点结构更新

新增 Service：
```
srv/
├── ...（原有 12 个）
└── GetStatus.srv                 # 健康检查（新增）
```

Service 总数：13 个

---

## 并发策略

### Action 并发处理

**策略**：排队处理（FIFO）

- 同一时刻只处理一个 Action 请求
- 后续请求进入队列等待
- 队列满时返回 `[9004] 资源繁忙` 错误

```
请求1 ──────►│        │
请求2 ──────►│  队列  │──────► GPU 处理 ──────► 结果
请求3 ──────►│        │
```

### 配置

```yaml
perception:
  concurrency:
    max_queue_size: 10            # 最大队列长度
    queue_timeout: 30.0           # 排队超时（秒）
```

---

## 坐标系说明

### 输出坐标系

所有 3D 坐标输出均为**相机坐标系**（camera_color_optical_frame）：
- X 轴：向右
- Y 轴：向下
- Z 轴：向前（深度方向）

### TF 转换

坐标系转换由调用方处理，本节点不做 TF 查询。

如需转换到机器人基座坐标系，调用方可使用：
```python
tf_buffer.transform(point_stamped, "base_link")
```

---

## 持久化策略

### 写入时机

**立即写入**：每次 `save_object` / `delete_object` / `add_sample` / `delete_sample` 操作后立即同步写入磁盘。

### 写入内容

| 操作 | 写入文件 |
|------|----------|
| save_object | index.yaml, clip_index.faiss, dino_index.faiss, id_mapping.json, 图像文件 |
| add_sample | index.yaml, clip_index.faiss, dino_index.faiss, id_mapping.json |
| delete_object | index.yaml, clip_index.faiss, dino_index.faiss, id_mapping.json |
| delete_sample | index.yaml, clip_index.faiss, dino_index.faiss, id_mapping.json |
| update_object | index.yaml |
| clear_all | 删除所有文件并重建空索引 |

### 原子性保证

使用临时文件 + 重命名方式保证写入原子性：
```
1. 写入 index.yaml.tmp
2. mv index.yaml.tmp index.yaml
```

---

## 模型文件说明

### NanoSAM（从 L1 拷贝）

```
src/perception/models/
├── nanosam_image_encoder.engine   # TensorRT 引擎
└── nanosam_mask_decoder.engine    # TensorRT 解码器
```

来源：`/home/jetson/L1/l1_stage2_segmentation/models/`

### DINOv3（从 L1 拷贝）

```
src/perception/models/
└── dinov2_vits14.pth              # DINOv2 ViT-S/14 权重
```

来源：`/home/jetson/L1/l1_stage2_segmentation/models/`

参考实现：`/home/jetson/L1/l1_stage2_segmentation/l1_stage2_segmentation/perception/dino_feature_extractor.py`

### CLIP（transformers 自动下载）

使用 `transformers` 库的 CLIPModel：
```python
from transformers import CLIPModel, CLIPProcessor

model = CLIPModel.from_pretrained("openai/clip-vit-base-patch32")
processor = CLIPProcessor.from_pretrained("openai/clip-vit-base-patch32")
```

首次运行时自动下载到 `~/.cache/huggingface/`

---

## 下一步

### 设计阶段
1. ✅ 确认架构方案（单一节点）
2. ✅ 确认分割方案（NanoSAM）
3. ✅ 确认点云输出（完整点云 + 中心坐标）
4. ✅ 确认向量化方案（CLIP + DINOv3）
5. ✅ 确认存储方案（FAISS）
6. ✅ 确认附加功能（可视化、删除/清理）
7. ✅ 确认交互模式（Action + 共享内存传输图像）
8. ✅ 确认 CLIP 实现（transformers CLIPModel）
9. ✅ 确认并发策略（排队处理）
10. ✅ 确认坐标系输出（相机坐标系）
11. ✅ 确认持久化策略（立即写入）
12. ✅ 定义错误码体系
13. ✅ 定义日志规范
14. ✅ 定义超时与性能指标
15. ✅ 定义健康检查接口
16. ✅ 定义边界条件与常量

### 实现阶段

---

#### Phase 1: 基础设施（无依赖）

| 步骤 | 任务 | 产出文件 | 说明 |
|------|------|----------|------|
| 1.1 | 创建目录结构 | 所有目录 | action/, srv/, msg/, config/, src/, scripts/, launch/, models/, data/ |
| 1.2 | 创建 package.xml | package.xml | 定义包名、依赖（rclpy, sensor_msgs, geometry_msgs 等） |
| 1.3 | 创建 setup.py + setup.cfg | setup.py, setup.cfg | Python 包配置，入口点 |
| 1.4 | 创建 CMakeLists.txt | CMakeLists.txt | 消息生成配置 |
| 1.5 | 创建 resource 标记文件 | resource/perception | ament 索引标记 |
| 1.6 | 创建全局常量 | src/perception_core/constants.py | MIN_CONFIDENCE, MAX_IMAGE_SIZE 等 |
| 1.7 | 创建错误码定义 | src/perception_core/error_codes.py | PerceptionError 类，错误码枚举 |

---

#### Phase 2: 消息定义（依赖 Phase 1）

| 步骤 | 任务 | 产出文件 | 说明 |
|------|------|----------|------|
| 2.1 | 定义 ObjectInfo 消息 | msg/ObjectInfo.msg | object_id, label, description, timestamps, sample_count |
| 2.2 | 定义 SampleInfo 消息 | msg/SampleInfo.msg | sample_id, object_id, created_at |
| 2.3 | 定义 Segment Action | action/Segment.action | 原子分割 Action |
| 2.4 | 定义 PointCloud Action | action/PointCloud.action | 原子点云 Action |
| 2.5 | 定义 Vectorize Action | action/Vectorize.action | 原子向量化 Action |
| 2.6 | 定义 ObjectTarget Action | action/ObjectTarget.action | 组合：分割→点云 |
| 2.7 | 定义 ObjectRecord Action | action/ObjectRecord.action | 组合：向量化→保存 |
| 2.8 | 定义 ObjectProcess Action | action/ObjectProcess.action | 组合：完整流程 |
| 2.9 | 定义存储 Service（5个） | srv/SaveObject.srv 等 | Save, Add, Update, Delete, DeleteSample |
| 2.10 | 定义查询 Service（4个） | srv/QueryByDesc.srv 等 | ByDesc, ByImage, ByLabel, ById |
| 2.11 | 定义列表 Service（3个） | srv/ListObjects.srv 等 | List, ListSamples, ClearAll |
| 2.12 | 定义健康检查 Service | srv/GetStatus.srv | 模块状态、资源使用 |
| 2.13 | 编译验证消息 | - | colcon build 验证消息生成正确 |

---

#### Phase 3: 配置文件（依赖 Phase 1）

| 步骤 | 任务 | 产出文件 | 说明 |
|------|------|----------|------|
| 3.1 | 创建配置文件 | config/perception_config.yaml | 所有模块配置项 |
| 3.2 | 拷贝 NanoSAM 模型 | models/nanosam_*.engine | 从 L1 拷贝 TensorRT 引擎 |
| 3.3 | 拷贝 DINOv3 模型 | models/dinov2_vits14.pth | 从 L1 拷贝模型权重 |

---

#### Phase 4: 分割模块（依赖 Phase 2, 3）

| 步骤 | 任务 | 产出文件 | 说明 |
|------|------|----------|------|
| 4.1 | 实现 SegmentationModule 骨架 | src/perception_core/segmentation.py | `__init__`, `is_ready`, 类结构 |
| 4.2 | 实现 initialize 方法 | segmentation.py | 加载 NanoSAM 引擎（参考 L1 nanosam_engine.py） |
| 4.3 | 实现 segment 方法 | segmentation.py | 核心分割逻辑，返回 SegmentResult |
| 4.4 | 实现 create_visualization | segmentation.py | 生成叠加可视化图 |
| 4.5 | 单元测试分割模块 | tests/test_segmentation.py | 验证分割功能正常 |

---

#### Phase 5: 点云模块（依赖 Phase 2）

| 步骤 | 任务 | 产出文件 | 说明 |
|------|------|----------|------|
| 5.1 | 实现 PointCloudModule 骨架 | src/perception_core/pointcloud.py | `__init__`, `is_ready`, 类结构 |
| 5.2 | 实现 initialize 方法 | pointcloud.py | 轻量初始化 |
| 5.3 | 实现 compute 方法 | pointcloud.py | 深度图→点云，计算质心、边界框 |
| 5.4 | 单元测试点云模块 | tests/test_pointcloud.py | 验证点云计算正确 |

---

#### Phase 6: 向量化模块（依赖 Phase 2, 3）

| 步骤 | 任务 | 产出文件 | 说明 |
|------|------|----------|------|
| 6.1 | 实现 VectorizerModule 骨架 | src/perception_core/vectorizer.py | `__init__`, `is_ready`, 类结构 |
| 6.2 | 实现 CLIP 加载 | vectorizer.py | 使用 transformers 加载 CLIPModel |
| 6.3 | 实现 DINOv3 加载 | vectorizer.py | 参考 L1 dino_feature_extractor.py |
| 6.4 | 实现 extract 方法 | vectorizer.py | 图像→CLIP+DINOv3 向量 |
| 6.5 | 实现 encode_text 方法 | vectorizer.py | 文本→CLIP 向量 |
| 6.6 | 单元测试向量化模块 | tests/test_vectorizer.py | 验证向量维度和格式正确 |

---

#### Phase 7: 向量索引模块（依赖 Phase 2）

| 步骤 | 任务 | 产出文件 | 说明 |
|------|------|----------|------|
| 7.1 | 实现 VectorManager 骨架 | src/perception_core/vector_manager.py | `__init__`, `is_ready`, 类结构 |
| 7.2 | 实现 initialize 方法 | vector_manager.py | 加载或创建 FAISS 索引 |
| 7.3 | 实现 add_vectors 方法 | vector_manager.py | 添加向量到索引 |
| 7.4 | 实现 search_by_clip/dino | vector_manager.py | 向量检索 |
| 7.5 | 实现 delete_vectors 方法 | vector_manager.py | 从索引删除（标记删除或重建） |
| 7.6 | 实现 save/clear 方法 | vector_manager.py | 持久化和清空 |
| 7.7 | 单元测试向量管理模块 | tests/test_vector_manager.py | 验证增删查功能 |

---

#### Phase 8: 物体存储模块（依赖 Phase 7）

| 步骤 | 任务 | 产出文件 | 说明 |
|------|------|----------|------|
| 8.1 | 实现 ObjectStorageManager 骨架 | src/perception_core/object_storage_manager.py | `__init__`, `is_ready` |
| 8.2 | 实现 initialize 方法 | object_storage_manager.py | 加载 index.yaml |
| 8.3 | 实现 save_object 方法 | object_storage_manager.py | 保存物体（图像+向量+元数据） |
| 8.4 | 实现 add_sample 方法 | object_storage_manager.py | 给物体增加样本 |
| 8.5 | 实现 update_object 方法 | object_storage_manager.py | 更新标签/描述 |
| 8.6 | 实现 delete_object/sample | object_storage_manager.py | 删除物体或样本 |
| 8.7 | 实现 query_by_* 方法 | object_storage_manager.py | 4 种查询方式 |
| 8.8 | 实现 list_*/clear_all | object_storage_manager.py | 列表和清空操作 |
| 8.9 | 单元测试存储模块 | tests/test_object_storage.py | 验证 CRUD 功能 |

---

#### Phase 9: 主节点实现（依赖 Phase 4-8）

| 步骤 | 任务 | 产出文件 | 说明 |
|------|------|----------|------|
| 9.1 | 实现 PerceptionNode 骨架 | src/perception_core/node.py | `__init__`，创建模块实例 |
| 9.2 | 实现 _initialize_modules | node.py | 异步并行初始化所有模块 |
| 9.3 | 实现 _register_actions | node.py | 注册 6 个 Action Server |
| 9.4 | 实现 _register_services | node.py | 注册 13 个 Service Server |
| 9.5 | 实现 _segment_execute | node.py | 原子 Action：分割 |
| 9.6 | 实现 _pointcloud_execute | node.py | 原子 Action：点云 |
| 9.7 | 实现 _vectorize_execute | node.py | 原子 Action：向量化 |
| 9.8 | 实现 _object_target_execute | node.py | 组合 Action：分割→点云 |
| 9.9 | 实现 _object_record_execute | node.py | 组合 Action：向量化→保存 |
| 9.10 | 实现 _object_process_execute | node.py | 组合 Action：完整流程 |
| 9.11 | 实现存储 Service 回调 | node.py | save, add, update, delete 回调 |
| 9.12 | 实现查询 Service 回调 | node.py | query_by_* 回调 |
| 9.13 | 实现列表 Service 回调 | node.py | list_*, clear_all 回调 |
| 9.14 | 实现 _get_status_callback | node.py | 健康检查回调 |

---

#### Phase 10: 入口和启动脚本（依赖 Phase 9）

| 步骤 | 任务 | 产出文件 | 说明 |
|------|------|----------|------|
| 10.1 | 创建入口脚本 | scripts/perception_node.py | main() 函数，节点启动 |
| 10.2 | 创建 launch 文件 | launch/perception.launch.py | 参数加载、节点启动 |
| 10.3 | 创建启动脚本 | start_perception.sh | --start / --stop 实现 |
| 10.4 | 完整编译测试 | - | colcon build 验证整个包 |

---

#### Phase 11: 集成测试（依赖 Phase 10）

| 步骤 | 任务 | 产出文件 | 说明 |
|------|------|----------|------|
| 11.1 | 测试节点启动 | - | 验证所有模块初始化成功 |
| 11.2 | 测试 segment Action | - | 调用分割，验证掩码输出 |
| 11.3 | 测试 object_target Action | - | 验证分割+点云组合 |
| 11.4 | 测试 object_record Action | - | 验证向量化+保存 |
| 11.5 | 测试查询 Service | - | 验证各种查询方式 |
| 11.6 | 测试删除/清空 Service | - | 验证数据删除和持久化 |
| 11.7 | 端到端测试 | - | 完整流程：分割→保存→查询→删除 |

---

#### 依赖关系图

```
Phase 1 (基础设施)
    │
    ├──► Phase 2 (消息定义) ──┬──► Phase 4 (分割)
    │                         ├──► Phase 5 (点云)
    └──► Phase 3 (配置) ──────┼──► Phase 6 (向量化)
                              │
                              └──► Phase 7 (向量索引)
                                       │
                                       ▼
                              Phase 8 (物体存储)
                                       │
                                       ▼
                              Phase 9 (主节点)
                                       │
                                       ▼
                              Phase 10 (启动脚本)
                                       │
                                       ▼
                              Phase 11 (集成测试)
```

#### 可并行执行

| 阶段 | 可并行任务 |
|------|------------|
| Phase 1 完成后 | Phase 2 + Phase 3 可并行 |
| Phase 2 + 3 完成后 | Phase 4 + 5 + 6 + 7 可并行 |
| Phase 7 完成后 | Phase 8 开始 |
| Phase 4-8 全部完成后 | Phase 9 开始 |

