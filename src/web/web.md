# Web Interactive GUI 节点

## 概述

Web 交互界面节点，提供以下能力：
- **图像推流**：相机图像实时推送到浏览器
- **分割交互**：点击图像触发目标分割，显示高亮
- **物体记录**：保存分割结果为新物体

**特点**：
- 基于 Flask + Socket.IO 实现 Web 服务
- 作为 Perception 节点的 Action Client
- 前后端分离，UI 模板独立

---

## 设计决策（已确认）

### 1. 高亮移除时机 ✅

**选择：以下全部**

| 触发条件 | 说明 |
|----------|------|
| 点击新位置 | 新高亮替换旧高亮 |
| 点击"取消"按钮 | 手动清除 |
| "记录"成功后 | 自动清除 |

### 2. 功能按钮 ✅

| 按钮 | 功能 | 调用接口 | 状态 |
|------|------|----------|------|
| **记录** | 保存为新物体 | `/perception/action/object_record` | ✅ 实现 |
| **采样** | 给已有物体增加样本 | `vectorize` + `add_sample` | 🔲 保留按钮，暂不实现 |
| **取消** | 清除高亮 | - | ✅ 实现 |

### 3. 数据传输方式 ✅

**选择：Web 节点订阅相机话题**

- Web 节点订阅 `/camera/color/image_raw`、`/camera/aligned_depth_to_color/image_raw`、`/camera/color/camera_info`
- 通过 Action Goal 传给 Perception 节点
- Perception 节点保持"纯服务节点"设计，不订阅话题

### 4. 进程模型 ✅

**选择：单进程（Flask 在 Node 内部线程运行）**

- ROS 节点主线程运行 `rclpy.spin()`
- Flask/Socket.IO 在后台 daemon 线程运行
- 通过锁（`threading.Lock`）保护共享数据
- 单个 PID，启动/停止简单

### 5. 模块化设计 ✅

**选择：协调层 + 独立模块**

| 模块 | 职责 |
|------|------|
| `WebInteractiveGui` | 协调层，只做调度 |
| `CameraManager` | 相机数据订阅和缓存 |
| `PerceptionClient` | Action Client 封装 |
| `WebServerManager` | Flask/Socket.IO 服务 |

### 6. 启动脚本 ✅

**选择：pgrep/pkill 方式，不维护 PID 文件**

- 通过进程名 `ros2 run web_interactive_gui` 查找/停止进程
- 监听日志文件中的 `[web_gui] 启动完成` 标记确认启动成功
- 启动超时 60 秒

### 7. 类命名 ✅

- 主类：`WebInteractiveGui`（不加 Node 后缀）
- 文件：`web_gui.py`

---

## 数据流设计

```
┌─────────────────────────────────────────────────────────────────┐
│                    Web Interactive GUI Node                      │
│                                                                  │
│  订阅话题:                                                        │
│    /camera/color/image_raw          ─┐                           │
│    /camera/aligned_depth_to_color   ─┼─► 缓存最新帧              │
│    /camera/color/camera_info        ─┘                           │
│                                                                  │
│  Socket.IO:                                                      │
│    image_update ──────────────────────► 浏览器实时显示            │
│                                                                  │
│  Action Client:                                                  │
│    /perception/action/object_target ──► 分割+点云                │
│    /perception/action/object_record ──► 记录物体                 │
└─────────────────────────────────────────────────────────────────┘
```

---

## 订阅话题

| 话题 | 类型 | QoS | 说明 |
|------|------|-----|------|
| `/camera/color/image_raw` | `sensor_msgs/Image` | BEST_EFFORT | RGB 图像（推流+分割输入） |
| `/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | BEST_EFFORT | 深度图（分割输入） |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | RELIABLE | 相机内参（分割输入） |

---

## Action Client

| Action | 说明 | 输入 | 输出 |
|--------|------|------|------|
| `/perception/action/object_target` | 分割→点云 | 图像+深度+点击坐标 | visualization, cropped_image, point_cloud |
| `/perception/action/object_record` | 记录物体 | cropped_image, label | object_id |

---

## HTTP/WebSocket 接口

### HTTP 路由

| 路由 | 方法 | 功能 | 请求体 | 响应 |
|------|------|------|--------|------|
| `/` | GET | Web UI 页面 | - | HTML |
| `/segment` | POST | 触发分割 | `{x, y}` | `{status, visualization}` |
| `/record` | POST | 记录物体 | `{label?}` | `{status, object_id}` |
| `/cancel` | POST | 取消高亮 | - | `{status}` |
| `/status` | GET | 当前状态 | - | `{has_highlight, processing}` |
| `/healthz` | GET | 健康检查 | - | `{status, ready}` |

### WebSocket 事件

| 事件 | 方向 | 数据 | 说明 |
|------|------|------|------|
| `image_update` | Server→Client | `{image, overlay?, overlay_opacity?}` | 图像帧更新 |
| `segment_result` | Server→Client | `{success, visualization?, error?}` | 分割结果通知 |
| `record_result` | Server→Client | `{success, object_id?, error?}` | 记录结果通知 |

---

## 前端 UI 功能

### 图像显示区
- 实时相机图像流（30 FPS）
- 分割高亮叠加显示
- 点击触发分割（十字光标）

### 功能按钮栏（高亮存在时显示）

| 按钮 | 功能 | 状态 |
|------|------|------|
| **记录** | 保存为新物体 | ✅ 可用 |
| **采样** | 给已有物体增加样本 | 🔲 暂不实现（灰色禁用） |
| **取消** | 清除高亮 | ✅ 可用 |

### 高亮移除时机
- 点击新位置（新高亮替换旧高亮）
- 点击"取消"按钮
- "记录"成功后

### 状态指示
- 连接状态（已连接/断开）
- 处理状态（分割中/记录中）
- 结果反馈（成功/失败提示）

---

## 参数配置

```yaml
web_interactive_gui:
  ros__parameters:
    # ===== Web 服务 =====
    host: "0.0.0.0"
    port: 5000
    update_rate: 30.0                 # 图像推送帧率
    
    # ===== 输入话题 =====
    image_topic: "/camera/color/image_raw"
    depth_topic: "/camera/aligned_depth_to_color/image_raw"
    camera_info_topic: "/camera/color/camera_info"
    
    # ===== Perception Action =====
    perception_action_prefix: "/perception/action"
    action_timeout: 30.0              # Action 超时（秒）
```

---

## 节点结构

```
src/web/                             # ROS 2 包根目录
├── web.md                           # 本文档
├── webInteractiveGUI.sh             # 启动/停止脚本
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── web_interactive_gui          # ament 索引标记
├── scripts/
│   └── web_interactive_gui_node.py  # 入口脚本
└── src/
    └── web_interactive_gui_core/
        ├── __init__.py
        ├── web_gui.py               # WebInteractiveGui（协调层）
        ├── camera_manager.py        # CameraManager（相机数据管理）
        ├── perception_client.py     # PerceptionClient（Action 调用）
        ├── web_server.py            # WebServerManager（Flask/Socket.IO）
        ├── templates/
        │   └── interactive_viewer.html
        └── static/
            └── socket.io.min.js
```

---

## 类接口设计

### 模块依赖关系

```
┌─────────────────────────────────────────────────────────────┐
│                    WebInteractiveGui                        │
│                      （协调层）                              │
│                                                             │
│   ┌─────────────┐  ┌─────────────────┐  ┌───────────────┐  │
│   │CameraManager│  │PerceptionClient │  │WebServerManager│  │
│   │             │  │                 │  │               │  │
│   │ - 图像订阅  │  │ - Action 调用   │  │ - Flask 路由  │  │
│   │ - 深度订阅  │  │ - 结果解析      │  │ - Socket.IO   │  │
│   │ - 内参缓存  │  │                 │  │ - 图像推送    │  │
│   └──────┬──────┘  └────────┬────────┘  └───────┬───────┘  │
│          │                  │                   │          │
│          └──────────────────┼───────────────────┘          │
│                             │                              │
│                      协调调度                               │
└─────────────────────────────────────────────────────────────┘
```

---

### 1. CameraManager（相机数据管理）

**文件**：`src/web_interactive_gui_core/camera_manager.py`

**职责**：订阅并缓存图像、深度图、相机内参

| 方法 | 类型 | 入参 | 出参 | 说明 |
|------|------|------|------|------|
| `__init__` | 同步 | node: Node, config: dict | - | 初始化，创建订阅 |
| `get_image` | 同步 | - | Optional[ndarray] | 获取最新 RGB 图像 |
| `get_depth` | 同步 | - | Optional[ndarray] | 获取最新深度图 |
| `get_camera_info` | 同步 | - | Optional[CameraInfo] | 获取相机内参 |
| `get_image_msg` | 同步 | - | Optional[Image] | 获取 RGB（ROS Image 格式） |
| `get_depth_msg` | 同步 | - | Optional[Image] | 获取深度图（ROS Image 格式） |
| `is_ready` | 同步 | - | tuple[bool, str] | 检查数据是否就绪 |

---

### 2. PerceptionClient（Perception Action 调用）

**文件**：`src/web_interactive_gui_core/perception_client.py`

**职责**：封装 Perception Action 调用

**数据类**：

```python
@dataclass
class SegmentResult:
    success: bool
    error_message: str = ""
    visualization: Optional[np.ndarray] = None
    cropped_image: Optional[Image] = None       # ROS Image，供 record 使用
    center_3d: Optional[tuple] = None
    confidence: float = 0.0

@dataclass
class RecordResult:
    success: bool
    error_message: str = ""
    object_id: str = ""
```

**方法**：

| 方法 | 类型 | 入参 | 出参 | 说明 |
|------|------|------|------|------|
| `__init__` | 同步 | node: Node, config: dict | - | 初始化，创建 Action Client |
| `is_ready` | 同步 | - | bool | 检查 Action Server 是否可用 |
| `segment` | 异步 | color_image, depth_image, camera_info, click_x, click_y | SegmentResult | 调用 object_target Action |
| `record` | 异步 | cropped_image, label, description | RecordResult | 调用 object_record Action |

---

### 3. WebServerManager（Flask/Socket.IO 服务）

**文件**：`src/web_interactive_gui_core/web_server.py`

**职责**：管理 Flask/Socket.IO 服务，提供 HTTP 路由和 WebSocket 事件

| 方法 | 类型 | 入参 | 出参 | 说明 |
|------|------|------|------|------|
| `__init__` | 同步 | config: dict | - | 初始化 Flask 应用 |
| `set_callbacks` | 同步 | on_segment, on_record, on_cancel, get_status, get_health | - | 注入业务回调 |
| `start` | 同步 | - | - | 启动 Flask 后台线程 |
| `emit_image_update` | 同步 | image_b64, overlay_b64?, opacity? | - | 推送图像更新 |
| `emit_segment_result` | 同步 | success, error? | - | 推送分割结果 |
| `emit_record_result` | 同步 | success, object_id?, error? | - | 推送记录结果 |

**回调函数签名**：

```python
on_segment: Callable[[float, float], None]      # (x, y)
on_record: Callable[[str], None]                # (label)
on_cancel: Callable[[], None]
get_status: Callable[[], dict]
get_health: Callable[[], dict]
```

---

### 4. WebInteractiveGui（协调层）

**文件**：`src/web_interactive_gui_core/web_gui.py`

**职责**：ROS 2 节点入口，协调各模块

**状态枚举**：

```python
class State(Enum):
    IDLE = "idle"                   # 空闲，无高亮
    SEGMENTING = "segmenting"       # 分割处理中
    HIGHLIGHTED = "highlighted"     # 有分割结果，显示高亮
    RECORDING = "recording"         # 记录处理中
```

**属性**：

| 属性 | 类型 | 说明 |
|------|------|------|
| `_camera` | CameraManager | 相机数据管理 |
| `_perception` | PerceptionClient | Perception 调用 |
| `_web_server` | WebServerManager | Web 服务 |
| `_state` | State | 当前状态 |
| `_visualization` | Optional[ndarray] | 高亮图缓存 |
| `_cropped_image` | Optional[Image] | 裁剪图缓存 |

**方法**：

| 方法 | 类型 | 说明 |
|------|------|------|
| `__init__` | 同步 | 初始化节点，创建模块，注入回调，启动服务 |
| `_load_parameters` | 同步 | 加载参数配置 |
| `_handle_segment` | 异步 | 处理分割请求 |
| `_handle_record` | 异步 | 处理记录请求 |
| `_handle_cancel` | 同步 | 处理取消请求 |
| `_get_status` | 同步 | 获取当前状态 |
| `_get_health` | 同步 | 获取健康状态 |
| `_broadcast_image` | 同步 | 定时推送图像（Timer 回调） |
| `_clear_cache` | 同步 | 清除分割结果缓存 |

---

### 状态流转图

```
                    ┌──────────────────────┐
                    │                      │
         点击新位置  │                      │  点击取消/记录成功
           ┌────────┴──────────┐           │
           ▼                   │           ▼
        ┌──────┐            ┌──────────────────┐
        │ IDLE │───点击────►│   SEGMENTING     │
        └──────┘            └────────┬─────────┘
           ▲                         │
           │                    分割完成
           │                         │
           │                         ▼
           │                ┌──────────────────┐
           │◄──取消/记录成功─│   HIGHLIGHTED    │
           │                └────────┬─────────┘
           │                         │
           │                    点击记录
           │                         │
           │                         ▼
           │                ┌──────────────────┐
           └──记录失败───────│    RECORDING     │
                            └──────────────────┘
```

---

### 线程模型

```
┌─────────────────────────────────────────────────────────────┐
│                      单进程                                  │
│                                                             │
│  ┌─────────────────────────┐  ┌──────────────────────────┐  │
│  │       主线程            │  │      Flask 线程          │  │
│  │                         │  │                          │  │
│  │  rclpy.spin(node)       │  │  socketio.run()          │  │
│  │    │                    │  │    │                     │  │
│  │    ├─ image_callback    │  │    ├─ POST /segment      │  │
│  │    ├─ depth_callback    │  │    ├─ POST /record       │  │
│  │    ├─ camera_info_cb    │  │    ├─ POST /cancel       │  │
│  │    └─ broadcast_timer   │  │    └─ WebSocket events   │  │
│  │                         │  │                          │  │
│  └───────────┬─────────────┘  └────────────┬─────────────┘  │
│              │                             │                │
│              │    data_lock / state_lock   │                │
│              └──────────────┬──────────────┘                │
│                             │                               │
│                    ┌────────┴────────┐                      │
│                    │   共享数据       │                      │
│                    │  current_image  │                      │
│                    │  current_depth  │                      │
│                    │  visualization  │                      │
│                    │  state          │                      │
│                    └─────────────────┘                      │
└─────────────────────────────────────────────────────────────┘
```

---

## 处理流程

### 1. 初始化

**启动流程**：

```
WebInteractiveGui.__init__()
    │
    ├── 加载参数配置
    │
    ├── 创建模块（同步）
    │   ├── CameraManager     → 创建订阅
    │   ├── PerceptionClient  → 创建 Action Client
    │   └── WebServerManager  → 初始化 Flask
    │
    ├── 注入回调到 WebServerManager
    │
    ├── 启动 Flask 后台线程（异步）
    │   └── WebServerManager.start()
    │
    ├── 等待所有模块就绪
    │   ├── WebServerManager 就绪（Flask 线程启动）
    │   └── PerceptionClient 就绪（Action Server 可用，可选）
    │
    ├── 启动图像推送定时器
    │
    └── 输出启动成功日志
        └── "[web_gui] 启动完成"
```

**启动成功标记**：

节点在所有模块初始化完成后，输出日志：
```
[web_gui] 启动完成
```

启动脚本 `webInteractiveGUI.sh` 监听此标记，确认启动成功。

**各模块就绪条件**：

| 模块 | 就绪条件 | 是否阻塞 |
|------|----------|----------|
| CameraManager | 订阅创建完成 | 同步（立即完成） |
| PerceptionClient | Action Client 创建完成 | 同步（立即完成） |
| WebServerManager | Flask 线程启动 | 异步（等待线程启动） |

**注意**：
- 相机数据（image/depth/camera_info）可能尚未收到，但不阻塞启动
- Perception Action Server 可能尚未运行，但不阻塞启动
- 这些在实际调用时会检查并返回错误

### 2. 图像推流
```
image_callback
    └── 缓存最新图像
        
broadcast_timer (30 Hz)
    ├── 编码图像为 JPEG Base64
    ├── 如有高亮，叠加 visualization
    └── Socket.IO emit('image_update')
```

### 3. 分割流程
```
POST /segment {x, y}
    │
    ├── 获取当前缓存的 image, depth, camera_info
    │
    ├── 调用 object_target Action
    │   Goal: color_image, depth_image, camera_info, click_x, click_y
    │
    ├── 等待 Result
    │   ├── 成功：缓存 visualization, cropped_image
    │   └── 失败：返回错误
    │
    └── 返回 {status, visualization}
```

### 4. 记录流程
```
POST /record {label?}
    │
    ├── 检查是否有 cropped_image 缓存
    │
    ├── 调用 object_record Action
    │   Goal: cropped_image, label
    │
    ├── 等待 Result
    │   ├── 成功：清除高亮，返回 object_id
    │   └── 失败：返回错误
    │
    └── 返回 {status, object_id}
```

---

## 状态管理

### 节点状态

| 状态 | 说明 |
|------|------|
| `idle` | 空闲，无高亮 |
| `segmenting` | 分割处理中 |
| `highlighted` | 有分割结果，显示高亮 |
| `recording` | 记录处理中 |

### 缓存数据

| 数据 | 生命周期 | 用途 |
|------|----------|------|
| `current_image` | 持续更新 | 推流+分割输入 |
| `current_depth` | 持续更新 | 分割输入 |
| `camera_info` | 启动后缓存 | 分割输入 |
| `visualization` | 分割成功→清除 | 高亮显示 |
| `cropped_image` | 分割成功→记录后清除 | 记录输入 |

---

## 参考实现

| 参考 | 路径 | 说明 |
|------|------|------|
| L1 Web GUI | `/home/jetson/L1/l1_stage2_segmentation/l1_stage2_segmentation/visualization/web_gui/segmentation_gui.py` | Flask + Socket.IO 实现参考 |
| L1 分割节点 | `/home/jetson/L1/l1_stage2_segmentation/l1_stage2_segmentation/perception/interactive_segmentation_node.py` | 点击分割交互参考 |

---

## 依赖

### ROS 2 包依赖
- `rclpy`
- `sensor_msgs`
- `geometry_msgs`
- `cv_bridge`
- `perception`（Action 消息定义）

### Python 依赖
- `flask`
- `flask-cors`
- `flask-socketio`
- `opencv-python`
- `numpy`

---

## 启动方式

```bash
# 启动节点
./src/web/webInteractiveGUI.sh start

# 停止节点
./src/web/webInteractiveGUI.sh stop

# 访问 Web UI
http://<jetson-ip>:5000
```

---

## 实现步骤

### 依赖关系图

```
Phase 1 (基础设施)
    │
    ├──► Phase 2 (CameraManager)     ──┐
    │                                  │
    ├──► Phase 3 (WebServerManager)  ──┼──► Phase 5 (WebInteractiveGui)
    │                                  │           │
    └──► Phase 4 (PerceptionClient)  ──┘           ▼
              │                             Phase 6 (前端更新)
              │                                    │
              └─── 依赖 perception 消息定义         ▼
                                            Phase 7 (集成测试)
```

### 可并行执行

| 阶段 | 可并行任务 |
|------|------------|
| Phase 1 完成后 | Phase 2 + Phase 3 可并行 |
| Phase 2 + 3 完成后 | Phase 4 可开始（需 perception 消息） |
| Phase 2 + 3 + 4 完成后 | Phase 5 开始 |
| Phase 5 完成后 | Phase 6 开始 |

---

### Phase 1: 基础设施（无依赖）

| 步骤 | 任务 | 产出文件 | 说明 |
|------|------|----------|------|
| 1.1 | 创建目录结构 | 所有目录 | src/, scripts/, templates/, static/ |
| 1.2 | 创建 package.xml | package.xml | 基础依赖（rclpy, sensor_msgs, cv_bridge） |
| 1.3 | 创建 setup.py | setup.py | Python 包配置 |
| 1.4 | 创建 setup.cfg | setup.cfg | 安装路径配置 |
| 1.5 | 创建 resource 标记 | resource/web_interactive_gui | ament 索引标记 |
| 1.6 | 创建 __init__.py | src/web_interactive_gui_core/__init__.py | 包初始化 |

---

### Phase 2: CameraManager（依赖 Phase 1）

| 步骤 | 任务 | 产出文件 | 说明 |
|------|------|----------|------|
| 2.1 | 创建骨架 | camera_manager.py | 类定义、`__init__` 签名 |
| 2.2 | 实现 `__init__` | camera_manager.py | 创建订阅、初始化锁和缓存 |
| 2.3 | 实现回调函数 | camera_manager.py | `_image_callback`, `_depth_callback`, `_camera_info_callback` |
| 2.4 | 实现 get 方法 | camera_manager.py | `get_image`, `get_depth`, `get_camera_info`, `get_*_msg` |
| 2.5 | 实现 `is_ready` | camera_manager.py | 检查数据是否就绪 |

---

### Phase 3: WebServerManager（依赖 Phase 1）

| 步骤 | 任务 | 产出文件 | 说明 |
|------|------|----------|------|
| 3.1 | 创建骨架 | web_server.py | 类定义、属性声明 |
| 3.2 | 实现 `__init__` | web_server.py | 创建 Flask 应用和 SocketIO |
| 3.3 | 实现 `set_callbacks` | web_server.py | 注入业务回调函数 |
| 3.4 | 实现 HTTP 路由 | web_server.py | `/`, `/segment`, `/record`, `/cancel`, `/status`, `/healthz` |
| 3.5 | 实现 Socket.IO 事件 | web_server.py | `connect`, `disconnect` |
| 3.6 | 实现 emit 方法 | web_server.py | `emit_image_update`, `emit_segment_result`, `emit_record_result` |
| 3.7 | 实现 `start` | web_server.py | 启动 Flask 后台线程 |
| 3.8 | 实现 `is_ready` | web_server.py | 检查 Flask 线程是否启动 |

---

### Phase 4: PerceptionClient（依赖 Phase 1 + perception 消息）

**前置条件**：perception 包的 Action 消息已定义并编译

| 步骤 | 任务 | 产出文件 | 说明 |
|------|------|----------|------|
| 4.1 | 更新 package.xml | package.xml | 添加 `<depend>perception</depend>` |
| 4.2 | 创建骨架 | perception_client.py | 类定义、数据类定义 |
| 4.3 | 实现 `__init__` | perception_client.py | 创建 Action Client |
| 4.4 | 实现 `is_ready` | perception_client.py | 检查 Action Server 是否可用 |
| 4.5 | 实现 `segment` | perception_client.py | 调用 object_target Action |
| 4.6 | 实现 `record` | perception_client.py | 调用 object_record Action |

---

### Phase 5: WebInteractiveGui（依赖 Phase 2, 3, 4）

| 步骤 | 任务 | 产出文件 | 说明 |
|------|------|----------|------|
| 5.1 | 创建骨架 | web_gui.py | 类定义、State 枚举 |
| 5.2 | 实现 `__init__` | web_gui.py | 创建模块、注入回调、启动服务 |
| 5.3 | 实现 `_load_parameters` | web_gui.py | 加载 ROS 参数 |
| 5.4 | 实现 `_handle_segment` | web_gui.py | 处理分割请求 |
| 5.5 | 实现 `_handle_record` | web_gui.py | 处理记录请求 |
| 5.6 | 实现 `_handle_cancel` | web_gui.py | 处理取消请求 |
| 5.7 | 实现 `_get_status` | web_gui.py | 返回当前状态 |
| 5.8 | 实现 `_get_health` | web_gui.py | 返回健康状态 |
| 5.9 | 实现 `_broadcast_image` | web_gui.py | 定时推送图像 |
| 5.10 | 实现 `_clear_cache` | web_gui.py | 清除分割结果缓存 |
| 5.11 | 输出启动成功日志 | web_gui.py | `[web_gui] 启动完成` |
| 5.12 | 创建入口脚本 | scripts/web_interactive_gui_node.py | main() 函数 |

---

### Phase 6: 前端更新（依赖 Phase 5）

| 步骤 | 任务 | 产出文件 | 说明 |
|------|------|----------|------|
| 6.1 | 添加按钮栏 HTML | interactive_viewer.html | 记录/采样/取消按钮 |
| 6.2 | 添加按钮栏样式 | interactive_viewer.html | CSS 样式 |
| 6.3 | 实现点击分割 | interactive_viewer.html | 调用 `/segment` |
| 6.4 | 实现高亮显示 | interactive_viewer.html | 监听 `segment_result` |
| 6.5 | 实现高亮移除 | interactive_viewer.html | 点击新位置/取消/记录成功 |
| 6.6 | 实现记录请求 | interactive_viewer.html | 调用 `/record` |
| 6.7 | 实现状态指示 | interactive_viewer.html | 处理中/成功/失败提示 |
| 6.8 | 禁用采样按钮 | interactive_viewer.html | 灰色禁用状态 |

---

### Phase 7: 集成测试（依赖 Phase 6）

| 步骤 | 任务 | 说明 |
|------|------|------|
| 7.1 | 编译验证 | `colcon build --packages-select web_interactive_gui` |
| 7.2 | 测试启动/停止 | `./webInteractiveGUI.sh start/stop` |
| 7.3 | 测试图像推流 | 启动相机，访问 Web UI 查看图像 |
| 7.4 | 测试分割流程 | 启动 perception 节点，点击图像测试分割 |
| 7.5 | 测试记录流程 | 分割后点击"记录"，验证 object_id 返回 |
| 7.6 | 测试高亮移除 | 验证三种移除时机都正常工作 |

---

### 任务清单汇总

| Phase | 任务数 | 依赖 | 预估 |
|-------|--------|------|------|
| Phase 1 | 6 | 无 | 基础 |
| Phase 2 | 5 | Phase 1 | 简单 |
| Phase 3 | 8 | Phase 1 | 中等 |
| Phase 4 | 6 | Phase 1 + perception | 中等 |
| Phase 5 | 12 | Phase 2,3,4 | 复杂 |
| Phase 6 | 8 | Phase 5 | 中等 |
| Phase 7 | 6 | Phase 6 | 测试 |
| **总计** | **51** | - | - |
