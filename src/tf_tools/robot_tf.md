# Robot TF 节点设计文档

## 概述

`robot_tf` 是一个统一的 TF 管理节点，提供以下功能：
- **TF 发布**：发布静态 TF 和动态 TF
- **TF 转换**：提供坐标转换 Service

## 数据流

```
┌─────────────────────────────────────────────────────────────────────────┐
│                            构建端（配置生成）                             │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  用户输入 ──交互──> static_tf_config_build.sh ──生成──> static_tf_config.yaml
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                            发布端（TF 发布）                             │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  static_tf_config.yaml ──读取──> _load_and_publish_static_tf()          │
│                                           │                             │
│                                           ▼                             │
│                                      /tf_static                         │
│                                                                         │
│  URDF + /joint_states ──计算──> publish_dynamic_transforms()            │
│                                           │                             │
│                                           ▼                             │
│                                        /tf                              │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                            转换端（坐标转换）                            │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  /tf + /tf_static ──订阅──> TransformListener ──存入──> TF Buffer       │
│                                                              │          │
│                                                              ▼          │
│                                        lookup_transform(source, target) │
│                                                              │          │
│                                                              ▼          │
│                                 /tf_tools/transform_points Service      │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

## 文件结构

```
tf_tools/
├── package.xml                    # ROS 2 包定义
├── CMakeLists.txt                 # 编译配置
├── srv/
│   └── TransformPoints.srv        # 坐标转换 Service 定义
├── src/
│   └── robot_tf.py                # 核心节点（发布+转换）
├── start_robot_tf.sh              # 启动脚本
├── static_tf_config_build.sh      # 静态 TF 配置生成工具
├── static_tf_config.yaml          # 静态 TF 配置文件
├── check_environment.sh           # 环境检查工具
├── robot_tf.md                    # 本文档
├── README.md                      # 使用说明
└── IMPLEMENTATION.md              # 实现细节
```

## Service 接口

### /tf_tools/transform_points

将点数组从源坐标系转换到目标坐标系。

**Request**:
```
string source_frame              # 源坐标系 (如 camera_color_optical_frame)
string target_frame              # 目标坐标系 (如 base_link)
float64[] points_in              # 输入点数组 [x1,y1,z1, x2,y2,z2, ...]
```

**Response**:
```
bool success                     # 是否成功
string message                   # 错误信息（失败时）
float64[] points_out             # 转换后的点数组
```

**调用示例**:
```python
from tf_tools.srv import TransformPoints

# 创建客户端
client = node.create_client(TransformPoints, '/tf_tools/transform_points')

# 构造请求
request = TransformPoints.Request()
request.source_frame = 'camera_color_optical_frame'
request.target_frame = 'base_link'
request.points_in = [0.1, 0.2, 0.5, 0.3, 0.4, 0.6]  # 2个点

# 调用
future = client.call_async(request)
response = await future

if response.success:
    # points_out: [x1', y1', z1', x2', y2', z2']
    transformed_points = response.points_out
```

## 节点设计

### 类结构

```python
class RobotTF(Node):
    """统一的 TF 管理节点"""
    
    def __init__(self):
        super().__init__('robot_tf')
        
        # === 发布功能（保留原有） ===
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        
        # 订阅 joint_states
        self.joint_state_sub = self.create_subscription(...)
        
        # 定时发布固定关节 TF
        self.timer = self.create_timer(...)
        
        # === 转换功能（新增） ===
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        
        # 转换 Service
        self.create_service(
            TransformPoints,
            '/tf_tools/transform_points',
            self._transform_points_callback
        )
```

### 保留的发布逻辑

| 方法 | 功能 |
|------|------|
| `_load_and_publish_static_tf()` | 加载 static_tf_config.yaml，发布静态 TF |
| `joint_state_callback()` | 处理 /joint_states，更新关节位置 |
| `publish_dynamic_transforms()` | 发布可动关节的 TF |
| `publish_fixed_transforms()` | 定时发布固定关节的 TF |
| `compute_transform()` | 根据关节类型计算变换 |

### 新增的转换逻辑

| 方法 | 功能 |
|------|------|
| `_transform_points_callback()` | Service 回调，处理转换请求 |
| `_do_transform_points()` | 实际的点云转换计算 |

## 启动脚本

### start_robot_tf.sh

```bash
# 后台启动
./start_robot_tf.sh --start

# 停止节点
./start_robot_tf.sh --stop

# 查看状态
./start_robot_tf.sh --status

# 前台运行（调试）
./start_robot_tf.sh --foreground
```

**参数说明**:

| 参数 | 说明 |
|------|------|
| `--start` | 后台启动节点（默认） |
| `--stop` | 停止后台节点 |
| `--status` | 查看节点运行状态 |
| `--foreground` | 前台运行，Ctrl+C 停止 |
| `--help` | 显示帮助信息 |

**文件位置**:
- PID 文件：`L2/log/tf_tools/robot_tf.pid`
- 日志文件：`L2/log/tf_tools/robot_tf.log`

## 配置生成

使用 `static_tf_config_build.sh` 生成静态 TF 配置：

```bash
./static_tf_config_build.sh
```

交互式输入：
- world → base_link 的平移和旋转
- world → camera_link 的平移和旋转

生成的 `static_tf_config.yaml` 会被 `robot_tf` 节点读取并发布。

## 编译

```bash
cd ~/L2
colcon build --packages-select tf_tools
source install/setup.bash
```

## 验证

### 1. 检查环境

```bash
./src/tf_tools/check_environment.sh
```

### 2. 启动节点

```bash
./src/tf_tools/start_robot_tf.sh --start
```

### 3. 查看 TF

```bash
# 查看 TF 树
ros2 run tf2_tools view_frames

# 查看特定变换
ros2 run tf2_ros tf2_echo camera_color_optical_frame base_link
```

### 4. 测试转换 Service

```bash
ros2 service call /tf_tools/transform_points tf_tools/srv/TransformPoints \
  "{source_frame: 'camera_color_optical_frame', target_frame: 'base_link', points_in: [0.1, 0.2, 0.5]}"
```

## 依赖

- ROS 2 Humble
- tf2_ros
- geometry_msgs
- sensor_msgs
- rclpy

## 迁移说明

### 从旧版本升级

| 旧文件 | 新文件 | 说明 |
|--------|--------|------|
| `robot_tf_publisher.py` | `robot_tf.py` | 重命名 + 扩展 |
| `tf_publisher.sh` | `start_robot_tf.sh` | 重命名 |
| `dynamic_tf_publish.sh` | 删除 | 合并到 start_robot_tf.sh |

### 节点名变化

- 旧：`robot_tf_publisher`
- 新：`robot_tf`

### 新增 Service

- `/tf_tools/transform_points`：坐标转换服务

