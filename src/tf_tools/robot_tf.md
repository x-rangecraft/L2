# Robot TF - 统一 TF 管理节点

## 概述

`robot_tf` 是一个统一的 ROS 2 TF 管理节点，整合了静态 TF 配置与发布、动态 TF 发布、以及坐标转换服务三大功能。

### 核心功能

| 功能 | 说明 |
|------|------|
| **静态 TF 发布** | 发布固定变换：`world→base_link`、`world→camera_link` |
| **动态 TF 发布** | 订阅 `/joint_states`，实时发布机器人关节变换 |
| **坐标转换服务** | 提供 `/tf_tools/transform_points` Service |

---

## 架构与数据流

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              构建端（配置生成）                              │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  用户输入 ──交互──> static_tf_config_build.sh ──生成──> static_tf_config.yaml
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
                                      │
                                      ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                              发布端（TF 发布）                               │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  static_tf_config.yaml ──读取──> _load_and_publish_static_tf()              │
│                                           │                                 │
│                                           ▼                                 │
│                                      /tf_static                             │
│                                                                             │
│  URDF + /joint_states ──计算──> publish_dynamic_transforms()                │
│                                           │                                 │
│                                           ▼                                 │
│                                        /tf                                  │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
                                      │
                                      ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                              转换端（坐标转换）                              │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  /tf + /tf_static ──订阅──> TransformListener ──存入──> TF Buffer           │
│                                                              │              │
│                                                              ▼              │
│                                        lookup_transform(source, target)     │
│                                                              │              │
│                                                              ▼              │
│                                 /tf_tools/transform_points Service          │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 目录结构

```
tf_tools/
├── package.xml                    # ROS 2 包定义
├── CMakeLists.txt                 # 编译配置
├── src/
│   └── robot_tf.py                # 核心节点（发布 + 转换）
├── srv/
│   └── TransformPoints.srv        # 坐标转换 Service 定义
├── start_robot_tf.sh              # 节点启动脚本
├── static_tf_config_build.sh      # 静态 TF 配置生成工具
├── static_tf_config.yaml          # 静态 TF 配置文件
├── check_environment.sh           # 环境检查工具
├── robot_tf.md                    # 本文档
├── README.md                      # 快速参考
└── IMPLEMENTATION.md              # 实现细节
```

### 运行时目录

- **PID 文件**：`L2/log/tf_tools/robot_tf.pid`
- **日志文件**：`L2/log/tf_tools/robot_tf.log`

---

## 使用方法

> 💡 以下命令都可在仓库任意目录执行，脚本会自动定位自身路径。

### 1. 生成静态 TF 配置

```bash
./src/tf_tools/static_tf_config_build.sh
```

按提示输入或保留默认值：
- `world → base_link` 的平移 (x, y, z) 和旋转
- `world → camera_link` 的平移 (x, y, z) 和旋转
- 单位：平移（米），角度（度）

脚本会生成/更新 `static_tf_config.yaml`，包含：
- 平移、世界/自身轴旋转、变换顺序
- 正/逆四元数及逆向平移
- 从 `src/description/urdf/yam.urdf` 解析的 URDF 关节链

### 2. 启动节点

```bash
# 后台启动（推荐）
./src/tf_tools/start_robot_tf.sh --start

# 前台运行（调试用）
./src/tf_tools/start_robot_tf.sh --foreground

# 查看状态
./src/tf_tools/start_robot_tf.sh --status

# 停止节点
./src/tf_tools/start_robot_tf.sh --stop

# 显示帮助
./src/tf_tools/start_robot_tf.sh --help
```

### 3. 检查环境

```bash
./src/tf_tools/check_environment.sh
```

---

## Service 接口

### /tf_tools/transform_points

将点列表从源坐标系转换到目标坐标系，并可指定采样时间。

**Request**:
```
string source_frame              # 源坐标系 (如 camera_color_optical_frame)
string target_frame              # 目标坐标系 (如 base_link)
builtin_interfaces/Time stamp    # 采样时间（0/0 为“最新”）
geometry_msgs/Point[] points_in  # 输入点列表
```

**Response**:
```
bool success                     # 是否成功
string message                   # 错误信息（失败时）
geometry_msgs/Point[] points_out # 转换后的点列表
```

**命令行调用示例**:
```bash
ros2 service call /tf_tools/transform_points tf_tools/srv/TransformPoints \
  "{source_frame: 'camera_color_optical_frame', target_frame: 'base_link', stamp: {sec: 0, nanosec: 0}, points_in: [{x: 0.1, y: 0.2, z: 0.5}, {x: 0.3, y: 0.4, z: 0.6}]}"
```

**Python 调用示例**:
```python
from geometry_msgs.msg import Point
from tf_tools.srv import TransformPoints

# 创建客户端
client = node.create_client(TransformPoints, '/tf_tools/transform_points')

# 构造请求
request = TransformPoints.Request()
request.source_frame = 'camera_color_optical_frame'
request.target_frame = 'base_link'
request.stamp.sec = 1_700_000_000
request.stamp.nanosec = 123_000_000
point1 = Point(x=0.1, y=0.2, z=0.5)
point2 = Point(x=0.3, y=0.4, z=0.6)
request.points_in = [point1, point2]

# 调用
future = client.call_async(request)
response = await future

if response.success:
    transformed_points = response.points_out  # list[geometry_msgs.msg.Point]
```

---

## 节点设计

### 类结构

```python
class RobotTF(Node):
    """统一的 TF 管理节点"""
    
    def __init__(self):
        super().__init__('robot_tf')
        
        # === 发布功能 ===
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.joint_state_sub = self.create_subscription(...)  # 订阅 joint_states
        self.timer = self.create_timer(...)                   # 定时发布固定关节 TF
        
        # === 转换功能 ===
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.create_service(TransformPoints, '/tf_tools/transform_points', ...)
```

### 核心方法

| 方法 | 功能 |
|------|------|
| `_load_and_publish_static_tf()` | 加载 static_tf_config.yaml，发布静态 TF |
| `joint_state_callback()` | 处理 /joint_states，更新关节位置 |
| `publish_dynamic_transforms()` | 发布可动关节的 TF |
| `publish_fixed_transforms()` | 定时发布固定关节的 TF |
| `compute_transform()` | 根据关节类型计算变换 |
| `_transform_points_callback()` | Service 回调，处理转换请求 |
| `_do_transform_points()` | 实际的点云转换计算 |

### 发布的变换

**静态变换** (`/tf_static`):
- `world → base_link`
- `world → camera_link`

**动态变换** (`/tf`):
- `base_link → link_1` (根据 joint1 角度)
- `link_1 → link_2` (根据 joint2 角度)
- `link_2 → link_3` (根据 joint3 角度)
- `link_3 → link_4` (根据 joint4 角度)
- `link_4 → link_5` (根据 joint5 角度)
- `link_5 → link_6` (根据 joint6 角度)

---

## 编译

```bash
cd ~/L2
colcon build --packages-select tf_tools
source install/setup.bash
```

---

## 验证与调试

### 1. 查看 TF 树

```bash
# 生成 TF 树图
ros2 run tf2_tools view_frames

# 查看特定变换
ros2 run tf2_ros tf2_echo base_link link_6
ros2 run tf2_ros tf2_echo camera_color_optical_frame base_link
```

### 2. 查看话题

```bash
# 查看 TF 话题
ros2 topic echo /tf

# 查看静态 TF
ros2 topic echo /tf_static

# 查看 joint_states
ros2 topic echo /joint_states
```

### 3. 测试转换服务

```bash
ros2 service call /tf_tools/transform_points tf_tools/srv/TransformPoints \
  "{source_frame: 'camera_color_optical_frame', target_frame: 'base_link', stamp: {sec: 0, nanosec: 0}, points_in: [{x: 0.1, y: 0.2, z: 0.5}]}"
```

### 4. 查看日志

```bash
tail -f L2/log/tf_tools/robot_tf.log
```

### 5. 在 Foxglove 中验证

1. 打开 Foxglove Studio
2. 连接到 ROS 2
3. 添加 3D 面板
4. 查看 TF 树和机器人模型
5. 移动机器人关节，观察模型是否同步运动

---

## 配置说明

### static_tf_config.yaml 结构

```yaml
metadata:
  generated_at: "..."
  translation_units: "m"
  rotation_units: "deg"
  urdf_source: ".../yam.urdf"

tf_tree:
  - {parent: "world", child: "base_link"}
  - {parent: "world", child: "camera_link"}

transforms:
  world_to_base_link:
    parent_frame: "world"
    child_frame: "base_link"
    translation_m: {x: ..., y: ..., z: ...}
    quaternion:
      forward_parent_to_child: {x: ..., y: ..., z: ..., w: ...}

urdf_chain:
  links:
    - {joint: "joint1", parent: "base_link", child: "link_1", ...}
    - ...
```

### 节点参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `urdf_path` | 必填 | URDF 文件路径 |
| `publish_rate` | 50.0 | TF 发布频率 (Hz) |
| `base_frame` | base_link | 基座坐标系名称 |
| `static_tf_config` | - | 静态 TF 配置文件路径 |

---

## 依赖

- ROS 2 Humble
- tf2_ros
- geometry_msgs
- sensor_msgs
- rclpy
- numpy

---

## 常见问题

### Q1: Foxglove 中机器人模型不动

**检查步骤**:
1. 确认节点正在运行：`./src/tf_tools/start_robot_tf.sh --status`
2. 确认 `/joint_states` 有数据：`ros2 topic echo /joint_states --once`
3. 确认 TF 正在发布：`ros2 topic hz /tf`

### Q2: 启动失败，提示找不到 URDF

**解决方案**:
```bash
ls -l /home/jetson/L2/src/description/urdf/yam.urdf
```
确保 URDF 文件存在且路径正确。

### Q3: 关节名称不匹配

URDF 中的关节名称必须与 `/joint_states` 中的名称一致：
```bash
# 检查 URDF 关节名
grep '<joint name=' src/description/urdf/yam.urdf

# 检查 joint_states 关节名
ros2 topic echo /joint_states --once | grep name
```

### Q4: 找不到 ros2 命令

启动前必须 source ROS 2 环境：
```bash
source /opt/ros/humble/setup.bash
```

### Q5: 缺少共享库 (如 librcl_action.so)

按错误提示修复 ROS 环境，然后重启节点。

---

## 参考资料

- ROS 2 TF2 教程: https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html
- URDF 规范: http://wiki.ros.org/urdf/XML
