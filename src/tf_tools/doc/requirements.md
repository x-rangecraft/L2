# TF Tools 功能需求文档

## 文档信息
- **版本**: v1.0
- **日期**: 2025-11-12
- **状态**: 需求整理阶段

## 1. 项目概述

### 1.1 背景
tf_tools 是 L2 机器人系统中负责 TF（坐标变换）管理的专用包。在 ROS 2 系统中，坐标变换是机器人感知和运动控制的基础，需要一个统一的工具来管理复杂的坐标系关系。

### 1.2 核心目标
本包专注于以下两个核心功能：
1. **TF关系树的生成和发布** - 构建和维护机器人系统的坐标系层级关系
2. **TF变换的生成和发布** - 管理静态和动态坐标变换的发布

### 1.3 与现有系统的关系

#### 与 description 包的职责划分
| 功能 | description 包 | tf_tools 包 |
|------|---------------|-------------|
| 发布 robot_description | ✓ | - |
| 发布机械臂关节TF | ✓ (robot_state_publisher) | - |
| 发布 world → base_link | - | ✓ |
| 发布传感器静态TF | - | ✓ |
| 发布多机器人全局TF | - | ✓ |
| TF树验证与诊断 | - | ✓ |
| URDF模型管理 | ✓ | - |

**职责说明**：
- **description 包**：负责机器人本体的描述和关节变换
  - 发布 `/robot_description` 话题（URDF内容）
  - 通过 robot_state_publisher 根据 joint_states 发布机械臂关节的动态TF

- **tf_tools 包**：负责机器人外围和系统级的坐标变换
  - 发布机器人在世界坐标系中的位置（world → base_link）
  - 发布传感器、工具等外围设备的静态TF
  - 管理多机器人场景下的全局坐标关系
  - 提供TF系统的验证和诊断工具

## 2. 功能需求

### 2.1 静态TF管理

#### 2.1.1 批量静态变换发布
**需求描述**：
在机器人系统中，通常有多个静态坐标变换需要发布（如传感器安装位置、工具坐标系等）。传统方式需要启动多个 `static_transform_publisher` 节点，管理复杂且易出错。

**功能要求**：
- 从配置文件批量加载静态变换定义
- 统一发布到 `/tf_static` 话题
- 支持平移（x, y, z）和旋转（roll, pitch, yaw 或 四元数）
- 支持注释和分组，便于管理

**典型应用场景**：
```
world → base_link           # 机器人基座在世界坐标系的位置
base_link → camera_mount    # 相机安装支架
camera_mount → camera_link  # 相机坐标系
base_link → lidar_link      # 激光雷达坐标系
```

#### 2.1.2 配置热重载
**需求描述**：
在开发和调试阶段，经常需要调整坐标变换参数。支持热重载可以避免重启节点。

**功能要求**：
- 提供服务接口 `~/reload_config`
- 重新加载配置文件并更新发布的变换
- 验证配置有效性，无效则保持原有配置
- 记录重载操作日志

#### 2.1.3 多机器人支持
**需求描述**：
支持多个机器人在同一系统中运行，避免坐标系名称冲突。

**功能要求**：
- 支持 `frame_prefix` 参数，为所有坐标系名称添加前缀
  - 示例：`frame_prefix: "arm1_"` → `arm1_base_link`
- 支持 `namespace` 参数，隔离话题和节点
  - 示例：`namespace: "/robot1"` → `/robot1/tf_static`
- 配置文件中支持变量替换
- 文档提供多机器人配置示例

### 2.2 动态TF管理

#### 2.2.1 基于话题的动态变换
**需求描述**：
某些坐标变换需要根据外部数据动态更新，如定位系统输出的 map → odom 变换。

**功能要求**：
- 订阅指定话题（geometry_msgs/PoseStamped, TransformStamped 等）
- 转换为TF并发布到 `/tf` 话题
- 支持配置更新频率
- 支持时间戳同步和验证
- 处理数据丢失和延迟情况

**配置示例**：
```yaml
dynamic_transforms:
  - name: "map_to_odom"
    parent_frame: map
    child_frame: odom
    source_topic: "/localization/pose"
    source_type: "geometry_msgs/PoseStamped"
    update_rate: 50.0  # Hz
```

#### 2.2.2 可编程变换接口
**需求描述**：
为高级用户提供编程接口，支持自定义变换逻辑。

**功能要求**：
- 提供 Python API 用于发布自定义变换
- 支持变换的组合和计算
- 提供时间戳管理工具
- 文档包含代码示例

### 2.3 TF树验证与诊断

#### 2.3.1 TF树可视化
**需求描述**：
提供类似 `view_frames` 的工具，查看当前系统的完整TF树结构。

**功能要求**：
- 命令行工具：`ros2 run tf_tools view_tf_tree`
- 输出格式：文本树状图、JSON、DOT（Graphviz）
- 显示变换类型（静态/动态）
- 显示发布者信息
- 可选输出到文件

**输出示例**（文本格式）：
```
world
  └── base_link (static, from: static_tf_publisher)
      ├── link_1 (dynamic, from: robot_state_publisher)
      ├── link_2 (dynamic, from: robot_state_publisher)
      └── camera_link (static, from: static_tf_publisher)
```

#### 2.3.2 TF链完整性检查
**需求描述**：
验证从源坐标系到目标坐标系的变换链是否完整可达。

**功能要求**：
- 命令行工具：`ros2 run tf_tools check_tf_chain --from world --to gripper_base`
- 检查变换链是否存在断裂
- 输出变换链路径
- 报告丢失的变换
- 退出码指示检查结果（0=成功，1=失败）

#### 2.3.3 TF健康度监控
**需求描述**：
持续监控TF系统的健康状态，检测异常情况。

**功能要求**：
- 检测项：
  - 变换时间戳过旧（超过阈值）
  - 变换发布频率异常（过低或停止）
  - 坐标系孤立（无父节点）
  - 坐标系命名冲突
- 输出健康报告（日志和话题）
- 可配置检查间隔和阈值
- 提供ROS 2服务接口查询健康状态

### 2.4 配置管理

#### 2.4.1 配置文件格式
**需求描述**：
使用YAML格式的配置文件管理所有TF相关参数。

**功能要求**：
- 支持注释和分组
- 支持变量替换（环境变量、launch参数）
- 提供配置文件验证工具
- 提供配置模板和示例

**配置文件位置**：
- 默认：`config/tf_config.yaml`
- 可通过参数指定：`config_file:=/path/to/custom.yaml`

#### 2.4.2 参数传递方式
**支持的参数传递方式**：
1. **配置文件** - 主要配置方式
2. **Launch参数** - 覆盖配置文件中的值
3. **环境变量** - 设置默认值
4. **命令行参数** - 快速调试

**参数优先级**：命令行 > Launch > 配置文件 > 环境变量 > 默认值

## 3. 节点设计

### 3.1 静态TF发布节点

**节点名称**：`static_tf_publisher_node`

**功能**：从配置文件加载并发布静态坐标变换。

**接口定义**：

| 类型 | 名称 | 消息/服务类型 | 说明 |
|------|------|--------------|------|
| 发布 | `/tf_static` | tf2_msgs/TFMessage | 静态变换发布 |
| 服务 | `~/reload_config` | std_srvs/Trigger | 重新加载配置 |
| 服务 | `~/add_transform` | 自定义srv | 动态添加静态变换 |
| 参数 | `config_file` | string | 配置文件路径 |
| 参数 | `frame_prefix` | string | 坐标系前缀 |
| 参数 | `publish_rate` | double | 发布频率（0=静态） |

**启动命令示例**：
```bash
ros2 run tf_tools static_tf_publisher_node \
  --ros-args \
  -p config_file:=config/tf_config.yaml \
  -p frame_prefix:=arm1_
```

### 3.2 动态TF发布节点

**节点名称**：`dynamic_tf_publisher_node`

**功能**：订阅外部话题并发布动态坐标变换。

**接口定义**：

| 类型 | 名称 | 消息/服务类型 | 说明 |
|------|------|--------------|------|
| 订阅 | 配置指定 | geometry_msgs/* | 动态变换数据源 |
| 发布 | `/tf` | tf2_msgs/TFMessage | 动态变换发布 |
| 服务 | `~/reload_config` | std_srvs/Trigger | 重新加载配置 |
| 参数 | `config_file` | string | 配置文件路径 |
| 参数 | `frame_prefix` | string | 坐标系前缀 |

### 3.3 TF验证节点

**节点名称**：`tf_validator_node`

**功能**：监控和验证TF系统的健康状态。

**接口定义**：

| 类型 | 名称 | 消息/服务类型 | 说明 |
|------|------|--------------|------|
| 订阅 | `/tf` | tf2_msgs/TFMessage | 动态变换监控 |
| 订阅 | `/tf_static` | tf2_msgs/TFMessage | 静态变换监控 |
| 发布 | `~/tf_health` | 自定义msg | TF健康状态 |
| 服务 | `~/get_tf_tree` | 自定义srv | 获取TF树结构 |
| 服务 | `~/check_transform` | 自定义srv | 检查变换链 |
| 参数 | `check_interval` | double | 检查间隔（秒） |
| 参数 | `max_transform_age` | double | 最大延迟（秒） |

## 4. 配置文件规范

### 4.1 配置文件结构

配置文件位置：`config/tf_config.yaml`

完整配置示例见：`config/tf_config.yaml`（参考模板）

### 4.2 配置项说明

#### 全局配置
```yaml
tf_config:
  frame_prefix: ""           # 坐标系前缀，多机器人场景使用
  enable_validation: true    # 是否启用配置验证
```

#### 静态变换配置
```yaml
  static_transforms:
    - parent_frame: world              # 父坐标系
      child_frame: base_link           # 子坐标系
      translation: [0.0, 0.0, 0.0]     # 平移 [x, y, z] (米)
      rotation: [0.0, 0.0, 0.0]        # 旋转 [roll, pitch, yaw] (弧度)
      # 或使用四元数
      # rotation_quaternion: [x, y, z, w]
```

#### 动态变换配置
```yaml
  dynamic_transforms:
    - name: "map_to_odom"                      # 变换名称（用于日志）
      parent_frame: map                         # 父坐标系
      child_frame: odom                         # 子坐标系
      source_topic: "/localization/pose"        # 数据源话题
      source_type: "geometry_msgs/PoseStamped"  # 消息类型
      update_rate: 50.0                         # 发布频率 (Hz)
```

#### 发布器配置
```yaml
  publisher:
    static_publish_rate: 1.0  # 静态变换发布频率（0=使用static发布器）
    qos_depth: 1              # QoS队列深度
```

#### 验证配置
```yaml
  validation:
    enable_health_check: true      # 启用健康检查
    check_interval: 5.0            # 检查间隔（秒）
    max_transform_age: 1.0         # 最大变换延迟（秒）
    required_transforms:           # 必需的变换链（可选）
      - from: world
        to: gripper_base
```

### 4.3 变量替换

配置文件支持以下变量替换：

**环境变量**：
```yaml
frame_prefix: "${ROBOT_NAME}_"
config_file: "${HOME}/tf_config.yaml"
```

**Launch参数**：
```yaml
frame_prefix: "${frame_prefix}"  # 从launch传入
```

## 5. 工具集

### 5.1 命令行工具

所有工具通过 `ros2 run tf_tools <tool_name>` 调用。

#### view_tf_tree
**功能**：可视化当前TF树结构

**用法**：
```bash
# 输出到终端（文本格式）
ros2 run tf_tools view_tf_tree

# 输出为JSON
ros2 run tf_tools view_tf_tree --format json --output tf_tree.json

# 输出为DOT（Graphviz）
ros2 run tf_tools view_tf_tree --format dot --output tf_tree.dot
dot -Tpdf tf_tree.dot -o tf_tree.pdf
```

#### check_tf_chain
**功能**：检查特定变换链的完整性

**用法**：
```bash
# 检查从world到gripper_base的变换链
ros2 run tf_tools check_tf_chain --from world --to gripper_base

# 详细模式（显示完整路径）
ros2 run tf_tools check_tf_chain --from world --to gripper_base --verbose
```

#### check_tf_health
**功能**：诊断TF系统健康状态

**用法**：
```bash
# 执行一次健康检查
ros2 run tf_tools check_tf_health

# 持续监控（每5秒检查一次）
ros2 run tf_tools check_tf_health --continuous --interval 5.0

# 输出报告到文件
ros2 run tf_tools check_tf_health --output health_report.json
```

#### export_tf_tree
**功能**：导出TF树结构到文件

**用法**：
```bash
# 导出为JSON
ros2 run tf_tools export_tf_tree --format json --output tf_tree.json

# 导出为YAML
ros2 run tf_tools export_tf_tree --format yaml --output tf_tree.yaml
```

### 5.2 Python API

提供 Python 模块供高级用户编程使用。

**导入**：
```python
from tf_tools import TFTreeManager, TFValidator, TFPublisher
```

**使用示例**：
```python
# 管理TF配置
manager = TFTreeManager(config_file='config/tf_config.yaml')
manager.load_config()
manager.publish_static_transforms()

# 验证TF健康度
validator = TFValidator(node)
health_status = validator.check_health()
tf_tree = validator.get_tf_tree()

# 动态发布变换
publisher = TFPublisher(node)
publisher.publish_transform(
    parent_frame='world',
    child_frame='base_link',
    translation=[0.0, 0.0, 0.0],
    rotation=[0.0, 0.0, 0.0]
)
```

## 6. 启动方式

### 6.1 使用启动脚本

**脚本名称**：`start_tf_tools.sh`

**基础用法**：
```bash
# 使用默认配置
./src/tf_tools/start_tf_tools.sh

# 指定配置文件
./src/tf_tools/start_tf_tools.sh --config config/custom_tf.yaml

# 多机器人场景
./src/tf_tools/start_tf_tools.sh --frame-prefix arm1_ --namespace /robot1

# 启用验证节点
./src/tf_tools/start_tf_tools.sh --enable-validation
```

**支持的参数**：
```
--config FILE           指定配置文件路径
--frame-prefix PREFIX   设置坐标系前缀
--namespace NS          设置命名空间
--enable-validation     启用TF验证节点
--skip-build            跳过构建步骤
--help                  显示帮助信息
```

### 6.2 使用Launch文件

**Launch文件**：`launch/tf_tools.launch.py`

**基础用法**：
```bash
# 使用默认配置
ros2 launch tf_tools tf_tools.launch.py

# 指定参数
ros2 launch tf_tools tf_tools.launch.py \
  tf_config_file:=config/custom_tf.yaml \
  frame_prefix:=arm1_ \
  enable_validation:=true
```

### 6.3 直接启动节点

**启动静态TF发布器**：
```bash
ros2 run tf_tools static_tf_publisher_node \
  --ros-args \
  -p config_file:=config/tf_config.yaml \
  -p frame_prefix:=""
```

**启动动态TF发布器**：
```bash
ros2 run tf_tools dynamic_tf_publisher_node \
  --ros-args \
  -p config_file:=config/tf_config.yaml
```

**启动验证节点**：
```bash
ros2 run tf_tools tf_validator_node \
  --ros-args \
  -p check_interval:=5.0 \
  -p max_transform_age:=1.0
```

## 7. 实现优先级

### P0 - 核心功能（首先实现）
- [ ] 静态TF发布节点
- [ ] YAML配置文件解析
- [ ] 基础launch文件
- [ ] 启动脚本

### P1 - 重要功能
- [ ] 动态TF发布节点
- [ ] frame_prefix 支持
- [ ] 配置热重载
- [ ] view_tf_tree 工具

### P2 - 增强功能
- [ ] TF健康检查节点
- [ ] check_tf_chain 工具
- [ ] check_tf_health 工具
- [ ] Python API

### P3 - 高级功能
- [ ] TF树导出工具
- [ ] 高级验证功能
- [ ] 性能优化
- [ ] 完整文档和示例

## 8. 使用场景示例

### 8.1 单机械臂 + 相机场景

**配置文件**：
```yaml
static_transforms:
  - parent_frame: world
    child_frame: base_link
    translation: [0.0, 0.0, 0.0]
    rotation: [0.0, 0.0, 0.0]

  - parent_frame: base_link
    child_frame: camera_link
    translation: [0.0, 0.0, 0.5]
    rotation: [0.0, 0.0, 0.0]
```

**启动**：
```bash
./src/tf_tools/start_tf_tools.sh --config config/single_arm.yaml
```

### 8.2 多机械臂场景

**配置文件模板** `config/multi_arm_template.yaml`：
```yaml
frame_prefix: "${ARM_PREFIX}"

static_transforms:
  - parent_frame: world
    child_frame: "${frame_prefix}base_link"
    translation: [0.0, 0.0, 0.0]
    rotation: [0.0, 0.0, 0.0]
```

**启动机械臂1**：
```bash
./src/tf_tools/start_tf_tools.sh \
  --config config/multi_arm_template.yaml \
  --frame-prefix arm1_ \
  --namespace /robot1
```

**启动机械臂2**：
```bash
./src/tf_tools/start_tf_tools.sh \
  --config config/multi_arm_template.yaml \
  --frame-prefix arm2_ \
  --namespace /robot2
```

### 8.3 移动机器人 + 定位场景

**配置文件**：
```yaml
static_transforms:
  - parent_frame: base_link
    child_frame: laser
    translation: [0.1, 0.0, 0.2]
    rotation: [0.0, 0.0, 0.0]

dynamic_transforms:
  - name: "localization"
    parent_frame: map
    child_frame: odom
    source_topic: "/amcl_pose"
    source_type: "geometry_msgs/PoseStamped"
    update_rate: 50.0
```

## 9. 测试需求

### 9.1 单元测试
- 配置文件解析正确性
- 坐标变换计算准确性
- 参数替换功能
- frame_prefix 应用

### 9.2 集成测试
- 与 robot_state_publisher 协同工作
- 多节点TF发布无冲突
- TF链完整性验证
- 热重载功能

### 9.3 性能测试
- 大量静态变换发布性能
- 高频动态变换发布延迟
- TF查询响应时间
- 内存占用

## 10. 开发规范

### 10.1 代码风格
- 遵循 ROS 2 Python 风格指南
- 使用类型注解
- 编写完整的 docstring
- 代码注释使用中文

### 10.2 日志规范
- 使用 ROS 2 logger（避免 print）
- 日志文件输出到 `log/tf_tools.log`
- 日志级别：DEBUG / INFO / WARN / ERROR / FATAL
- 关键操作记录日志（配置加载、重载、异常等）

### 10.3 错误处理
- 配置文件不存在：清晰错误提示，给出示例路径
- 配置格式错误：指出具体行号和错误内容
- TF链断裂：记录详细信息，发出警告但不中断运行
- 参数错误：启动时验证，错误则拒绝启动

### 10.4 命名规范
- 节点名称：使用下划线命名，如 `static_tf_publisher_node`
- 话题名称：遵循 ROS 2 规范，如 `/tf`, `/tf_static`
- 参数名称：使用下划线命名，如 `config_file`, `frame_prefix`
- 文件名称：小写+下划线，如 `tf_config.yaml`

## 11. 文档需求

### 11.1 用户文档
- [x] 功能需求文档（本文档）
- [ ] 快速入门指南
- [ ] 配置文件格式详解
- [ ] 命令行工具使用手册
- [ ] 常见问题FAQ

### 11.2 开发文档
- [ ] 架构设计文档
- [ ] API参考文档
- [ ] 扩展开发指南
- [ ] 测试指南

### 11.3 示例文档
- [ ] 基础使用示例
- [ ] 多机器人配置示例
- [ ] 与MoveIt集成示例
- [ ] 自定义变换发布示例

## 12. 依赖管理

### 12.1 ROS 2 依赖
**运行时依赖**：
- `rclpy` - Python ROS 2 客户端
- `tf2_ros` - TF2 库
- `geometry_msgs` - 几何消息类型
- `std_srvs` - 标准服务类型

**可选依赖**：
- `rosidl_default_generators` - 自定义消息/服务（如需要）

### 12.2 Python 依赖
- `pyyaml` - YAML解析
- `numpy` - 数值计算（坐标变换）
- `transforms3d` - 坐标变换计算（可选）

### 12.3 系统依赖
- Python >= 3.8
- ROS 2 Humble 或更高版本

## 13. 后续扩展方向

### 13.1 短期扩展（3个月内）
1. 与SLAM/定位系统的深度集成
2. TF录制与回放功能
3. Web界面的TF树可视化
4. 更丰富的验证规则

### 13.2 中期扩展（6个月内）
1. TF性能分析工具
2. 自动化测试框架
3. 与仿真系统集成
4. 多传感器标定工具链

### 13.3 长期规划
1. 分布式TF管理
2. TF时间同步优化
3. 机器学习辅助的TF异常检测
4. 跨平台支持（Windows, MacOS）

## 14. 注意事项

### 14.1 坐标系命名规范
遵循 REP-105（机器人坐标系规范）：
- `world` - 世界坐标系（全局固定）
- `map` - 地图坐标系（SLAM/定位输出）
- `odom` - 里程计坐标系（连续但有漂移）
- `base_link` - 机器人基座坐标系
- 传感器坐标系：`camera_link`, `lidar_link` 等

### 14.2 时间戳管理
- 静态TF：时间戳设为0（表示永久有效）
- 动态TF：使用当前时间或数据源时间戳
- 避免未来时间戳（会导致外推错误）
- 注意时钟源同步（使用ROS 2 时间API）

### 14.3 性能优化
- 静态TF使用 `/tf_static` 话题（发布一次即可）
- 动态TF根据需求设置合理的发布频率
- 避免发布冗余变换
- 大量变换时考虑批量发布

### 14.4 多机器人场景
- 确保 frame_prefix 和 namespace 正确配置
- 避免不同机器人的坐标系名称冲突
- 考虑全局坐标系的统一管理
- 文档提供清晰的配置模板

### 14.5 与 robot_state_publisher 的协同
- 避免重复发布相同的TF（如 base_link 的子坐标系）
- tf_tools 负责 world → base_link
- robot_state_publisher 负责 base_link 及其子坐标系（机械臂链）
- 明确职责划分，避免冲突

## 15. 参考资源

### 15.1 ROS 2 官方文档
- [tf2 Documentation](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html)
- [REP-105: Coordinate Frames for Mobile Platforms](https://www.ros.org/reps/rep-0105.html)
- [robot_state_publisher](https://github.com/ros/robot_state_publisher)

### 15.2 相关工具
- `static_transform_publisher` - 静态变换发布工具
- `view_frames` - TF树可视化工具（ROS 1）
- `tf2_echo` - TF变换查询工具

### 15.3 示例项目
- [navigation2](https://github.com/ros-planning/navigation2) - 导航框架的TF使用
- [moveit2](https://github.com/ros-planning/moveit2) - 运动规划框架的TF使用

---

**文档维护者**：L2 开发团队
**最后更新**：2025-11-12
**状态**：需求整理完成，待评审
