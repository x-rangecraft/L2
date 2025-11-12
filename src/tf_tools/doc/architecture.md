# TF Tools 架构设计文档

## 文档信息
- **版本**: v1.0
- **日期**: 2025-11-12
- **状态**: 架构设计阶段

## 1. 架构概述

### 1.1 设计原则

tf_tools 包的架构设计遵循以下核心原则：

1. **职责分离**：静态TF、动态TF、验证功能独立实现
2. **配置驱动**：通过外部配置文件管理所有参数，避免硬编码
3. **可扩展性**：模块化设计，便于添加新功能
4. **高性能**：优化TF发布频率和内存使用
5. **可靠性**：完善的错误处理和健康检查机制

### 1.2 系统边界

**tf_tools 负责的功能**：
- 静态坐标变换的批量发布
- 动态坐标变换的订阅转发
- TF树结构的验证和诊断
- 多机器人场景的坐标系管理

**tf_tools 不负责的功能**：
- 机械臂关节的动态TF（由 robot_state_publisher 负责）
- URDF模型的管理（由 description 包负责）
- 传感器数据的处理（由各传感器驱动负责）
- 运动学计算（由 MoveIt 等框架负责）

## 2. 整体架构

### 2.1 系统架构图

```
┌─────────────────────────────────────────────────────────────────────┐
│                           tf_tools 包                                │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐      │
│  │  Static TF      │  │  Dynamic TF     │  │  TF Validator   │      │
│  │  Publisher      │  │  Publisher      │  │                 │      │
│  │                 │  │                 │  │                 │      │
│  │ ┌─────────────┐ │  │ ┌─────────────┐ │  │ ┌─────────────┐ │      │
│  │ │Config Loader│ │  │ │Topic Subscriber││  │ │Health Check │ │      │
│  │ └─────────────┘ │  │ └─────────────┘ │  │ └─────────────┘ │      │
│  │ ┌─────────────┐ │  │ ┌─────────────┐ │  │ ┌─────────────┐ │      │
│  │ │TF Generator │ │  │ │Transform Converter││ │Tree Analyzer│ │      │
│  │ └─────────────┘ │  │ └─────────────┘ │  │ └─────────────┘ │      │
│  └─────────────────┘  └─────────────────┘  └─────────────────┘      │
│                                                                     │
├─────────────────────────────────────────────────────────────────────┤
│                         共享组件层                                    │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐      │
│  │  Configuration  │  │  TF Utilities   │  │  Logging        │      │
│  │  Manager        │  │                 │  │  System         │      │
│  └─────────────────┘  └─────────────────┘  └─────────────────┘      │
│                                                                     │
├─────────────────────────────────────────────────────────────────────┤
│                         工具集                                       │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐      │
│  │  CLI Tools      │  │  Launch Files   │  │  Python API     │      │
│  │  - view_tf_tree │  │  - tf_tools.launch.py │  - TFManager    │      │
│  │  - check_tf_chain│  │                 │  │  - TFValidator  │      │
│  │  - tf_health    │  │                 │  │                 │      │
│  └─────────────────┘  └─────────────────┘  └─────────────────┘      │
└─────────────────────────────────────────────────────────────────────┘
                                    │
                              ┌─────▼─────┐
                              │ ROS 2 TF  │
                              │ 子系统     │
                              │ /tf       │
                              │ /tf_static │
                              └───────────┘
```

### 2.2 数据流图

```
配置文件 (YAML)
        │
        ▼
 ┌─────────────────┐
 │ Configuration   │
 │ Manager         │
 └─────┬───────────┘
       │
       ├── Static Config ──► ┌─────────────────┐
       │                    │ Static TF       │ ──► /tf_static
       │                    │ Publisher       │
       │                    └─────────────────┘
       │
       └── Dynamic Config ─► ┌─────────────────┐
                             │ Dynamic TF      │ ──► /tf
                             │ Publisher       │ ◄── 外部话题
                             └─────────────────┘
                                     │
                             ┌─────────────────┐
                             │ TF Validator    │ ──► 健康报告
                             └─────────────────┘
                                     ▲
                                     │
                              /tf + /tf_static
```

## 3. 核心组件设计

### 3.1 Static TF Publisher

#### 3.1.1 组件职责
- 从配置文件加载静态变换定义
- 生成 TransformStamped 消息
- 发布到 `/tf_static` 话题
- 支持配置热重载

#### 3.1.2 类设计

```python
class StaticTFPublisher(Node):
    """静态TF发布器节点"""

    def __init__(self):
        super().__init__('static_tf_publisher_node')

        # 配置管理器
        self.config_manager = ConfigurationManager(self)

        # TF发布器
        self.tf_publisher = self.create_publisher(
            TFMessage, '/tf_static',
            qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        )

        # 服务接口
        self.reload_service = self.create_service(
            Trigger, '~/reload_config', self.reload_config_callback
        )

        # 定时器（可选，用于调试）
        self.publish_timer = None

    def load_and_publish_transforms(self):
        """加载配置并发布静态变换"""

    def reload_config_callback(self, request, response):
        """重新加载配置文件"""

    def publish_static_transforms(self, transforms: List[StaticTransform]):
        """发布静态变换列表"""
```

#### 3.1.3 配置数据结构

```python
@dataclass
class StaticTransform:
    """静态变换数据类"""
    parent_frame: str
    child_frame: str
    translation: List[float]  # [x, y, z]
    rotation: List[float]     # [roll, pitch, yaw] or [x, y, z, w]
    rotation_type: str = "euler"  # "euler" or "quaternion"

@dataclass
class StaticTFConfig:
    """静态TF配置"""
    frame_prefix: str = ""
    transforms: List[StaticTransform] = field(default_factory=list)
    publish_rate: float = 0.0  # 0表示使用静态发布器
    qos_depth: int = 1
```

### 3.2 Dynamic TF Publisher

#### 3.2.1 组件职责
- 订阅外部话题数据源
- 转换为 TransformStamped 格式
- 发布到 `/tf` 话题
- 管理时间戳同步

#### 3.2.2 类设计

```python
class DynamicTFPublisher(Node):
    """动态TF发布器节点"""

    def __init__(self):
        super().__init__('dynamic_tf_publisher_node')

        self.config_manager = ConfigurationManager(self)
        self.tf_publisher = self.create_publisher(TFMessage, '/tf', 10)

        # 动态订阅者字典
        self.subscribers: Dict[str, Subscription] = {}

        # 变换缓存
        self.transform_cache: Dict[str, TransformStamped] = {}

    def setup_dynamic_subscribers(self):
        """根据配置设置动态订阅者"""

    def pose_callback(self, msg, transform_name: str):
        """处理 PoseStamped 消息"""

    def odometry_callback(self, msg, transform_name: str):
        """处理 Odometry 消息"""

    def transform_callback(self, msg, transform_name: str):
        """处理 TransformStamped 消息"""
```

#### 3.2.3 消息转换器

```python
class MessageConverter:
    """消息类型转换器"""

    @staticmethod
    def pose_to_transform(pose_msg: PoseStamped,
                         parent_frame: str,
                         child_frame: str) -> TransformStamped:
        """将 PoseStamped 转换为 TransformStamped"""

    @staticmethod
    def odometry_to_transform(odom_msg: Odometry,
                            parent_frame: str,
                            child_frame: str) -> TransformStamped:
        """将 Odometry 转换为 TransformStamped"""

    @staticmethod
    def apply_frame_prefix(transform: TransformStamped,
                          prefix: str) -> TransformStamped:
        """应用坐标系前缀"""
```

### 3.3 TF Validator

#### 3.3.1 组件职责
- 监控TF系统健康状态
- 验证TF链完整性
- 检测异常情况
- 生成TF树结构

#### 3.3.2 类设计

```python
class TFValidator(Node):
    """TF验证器节点"""

    def __init__(self):
        super().__init__('tf_validator_node')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 健康检查定时器
        self.health_timer = self.create_timer(
            self.check_interval, self.health_check_callback
        )

        # TF监控订阅者
        self.tf_subscriber = self.create_subscription(
            TFMessage, '/tf', self.tf_callback, 10
        )
        self.tf_static_subscriber = self.create_subscription(
            TFMessage, '/tf_static', self.tf_static_callback, 10
        )

        # 健康状态发布者
        self.health_publisher = self.create_publisher(
            TFHealth, '~/tf_health', 10
        )

        # 服务接口
        self.tree_service = self.create_service(
            GetTFTree, '~/get_tf_tree', self.get_tree_callback
        )
        self.check_service = self.create_service(
            CheckTransform, '~/check_transform', self.check_transform_callback
        )

    def health_check_callback(self):
        """执行健康检查"""

    def build_tf_tree(self) -> TFTree:
        """构建TF树结构"""

    def check_transform_chain(self, from_frame: str, to_frame: str) -> bool:
        """检查变换链完整性"""
```

### 3.4 Configuration Manager

#### 3.4.1 组件职责
- 解析YAML配置文件
- 管理配置参数
- 支持变量替换
- 提供配置验证

#### 3.4.2 类设计

```python
class ConfigurationManager:
    """配置管理器"""

    def __init__(self, node: Node):
        self.node = node
        self.config_file_path: str = ""
        self.static_config: StaticTFConfig = StaticTFConfig()
        self.dynamic_config: DynamicTFConfig = DynamicTFConfig()

    def load_config(self, config_file: str) -> bool:
        """加载配置文件"""

    def parse_yaml_config(self, yaml_data: dict) -> bool:
        """解析YAML配置数据"""

    def substitute_variables(self, text: str) -> str:
        """变量替换（环境变量、launch参数）"""

    def validate_config(self) -> Tuple[bool, List[str]]:
        """验证配置有效性"""

    def apply_frame_prefix(self, frame_name: str) -> str:
        """应用坐标系前缀"""

    def get_parameter_with_fallback(self, param_name: str, default_value):
        """获取参数（支持多级fallback）"""
```

### 3.5 TF Utilities

#### 3.5.1 工具函数

```python
class TFUtils:
    """TF工具函数库"""

    @staticmethod
    def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> List[float]:
        """欧拉角转四元数"""

    @staticmethod
    def quaternion_to_euler(x: float, y: float, z: float, w: float) -> List[float]:
        """四元数转欧拉角"""

    @staticmethod
    def create_transform_stamped(parent_frame: str,
                               child_frame: str,
                               translation: List[float],
                               rotation: List[float],
                               stamp: Time = None) -> TransformStamped:
        """创建 TransformStamped 消息"""

    @staticmethod
    def validate_frame_name(frame_name: str) -> bool:
        """验证坐标系名称格式"""

    @staticmethod
    def normalize_quaternion(q: List[float]) -> List[float]:
        """归一化四元数"""
```

## 4. 文件结构

### 4.1 包目录结构

```
tf_tools/
├── config/                      # 配置文件
│   ├── tf_config.yaml          # 默认配置模板
│   ├── examples/               # 示例配置
│   │   ├── single_arm.yaml
│   │   ├── multi_arm.yaml
│   │   └── mobile_robot.yaml
│   └── schemas/                # 配置文件JSON Schema
│       └── tf_config_schema.json
├── doc/                        # 文档
│   ├── requirements.md         # 需求文档
│   ├── architecture.md         # 架构文档（本文档）
│   ├── api_reference.md        # API参考
│   └── user_guide.md          # 用户指南
├── launch/                     # Launch文件
│   ├── tf_tools.launch.py     # 主启动文件
│   ├── static_tf.launch.py    # 仅静态TF
│   └── dynamic_tf.launch.py   # 仅动态TF
├── scripts/                    # 可执行脚本
│   ├── view_tf_tree           # TF树可视化工具
│   ├── check_tf_chain         # TF链检查工具
│   ├── check_tf_health        # TF健康检查工具
│   └── export_tf_tree         # TF树导出工具
├── src/                       # 源码目录
│   └── tf_tools/              # Python包
│       ├── __init__.py
│       ├── nodes/             # 节点实现
│       │   ├── __init__.py
│       │   ├── static_tf_publisher.py
│       │   ├── dynamic_tf_publisher.py
│       │   └── tf_validator.py
│       ├── core/              # 核心组件
│       │   ├── __init__.py
│       │   ├── config_manager.py
│       │   ├── tf_utils.py
│       │   └── message_converter.py
│       ├── tools/             # 工具模块
│       │   ├── __init__.py
│       │   ├── tree_visualizer.py
│       │   ├── chain_checker.py
│       │   └── health_monitor.py
│       └── msgs/              # 自定义消息（如需要）
│           ├── __init__.py
│           ├── TFHealth.py
│           └── TFTree.py
├── test/                      # 测试文件
│   ├── unit/                  # 单元测试
│   ├── integration/           # 集成测试
│   └── data/                  # 测试数据
├── log/                       # 日志目录（运行时创建）
├── package.xml                # ROS包描述文件
├── setup.py                   # Python包设置文件
├── setup.cfg                  # 配置文件
├── start_tf_tools.sh          # 启动脚本
└── README.md                  # 包说明文档
```

### 4.2 关键文件说明

#### package.xml
```xml
<?xml version="1.0"?>
<package format="3">
  <name>tf_tools</name>
  <version>1.0.0</version>
  <description>ROS 2 TF管理工具包</description>
  <maintainer email="dev@l2robotics.com">L2 开发团队</maintainer>
  <license>MIT</license>

  <depend>rclpy</depend>
  <depend>tf2_ros</depend>
  <depend>geometry_msgs</depend>
  <depend>std_srvs</depend>

  <test_depend>pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

#### setup.py
```python
from setuptools import setup, find_packages

package_name = 'tf_tools'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/tf_tools.launch.py']),
        ('share/' + package_name + '/config', ['config/tf_config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='L2 开发团队',
    maintainer_email='dev@l2robotics.com',
    description='ROS 2 TF管理工具包',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'static_tf_publisher_node = tf_tools.nodes.static_tf_publisher:main',
            'dynamic_tf_publisher_node = tf_tools.nodes.dynamic_tf_publisher:main',
            'tf_validator_node = tf_tools.nodes.tf_validator:main',
            'view_tf_tree = tf_tools.tools.tree_visualizer:main',
            'check_tf_chain = tf_tools.tools.chain_checker:main',
            'check_tf_health = tf_tools.tools.health_monitor:main',
            'export_tf_tree = tf_tools.tools.tree_exporter:main',
        ],
    },
)
```

## 5. 接口设计

### 5.1 ROS 2 话题

| 话题名称 | 消息类型 | 发布/订阅 | 说明 |
|---------|----------|----------|------|
| `/tf` | tf2_msgs/TFMessage | 发布 | 动态变换发布 |
| `/tf_static` | tf2_msgs/TFMessage | 发布 | 静态变换发布 |
| `~/tf_health` | TFHealth (自定义) | 发布 | TF健康状态 |
| `/localization/pose` | geometry_msgs/PoseStamped | 订阅 | 定位数据（示例） |
| `/odometry/filtered` | nav_msgs/Odometry | 订阅 | 里程计数据（示例） |

### 5.2 ROS 2 服务

| 服务名称 | 服务类型 | 说明 |
|---------|----------|------|
| `~/reload_config` | std_srvs/Trigger | 重新加载配置文件 |
| `~/add_transform` | AddTransform (自定义) | 动态添加静态变换 |
| `~/get_tf_tree` | GetTFTree (自定义) | 获取TF树结构 |
| `~/check_transform` | CheckTransform (自定义) | 检查变换链 |

### 5.3 ROS 2 参数

| 参数名称 | 类型 | 默认值 | 说明 |
|---------|------|--------|------|
| `config_file` | string | config/tf_config.yaml | 配置文件路径 |
| `frame_prefix` | string | "" | 坐标系前缀 |
| `namespace` | string | "" | 命名空间 |
| `publish_rate` | double | 0.0 | 发布频率（0=静态） |
| `check_interval` | double | 5.0 | 健康检查间隔 |
| `max_transform_age` | double | 1.0 | 最大变换延迟 |

### 5.4 自定义消息类型

#### TFHealth
```
# TF系统健康状态消息
std_msgs/Header header

# 总体状态
bool healthy

# 检查项详情
TFHealthItem[] health_items

# 统计信息
int32 total_transforms
int32 static_transforms
int32 dynamic_transforms
int32 failed_transforms
```

#### TFHealthItem
```
# 单个检查项
string name                # 检查项名称
string status              # OK, WARNING, ERROR
string message             # 详细信息
float64 value              # 检查值（如延迟时间）
float64 threshold          # 阈值
```

#### TFTree
```
# TF树结构
TFTreeNode[] nodes
TFTreeEdge[] edges
string root_frame
```

### 5.5 Python API

```python
# 主要类导出
from tf_tools import (
    TFTreeManager,      # TF树管理器
    TFValidator,        # TF验证器
    ConfigManager,      # 配置管理器
    TFPublisher,        # TF发布器
    TFUtils,           # 工具函数
)

# 使用示例
manager = TFTreeManager()
manager.load_config('config/tf_config.yaml')
manager.start_publishing()

# 验证器
validator = TFValidator(node)
health = validator.check_health()
tree = validator.get_tf_tree()
```

## 6. 性能设计

### 6.1 内存优化

1. **配置缓存**：配置加载后缓存在内存，避免重复解析
2. **消息复用**：复用 TransformStamped 对象，减少内存分配
3. **延迟初始化**：按需创建订阅者和发布者
4. **内存池**：对于高频变换，使用对象池模式

### 6.2 CPU优化

1. **批量发布**：将多个静态变换打包为一条 TFMessage
2. **计算缓存**：缓存旋转矩阵等计算结果
3. **异步处理**：使用多线程处理复杂计算
4. **智能频率控制**：根据变换变化幅度动态调整发布频率

### 6.3 网络优化

1. **QoS配置**：
   - 静态TF：`TRANSIENT_LOCAL` + `RELIABLE`
   - 动态TF：`VOLATILE` + `BEST_EFFORT`
2. **压缩发布**：大量变换时考虑数据压缩
3. **频率控制**：避免不必要的高频发布

### 6.4 性能监控

```python
class PerformanceMonitor:
    """性能监控器"""

    def __init__(self):
        self.publish_count = 0
        self.publish_time_total = 0.0
        self.memory_usage = 0

    def record_publish(self, duration: float):
        """记录发布耗时"""

    def get_stats(self) -> Dict[str, float]:
        """获取性能统计"""
        return {
            'avg_publish_time': self.publish_time_total / self.publish_count,
            'publish_rate': self.publish_count / self.uptime,
            'memory_usage_mb': self.memory_usage / 1024 / 1024
        }
```

## 7. 错误处理设计

### 7.1 错误分类

1. **配置错误**：文件不存在、格式错误、参数无效
2. **运行时错误**：TF发布失败、时间戳异常、网络问题
3. **逻辑错误**：TF链断裂、坐标系冲突、循环依赖

### 7.2 错误处理策略

```python
class ErrorHandler:
    """统一错误处理器"""

    def __init__(self, logger):
        self.logger = logger
        self.error_count = defaultdict(int)

    def handle_config_error(self, error: Exception, file_path: str):
        """处理配置错误"""
        self.logger.error(f"配置文件错误 {file_path}: {error}")
        # 使用默认配置或拒绝启动

    def handle_tf_publish_error(self, error: Exception, transform: str):
        """处理TF发布错误"""
        self.error_count[transform] += 1
        if self.error_count[transform] > 10:
            self.logger.error(f"TF发布持续失败，停止发布: {transform}")
            # 停止该变换的发布
        else:
            self.logger.warning(f"TF发布暂时失败，将重试: {transform}")

    def handle_validation_error(self, error: Exception, check_name: str):
        """处理验证错误"""
        self.logger.warning(f"验证检查失败 {check_name}: {error}")
        # 继续运行但标记状态
```

### 7.3 恢复机制

1. **自动重试**：网络错误、临时故障自动重试
2. **降级模式**：部分功能失败时保持核心功能运行
3. **状态恢复**：重新加载配置、重新建立连接
4. **用户通知**：通过日志、话题发布错误状态

## 8. 测试架构

### 8.1 测试策略

1. **单元测试**：每个模块的独立功能测试
2. **集成测试**：模块间协作的端到端测试
3. **性能测试**：大量数据的压力测试
4. **兼容性测试**：与其他ROS包的兼容性验证

### 8.2 测试框架

```python
# test/unit/test_config_manager.py
import pytest
from tf_tools.core.config_manager import ConfigurationManager

class TestConfigManager:

    def setup_method(self):
        """测试设置"""
        self.config_manager = ConfigurationManager(mock_node)

    def test_load_valid_config(self):
        """测试加载有效配置"""

    def test_load_invalid_config(self):
        """测试加载无效配置"""

    def test_variable_substitution(self):
        """测试变量替换"""

    def test_frame_prefix_application(self):
        """测试坐标系前缀应用"""
```

### 8.3 CI/CD集成

```yaml
# .github/workflows/test.yml
name: tf_tools测试

on: [push, pull_request]

jobs:
  test:
    runs-on: ubuntu-latest
    container: ros:humble

    steps:
    - uses: actions/checkout@v2

    - name: 安装依赖
      run: |
        apt-get update
        apt-get install -y python3-pip
        pip3 install pytest

    - name: 构建包
      run: |
        source /opt/ros/humble/setup.bash
        colcon build --packages-select tf_tools

    - name: 运行测试
      run: |
        source install/setup.bash
        python3 -m pytest test/ -v

    - name: 静态检查
      run: |
        flake8 src/tf_tools/
        mypy src/tf_tools/
```

## 9. 部署架构

### 9.1 部署模式

#### 模式1：单节点部署（推荐）
```bash
# 在一个节点中运行所有功能
ros2 launch tf_tools tf_tools.launch.py \
  enable_static:=true \
  enable_dynamic:=true \
  enable_validation:=true
```

#### 模式2：分布式部署
```bash
# 分别启动不同功能节点
ros2 run tf_tools static_tf_publisher_node &
ros2 run tf_tools dynamic_tf_publisher_node &
ros2 run tf_tools tf_validator_node &
```

#### 模式3：容器化部署
```dockerfile
FROM ros:humble

COPY . /workspace/src/tf_tools
WORKDIR /workspace

RUN source /opt/ros/humble/setup.bash && \
    colcon build --packages-select tf_tools

ENTRYPOINT ["ros2", "launch", "tf_tools", "tf_tools.launch.py"]
```

### 9.2 配置管理

1. **开发环境**：使用本地配置文件
2. **测试环境**：通过环境变量覆盖配置
3. **生产环境**：使用专用配置文件和参数服务器

### 9.3 监控和维护

1. **健康检查端点**：提供HTTP接口查询状态
2. **日志聚合**：集中收集和分析日志
3. **性能监控**：监控CPU、内存、网络使用情况
4. **告警机制**：TF异常时自动通知

## 10. 扩展性设计

### 10.1 插件架构

```python
class TFPlugin:
    """TF插件基类"""

    def initialize(self, node: Node, config: dict):
        """插件初始化"""
        pass

    def process_transform(self, transform: TransformStamped) -> TransformStamped:
        """处理变换（可选的变换处理链）"""
        return transform

    def validate_transform(self, transform: TransformStamped) -> bool:
        """验证变换（可选的验证规则）"""
        return True

# 插件注册
class PluginManager:
    def __init__(self):
        self.plugins: List[TFPlugin] = []

    def register_plugin(self, plugin: TFPlugin):
        """注册插件"""
        self.plugins.append(plugin)

    def load_plugins_from_config(self, config: dict):
        """从配置加载插件"""
        pass
```

### 10.2 扩展点

1. **消息转换器**：支持新的消息类型
2. **验证规则**：添加自定义验证逻辑
3. **输出格式**：支持新的TF树输出格式
4. **数据源**：支持新的动态变换数据源

### 10.3 API版本控制

```python
# 向前兼容的API设计
class TFManager_v1:
    """API v1.0 - 稳定接口"""

class TFManager_v2(TFManager_v1):
    """API v2.0 - 扩展功能，保持兼容"""

# 版本选择
def create_tf_manager(api_version: str = "latest") -> TFManager:
    """创建指定版本的TF管理器"""
    if api_version == "v1" or api_version == "1.0":
        return TFManager_v1()
    elif api_version == "v2" or api_version == "2.0":
        return TFManager_v2()
    else:
        return TFManager_v2()  # 默认最新版本
```

## 11. 安全性设计

### 11.1 输入验证

1. **配置文件验证**：使用JSON Schema验证配置格式
2. **参数范围检查**：验证数值参数在合理范围内
3. **坐标系名称检查**：防止恶意或冲突的坐标系名称
4. **文件路径验证**：防止路径遍历攻击

### 11.2 运行时安全

1. **资源限制**：限制内存和CPU使用
2. **权限控制**：最小权限原则运行
3. **输入过滤**：过滤异常的TF数据
4. **日志安全**：避免敏感信息泄露到日志

### 11.3 配置安全

```python
class SecureConfigValidator:
    """安全配置验证器"""

    ALLOWED_FRAME_CHARS = re.compile(r'^[a-zA-Z0-9_/]+$')
    MAX_TRANSLATION = 100.0  # 最大平移距离（米）
    MAX_TRANSFORMS = 1000    # 最大变换数量

    def validate_frame_name(self, frame_name: str) -> bool:
        """验证坐标系名称安全性"""
        if not self.ALLOWED_FRAME_CHARS.match(frame_name):
            return False
        if len(frame_name) > 100:
            return False
        return True

    def validate_transform_values(self, translation: List[float],
                                rotation: List[float]) -> bool:
        """验证变换数值安全性"""
        # 检查平移值
        if any(abs(x) > self.MAX_TRANSLATION for x in translation):
            return False
        # 检查旋转值
        if len(rotation) == 3:  # 欧拉角
            if any(abs(x) > 2*math.pi for x in rotation):
                return False
        elif len(rotation) == 4:  # 四元数
            norm = sum(x*x for x in rotation)
            if abs(norm - 1.0) > 0.1:  # 非归一化四元数
                return False
        return True
```

## 12. 总结

tf_tools 包的架构设计充分考虑了以下几个方面：

1. **模块化设计**：清晰的职责分离，便于开发和维护
2. **配置驱动**：灵活的配置管理，支持多种部署场景
3. **高性能**：优化的发布策略和内存使用
4. **可扩展性**：插件架构支持功能扩展
5. **可靠性**：完善的错误处理和恢复机制
6. **安全性**：输入验证和运行时保护

这个架构为后续的具体实现提供了清晰的指导方针，确保系统的稳定性、性能和可维护性。

---

**文档维护者**：L2 开发团队
**最后更新**：2025-11-12
**状态**：架构设计完成，待评审