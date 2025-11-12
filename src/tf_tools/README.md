# TF Tools - ROS 2 坐标变换管理工具包

## 概述

tf_tools 是 L2 机器人系统中负责 TF（坐标变换）管理的专用包。本包提供了统一的接口来管理机器人系统中的静态和动态坐标变换，简化了复杂TF关系的配置和维护工作。

## 核心功能

- **静态TF管理**：批量发布和管理静态坐标变换
- **动态TF管理**：订阅外部数据源并转发为TF变换
- **TF树验证**：诊断和验证TF系统的健康状态
- **多机器人支持**：通过 frame_prefix 和 namespace 支持多机器人场景
- **配置驱动**：所有参数通过 YAML 配置文件管理

## 目录结构

```
tf_tools/
├── config/          # 配置文件
│   └── tf_config.yaml
├── doc/             # 文档
│   ├── requirements.md      # 功能需求文档
│   └── architecture.md      # 架构设计文档
├── launch/          # Launch文件（待实现）
├── scripts/         # 命令行工具（待实现）
└── src/             # 源码（待实现）
```

## 快速开始

### 1. 配置TF关系

编辑配置文件 `config/tf_config.yaml`：

```yaml
tf_config:
  static_transforms:
    - parent_frame: world
      child_frame: base_link
      translation: [0.0, 0.0, 0.0]
      rotation: [0.0, 0.0, 0.0]
```

### 2. 启动TF发布节点（待实现）

```bash
# 使用启动脚本
./src/tf_tools/start_tf_tools.sh

# 或使用 launch 文件
ros2 launch tf_tools tf_tools.launch.py
```

### 3. 验证TF发布（待实现）

```bash
# 查看TF树
ros2 run tf_tools view_tf_tree

# 检查变换链
ros2 run tf_tools check_tf_chain --from world --to base_link
```

## 文档

- **[功能需求文档](doc/requirements.md)** - 详细的功能需求和使用场景
- **[架构设计文档](doc/architecture.md)** - 系统架构和技术设计
- **[配置文件说明](config/tf_config.yaml)** - 配置文件格式和参数说明

## 与 description 包的职责划分

| 功能 | description 包 | tf_tools 包 |
|------|---------------|-------------|
| 发布 robot_description | ✓ | - |
| 发布机械臂关节TF | ✓ | - |
| 发布 world → base_link | - | ✓ |
| 发布传感器静态TF | - | ✓ |
| 发布多机器人全局TF | - | ✓ |
| TF树验证 | - | ✓ |

## 实现进度

### 当前状态：需求整理阶段

- [x] 创建包目录结构
- [x] 编写功能需求文档
- [x] 编写架构设计文档
- [x] 创建配置文件模板
- [ ] 实现静态TF发布节点
- [ ] 实现动态TF发布节点
- [ ] 实现TF验证节点
- [ ] 实现命令行工具
- [ ] 编写测试用例
- [ ] 编写用户指南

## 依赖

### ROS 2 依赖
- rclpy
- tf2_ros
- geometry_msgs
- std_srvs

### Python 依赖
- pyyaml
- numpy

## 开发规范

- 遵循 ROS 2 Python 风格指南
- 使用类型注解
- 编写完整的 docstring
- 代码注释使用中文
- 使用 ROS 2 logger（避免 print）

## 参与贡献

本项目由 L2 开发团队维护。如有问题或建议，请联系项目维护者。

## 许可证

MIT License

---

**版本**: 1.0.0
**最后更新**: 2025-11-12
**维护者**: L2 开发团队
