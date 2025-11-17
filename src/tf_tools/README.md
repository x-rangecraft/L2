# TF Tools - 静态与动态 TF 发布工具

## 目录结构

```
tf_tools/
├── README.md                          # 本文档
├── IMPLEMENTATION.md                  # 实现细节文档
├── tf.md                              # 原始文档（静态TF相关）
├── check_environment.sh               # 环境检查工具
├── static_tf_config_build.sh          # 静态TF配置生成工具
├── tf_publisher.sh                    # 统一启动入口（静态+动态TF）
├── static_tf_config.yaml              # 静态TF配置文件
├── src/                               # 代码实现目录
│   ├── dynamic_tf_publish.sh          # 动态TF独立启动脚本
│   └── robot_tf_publisher.py          # 动态TF发布节点（Python）
└── .static_tf_publish/                # 静态TF运行状态目录
    ├── pids                           # 静态TF进程PID
    └── logs/                          # 静态TF日志（本地）
```

## 日志位置

- **静态TF日志**: `src/tf_tools/.static_tf_publish/logs/`
- **动态TF日志**: `L2/log/tf_tools/robot_tf_publisher.log`

## 功能说明

### 1. 静态 TF 发布（保留原有功能）

发布固定不变的坐标变换，如：
- `world → base_link`
- `world → camera_link`

**不再发布 URDF 链接间的静态变换**（这些现在由动态TF负责）

### 2. 动态 TF 发布（新增功能）

**节点名称**: `robot_tf_publisher`

**功能**:
- 读取 URDF 模型（`src/description/urdf/yam.urdf`）
- 订阅 `/joint_states` 话题
- 根据关节角度实时计算并发布机器人各链接的 TF 变换
- 发布到 `/tf` 话题（动态变换）

**发布的变换**:
- `base_link → link_1` (根据 joint1 角度)
- `link_1 → link_2` (根据 joint2 角度)
- `link_2 → link_3` (根据 joint3 角度)
- `link_3 → link_4` (根据 joint4 角度)
- `link_4 → link_5` (根据 joint5 角度)
- `link_5 → link_6` (根据 joint6 角度)

## 使用方法

### 统一入口：tf_publisher.sh

这是推荐的启动方式，可以同时管理静态和动态TF。

#### 1. 前台运行（调试用）

```bash
# 同时启动静态和动态TF
./src/tf_tools/tf_publisher.sh

# 仅启动动态TF（用于机器人关节）
./src/tf_tools/tf_publisher.sh --dynamic-only

# 仅启动静态TF（world相关变换）
./src/tf_tools/tf_publisher.sh --static-only
```

按 `Ctrl+C` 停止。

#### 2. 后台运行（生产环境）

```bash
# 后台启动全部
./src/tf_tools/tf_publisher.sh --daemon

# 后台启动动态TF
./src/tf_tools/tf_publisher.sh --daemon --dynamic-only

# 后台启动静态TF
./src/tf_tools/tf_publisher.sh --daemon --static-only
```

#### 3. 查看状态

```bash
# 查看所有TF发布器状态
./src/tf_tools/tf_publisher.sh --status

# 仅查看动态TF状态
./src/tf_tools/tf_publisher.sh --status --dynamic-only
```

#### 4. 停止服务

```bash
# 停止所有TF发布器
./src/tf_tools/tf_publisher.sh --stop

# 仅停止动态TF
./src/tf_tools/tf_publisher.sh --stop --dynamic-only

# 仅停止静态TF
./src/tf_tools/tf_publisher.sh --stop --static-only
```

### 独立使用动态TF发布

如果只需要动态TF发布，可以直接使用：

```bash
# 前台运行
./src/tf_tools/src/dynamic_tf_publish.sh

# 后台运行
./src/tf_tools/src/dynamic_tf_publish.sh --daemon

# 查看状态
./src/tf_tools/src/dynamic_tf_publish.sh --status

# 停止
./src/tf_tools/src/dynamic_tf_publish.sh --stop
```

## 工作原理

### 动态TF发布流程

```
┌─────────────────┐
│  URDF 模型      │
│  (yam.urdf)     │
└────────┬────────┘
         │
         ├─ 解析运动学结构
         │  - 关节类型
         │  - 父子关系
         │  - 原点变换
         │  - 关节轴
         │
         v
┌─────────────────────────┐
│ robot_tf_publisher      │
│                         │
│  订阅 /joint_states ───>│──> 读取关节角度
│                         │
│  正向运动学计算 ────────>│──> 计算每个link位姿
│                         │
│  发布 TF ───────────────>│──> 发布到 /tf 话题
└─────────────────────────┘
```

### 与 robot_state_publisher 的区别

`robot_tf_publisher` 是一个**简化版**的 `robot_state_publisher`：

| 功能 | robot_state_publisher | robot_tf_publisher |
|------|----------------------|-------------------|
| 读取 URDF | ✅ | ✅ |
| 订阅 joint_states | ✅ | ✅ |
| 发布动态 TF | ✅ | ✅ |
| 复杂运动学树 | ✅ | ⚠️ 支持简单链式结构 |
| URDF mimic joints | ✅ | ❌ |
| 发布 robot_description | ✅ | ❌ |

**适用场景**:
- ✅ 简单的串联机器人（如6轴机械臂）
- ✅ 需要自定义TF发布逻辑
- ✅ 想要完全控制TF发布行为
- ❌ 复杂的并联机构
- ❌ 需要 mimic joints 支持

## 配置参数

### robot_tf_publisher 参数

在 `src/dynamic_tf_publish.sh` 中可以修改：

```bash
python3 src/robot_tf_publisher.py \
  --ros-args \
  -p urdf_path:="/path/to/urdf"    # URDF文件路径
  -p publish_rate:=50.0             # 发布频率（Hz）
  -p base_frame:=base_link          # 基座坐标系名称
```

## 验证与调试

### 1. 检查 TF 树

```bash
# 查看 TF 树结构
ros2 run tf2_tools view_frames

# 查看特定变换
ros2 run tf2_ros tf2_echo base_link link_6
```

### 2. 查看发布的话题

```bash
# 查看 TF 话题
ros2 topic echo /tf

# 查看 joint_states
ros2 topic echo /joint_states
```

### 3. 在 Foxglove 中验证

1. 打开 Foxglove Studio
2. 连接到 ROS 2
3. 添加 3D 面板
4. 查看 `/robot_description` 和 TF 树
5. 移动机器人关节，观察模型是否同步运动

### 4. 查看日志

```bash
# 动态TF日志
tail -f L2/log/tf_tools/robot_tf_publisher.log

# 静态TF日志
ls -l src/tf_tools/.static_tf_publish/logs/
```

## 常见问题

### Q1: Foxglove 中机器人模型不动

**检查步骤**:
1. 确认 `robot_tf_publisher` 正在运行
   ```bash
   ./src/tf_tools/tf_publisher.sh --status
   ```
2. 确认 `/joint_states` 有数据
   ```bash
   ros2 topic echo /joint_states --once
   ```
3. 确认 TF 正在发布
   ```bash
   ros2 topic hz /tf
   ```

### Q2: 启动失败，提示找不到 URDF

**解决方案**:
确保 URDF 文件存在：
```bash
ls -l /home/jetson/L2/src/description/urdf/yam.urdf
```

### Q3: 关节名称不匹配

**解决方案**:
- URDF 中的关节名称必须与 `/joint_states` 中的名称一致
- 检查 URDF: `grep '<joint name=' src/description/urdf/yam.urdf`
- 检查 joint_states: `ros2 topic echo /joint_states --once | grep name`

### Q4: 想要关闭静态TF，只用动态TF

**解决方案**:
```bash
# 停止静态TF
./src/tf_tools/tf_publisher.sh --stop --static-only

# 只启动动态TF
./src/tf_tools/tf_publisher.sh --daemon --dynamic-only
```

## 升级说明

### 从旧版本升级

如果你之前使用的是纯静态TF发布：

1. **不需要重新配置** - `static_tf_config.yaml` 保持不变
2. **URDF 链接变换会自动移除** - 由动态TF接管
3. **使用相同的启动脚本** - `tf_publisher.sh`
4. **新增了 `--dynamic-only` 选项** - 可以灵活控制

### 回退到纯静态模式

如果需要回退（不推荐）：

```bash
# 停止动态TF
./src/tf_tools/tf_publisher.sh --stop --dynamic-only

# 只启动静态TF（包括URDF链接）
./src/tf_tools/tf_publisher.sh --daemon --static-only
```

## 维护与开发

### 修改动态TF发布逻辑

编辑 `src/robot_tf_publisher.py`，主要修改点：

- `compute_transform()`: 修改变换计算逻辑
- `joint_state_callback()`: 修改关节状态处理
- `publish_dynamic_transforms()`: 修改发布策略

### 添加新的关节类型支持

在 `URDFJoint.is_movable()` 中添加新的关节类型。

### 调试技巧

```bash
# 直接运行Python节点（查看详细输出）
python3 src/tf_tools/src/robot_tf_publisher.py \
  --ros-args -p urdf_path:=src/description/urdf/yam.urdf

# 设置ROS日志级别
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}]: {message}"
```

## 参考资料

- 原始静态TF文档: `tf.md`
- ROS 2 TF2 教程: https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html
- URDF 规范: http://wiki.ros.org/urdf/XML
