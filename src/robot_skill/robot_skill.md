# robot_skill 节点需求草案

本文件用于与用户一起逐步补齐机器人技能规划与执行相关的节点需求。当前为初稿，请指出需要增删的内容。

## 1. 背景与目标
- 节点名称：`robot_skill`
- 硬件背景：单机械臂，型号 YAM 6-DoF（见 `driver/robot_description.yaml`），单 RealSense D435i 用于目标检测与定位
- 输入特性：上游视觉模块输出基于 `base_link` 坐标系的目标位姿（XYZ + 三维姿态信息），本节点需以此为起点规划连续指令序列
- 主要目标：根据目标位置姿态三维信息，规划并执行一连串的机械臂运动指令，通过发布 ROS2 话题到 `robot_driver` 来控制机械臂执行复合技能（如抓取、放置等）
- 场景：抓取、放置、对位等单臂任务

## 2. 架构分层与职责边界

### 2.1 系统分层
```
┌─────────────────────────────────────────────────────────────┐
│                    上层应用 / 任务调度                        │
│            （语音指令、状态机、任务队列等）                    │
└─────────────────────────────────────────────────────────────┘
                              ↓ 调用技能
┌─────────────────────────────────────────────────────────────┐
│                    robot_skill (本节点)                      │
│                      任务层 / 技能层                         │
│               关心: "做什么" (What to do)                    │
└─────────────────────────────────────────────────────────────┘
                              ↓ 笛卡尔位姿指令
┌─────────────────────────────────────────────────────────────┐
│                      robot_driver                           │
│                     驱动层 / 能力层                          │
│               关心: "怎么做" (How to do)                     │
└─────────────────────────────────────────────────────────────┘
                              ↓ 关节控制
┌─────────────────────────────────────────────────────────────┐
│                    硬件 / I2RT SDK                          │
│                  电机控制、CAN 通信                          │
└─────────────────────────────────────────────────────────────┘
```

### 2.2 robot_skill 职责（本节点）

**核心职责：任务分解与位姿序列规划**

| 职责 | 说明 |
|------|------|
| 任务分解 | 将高层任务（如"抓取物体"）分解为有序的位姿序列 |
| 位姿序列生成 | 在笛卡尔空间规划：预抓取位姿、接近位姿、抓取位姿、抬起位姿等 |
| 执行状态机 | 管理技能执行流程：等待到位 → 控制夹爪 → 确认状态 → 下一步 |
| 夹爪协调 | 在合适的时机发送夹爪开/闭指令 |
| 异常处理 | 处理执行失败（超时、IK失败、抓取失败等），决定重试或终止 |
| 参数管理 | 管理技能相关参数（接近距离、抬起高度、超时时间等） |

**robot_skill 需要做的事情：**
- 接收视觉模块的目标位姿
- 根据任务类型生成位姿序列（所有计算在笛卡尔空间完成）
- 按顺序下发 `PoseStamped` 到 `/robot_driver/robot_command`
- 监听 `/joint_states` 判断是否到位
- 在合适时机控制夹爪
- 验证任务结果（如检测夹爪是否夹住物体）
- 向上层报告任务执行结果

**robot_skill 不需要做的事情：**
- ❌ IK 求解（由 robot_driver 负责）
- ❌ 关节空间轨迹插值（由 robot_driver 负责）
- ❌ 关节限位检查（由 robot_driver 负责）
- ❌ 速度/加速度规划（由 robot_driver 负责）
- ❌ 自碰撞检测（由 robot_driver 负责）
- ❌ 电机控制细节（由 robot_driver 负责）

### 2.3 robot_driver 职责（下游节点）

| 职责 | 说明 |
|------|------|
| IK 求解 | 将笛卡尔位姿转换为关节角度 |
| 轨迹插值 | 在关节空间平滑插值，避免突变 |
| 速度规划 | 根据关节变化量自动计算 ramp 时间 |
| 关节限位 | 检查并约束关节角度在安全范围内 |
| 自碰撞检测 | 检测并避免机械臂自碰撞（基础级别） |
| 电机控制 | 通过 CAN 总线下发关节指令 |
| 状态发布 | 发布 `/joint_states`、TF 等 |

### 2.4 设计原则

1. **笛卡尔优先**：robot_skill 只在笛卡尔空间规划，不涉及关节空间
2. **职责单一**：skill 关注"做什么"，driver 关注"怎么做"
3. **接口简洁**：skill 只需发布 `PoseStamped`，无需了解机器人运动学
4. **解耦设计**：更换机器人型号只需修改 driver，skill 代码无需改动

## 3. 接口定义

### 3.1 输出接口

| 话题/服务 | 消息类型 | 说明 |
|-----------|---------|------|
| `/robot_driver/robot_command` | `geometry_msgs/PoseStamped` | 笛卡尔位姿指令（主要接口） |
| `/robot_driver/gripper_command` | 待定 | 夹爪控制指令 |

### 3.2 笛卡尔位姿指令格式

```yaml
header:
  frame_id: base_link  # 默认坐标系
  stamp: {sec: 0, nanosec: 0}
pose:
  position: {x: 0.35, y: 0.0, z: 0.32}  # 单位：米
  orientation: {x: 0.0, y: 0.707, z: 0.0, w: 0.707}  # 单位四元数
```

- 可达范围：x ≈ [-0.61, 0.60] m, y ≈ [-0.59, 0.61] m, z ≈ [-0.37, 0.72] m
- 末端到 base_link 原点距离 r ≈ [0.04, 0.73] m
- 若启用 `xyz_only_mode` 则忽略姿态，沿用当前值

### 3.3 Driver 行为说明

- IK 求解后在关节空间插值
- 根据当前-目标关节最大夹角自动计算 ramp 速度（0.25–5 s）
- 若 IK 失败或目标超出可达区，机械臂保持不动，日志输出 `IK failed for target pose`
- 新指令会抢占旧指令（支持 10–30 Hz 连续推送）

## 4. 技能设计

### 4.1 抓取技能（GraspSkill）

**位姿序列（笛卡尔空间）：**

```
1. 预抓取位姿 (approach_pose)
   - 目标物体正上方 ~20cm
   - 用于安全接近，避免碰撞

2. 接近位姿 (pre_grasp_pose)
   - 目标物体正上方 ~2cm
   - 准备抓取

3. 抓取位姿 (grasp_pose)
   - 目标物体位置
   - 夹爪接触物体

4. 抬起位姿 (lift_pose)
   - 抓取后抬起 ~15cm
   - 确认抓取成功
```

**执行流程：**

```
┌──────────────┐
│   开始抓取    │
└──────┬───────┘
       ↓
┌──────────────┐
│ 移动到预抓取  │ ← PoseStamped (approach_pose)
│   等待到位    │
└──────┬───────┘
       ↓
┌──────────────┐
│  打开夹爪    │ ← GripperCommand (open)
└──────┬───────┘
       ↓
┌──────────────┐
│ 移动到接近位姿 │ ← PoseStamped (pre_grasp_pose)
│   等待到位    │
└──────┬───────┘
       ↓
┌──────────────┐
│ 移动到抓取位姿 │ ← PoseStamped (grasp_pose)
│   等待到位    │
└──────┬───────┘
       ↓
┌──────────────┐
│  闭合夹爪    │ ← GripperCommand (close)
│ 等待夹爪反馈  │
└──────┬───────┘
       ↓
┌──────────────┐
│ 移动到抬起位姿 │ ← PoseStamped (lift_pose)
│   等待到位    │
└──────┬───────┘
       ↓
┌──────────────┐
│ 验证抓取成功  │ ← 检查夹爪位置/力
└──────┬───────┘
       ↓
┌──────────────┐
│   抓取完成    │
└──────────────┘
```

### 4.2 放置技能（PlaceSkill）

**位姿序列：**
1. 预放置位姿：目标位置上方 ~15cm
2. 放置位姿：目标位置
3. 撤离位姿：放置后抬起 ~10cm

### 4.3 位姿计算示例

```python
def compute_approach_pose(target_pose: Pose, offset_z: float = 0.20) -> Pose:
    """计算预抓取位姿：在目标正上方"""
    approach = copy.deepcopy(target_pose)
    approach.position.z += offset_z
    return approach

def compute_lift_pose(grasp_pose: Pose, lift_height: float = 0.15) -> Pose:
    """计算抬起位姿：保持 XY 不变，Z 抬高"""
    lift = copy.deepcopy(grasp_pose)
    lift.position.z += lift_height
    return lift
```

### 4.4 动作类型规范

| Type | 必填参数 | 可选参数 | 说明 |
|------|----------|----------|------|
| `safety_pose` | 无 | `enable_zero_gravity_after`（bool） | 触发 `SafetyPose` action，把机械臂送回安全位姿；仅支持布尔参数控制是否进入零重力模式。 |
| `cartesian_move` | `frame_id`、`target_pose.position.{x,y,z}`、`target_pose.orientation.{x,y,z,w}` | `speed_scale（默认 DEFAULT_SPEED_SCALE）`、`label` | 发送 `PoseStamped` 给 `/robot_driver/action/robot`；姿态必须是单位四元数，所有坐标以米为单位。 |
| `gripper` | `command`（0=张开，1=闭合） | `width_m`、`speed_scale（默认 DEFAULT_SPEED_SCALE）`、`stop_on_contact`、`label` | 驱动夹爪动作；`width_m` 代表目标开度，`stop_on_contact` 控制遇物体即停。 |
| `joint_move` | `positions`（至少一个浮点角度）、`joint_name` 或合法的 `joint_index`（0-5） | `speed_scale（默认 DEFAULT_SPEED_SCALE）`、`relative`、`label` | 向 `/robot_driver/action/joint` 依次下发关节指令；如果走相对运动需把 `relative` 设为 `true`。 |

> Skill YAML 中只能选择上述标准 `type`，`DEFAULT_ACTION_TIMEOUT_SEC = 30.0` 秒会自动作用于所有动作。

### 4.5 配置自检

- `robot_skill` 节点启动时会读取 `skill_sets/` 下的 `general_skill.yaml` 及所有 `*.yaml` 技能文件，并基于上述类型规范做一次 schema 自检：
  - 校验每个公共动作与技能步骤只使用受支持的 `type`；
  - 检查必填字段是否完整（例如 `cartesian_move` 的 `target_pose`、`joint_move` 的 `positions` 等）；
  - 任何校验失败会在日志里定位具体动作/步骤并阻止节点继续运行，避免运行期才暴露配置错误。
- 因此新增动作或技能时，请确保参数命名与表格一致，自检通过后再上线。

## 5. 参数配置

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `skill_sets_dir` | 包默认 `skill_sets` 目录 | 允许覆盖技能 YAML 目录；留空则使用安装目录内置的动作集 |

> 动作执行（笛卡尔移动、关节移动、夹爪）统一使用 `robot_skill_core/constants.py` 中的常量：`DEFAULT_ACTION_TIMEOUT_SEC = 30.0` 秒、`DEFAULT_SPEED_SCALE = 1.0`。若需调整默认行为，请修改该文件即可全局生效。
## 6. 异常处理

| 异常类型 | 处理策略 |
|---------|---------|
| IK 失败 | driver 返回失败，skill 可尝试调整目标姿态或报告失败 |
| 移动超时 | 停止当前技能，执行安全位姿，报告失败 |
| 夹爪超时 | 检查夹爪状态，决定重试或报告失败 |
| 抓取失败 | 检测到夹爪未夹住物体，执行放弃流程 |

## 7. 约束与限制

- 当前阶段暂不考虑外部障碍规避，主要关注任务序列执行
- 关节位置/速度限制由 robot_driver 保证（各关节速度上限约 10 rad/s）
- 轨迹平滑由 robot_driver 保证

## 8. 实施方案与目录规划

为保证 `robot_skill` 可以快速接入现有 ROS2/colcon 工作流，规划如下目录与文件骨架（与 `robot_driver` 风格保持一致）：

```
src/robot_skill/
├── CMakeLists.txt                  # ament_cmake 包装，方便 colcon build
├── package.xml                     # 声明依赖（rclpy、geometry_msgs 等）
├── setup.py / setup.cfg            # Python 节点安装入口
├── resource/robot_skill            # ament 索引用占位文件
├── config/robot_skill_config.yaml  # 节点参数（位姿偏置、超时等）
├── launch/robot_skill.launch.py    # 启动 rclpy 节点并加载参数
├── scripts/robot_skill_node.py     # `#!/usr/bin/env python3` 启动脚本
├── src/robot_skill/
│   ├── __init__.py
│   ├── node.py                     # Node 类封装（订阅视觉 / 发布 driver 指令）
│   └── state_machine.py            # 抓取/放置等技能状态机骨架
├── skill_sets/                     # 技能配置目录（通用动作 + 具体技能）
├── tests/                          # pytest：参数解析、状态机流转
└── start_robot_skill.sh            # 启停包装脚本
```

### 8.1 start_robot_skill.sh 设计

- **功能定位**：统一拉起/关闭 `robot_skill` 节点，简化与 driver 一致的日志、环境设置体验。
- **命令参数**：仅保留 `--start`（默认）与 `--stop`，其中 `--stop` 负责清理通过 launch 或脚本启动的节点；默认行为为启动节点。
- **实现要点**：
  - 自动检测当前 Shell，选择 `/opt/ros/humble/setup.{bash|zsh}` 并 source；同时 source 工作区 `install/setup.bash`。
  - 日志输出统一写入 `log/robot_skill/robot_skill_YYYYmmdd_HHMMSS.log`，便于排查。
  - `--start` 流程中调用 `ros2 launch robot_skill robot_skill.launch.py`；`--stop` 流程通过 `pkill -f` 方式关闭历史进程并提示状态。
  - 预留扩展位：后续如需守护/参数切换，可在脚本中继续补全。

该目录及脚本将在本迭代中创建，形成最小可运行的 `robot_skill` 节点骨架，后续可逐步填入实际技能逻辑。

### 8.2 技能配置与 SkillSequence Action

- 在 `skill_sets/` 目录中维护：
  - `general_skill.yaml`：定义通用动作库 `common_actions`，每个动作包含 `id`、`type`（如 `safety_pose`、`cartesian_move`、`gripper`、`joint_move` 等）、`desc`、默认 `params`（速度、Pose、夹爪命令等）。
  - 具体技能文件（例如 `record_object.yaml`）：按步骤列出需要执行的动作。步骤可以通过 `action` 字段引用 `common_actions` 中的模板，并使用 `params_override`（或同名字段）覆盖个别参数；也可直接自定义 `type + params`。
- 节点加载技能时先解析 `general_skill.yaml`，再读取目标技能文件，将通用动作与技能步骤合并为最终执行序列，按顺序调用 `/robot_driver/action/robot`、`/joint`、`/gripper`、`/safety_pose` 等 action。
- 对外暴露 `/robot_skill/action/skill_sequence`（action 名 `SkillSequence`）：Goal 包含 `skill_id`（对应技能文件）、`targets[]`（可选目标位姿数组）、`context_id`（任务标识）；Result 提供 `success`、`error_code`、`message`、`steps_executed`；Feedback 输出 `step_index`、`step_id`、`step_desc`、`progress`、`detail`。这一接口让上层仅需传技能 ID，底层即可串行执行预定义动作集并实时反馈。
