# /robot_driver/action/robot 设计记录

本文档整理了为 `/robot_driver/action/robot` 新增笛卡尔动作接口的需求，以便后续实现与联调。

## 目标与范围

- 以 Action 形式接受末端目标位姿（`PoseStamped`），统一由 `robot_driver` 节点内部转换为关节命令。
- 绑定基坐标系 `base_link`，调用方必须在发送 Goal 前自行完成坐标系转换。
- 未来 `/robot_driver/robot_command` Topic 将删除；短期内 Topic 会复用本 Action 客户端作为语法糖。
- 本文档仅覆盖动作接口协议、执行流程与反馈需求，硬件细节沿用现有 `HardwareCommander`。

## 关键信息

| 项目 | 约定 |
| --- | --- |
| Action 名称 | `/robot_driver/action/robot` |
| Action 类型 | `robot_driver/action/RobotCommand`（Goal/Result/Feedback 见下） |
| 参考坐标系 | 固定 `base_link`，不支持其他 frame |
| 姿态模式 | 固定为“完整姿态”（必须提供目标四元数） |
| 零重力 | 执行前若处于零重力，必须自动退出（无配置项，强制执行） |
| 速度语义 | Goal 中提供单个 `speed_scale`（0~1），结合关节差值推算平滑时间（默认 0.9） |
| IK/FK | 统一通过 `KinematicsSolver`（封装 I2RT `self._kinematics`）提供；无 SDK 时返回缓存/NaN |
| 反馈 | 每条反馈包含关节状态 + `KinematicsSolver` 产出的末端 FK pose；并有独立的 `/robot_driver/end_effector_pose` Topic 由后台线程轮询 `HardwareCommander` 当前关节状态并通过同一个 `KinematicsSolver` 发布 FK（频率与 `/joint_states` 对齐） |
| 抢占 | 暂不做自动抢占；支持 Action cancel 即可 |

## 姿态要求

- `/robot_driver/action/robot` 仅支持“完整姿态”模式：Goal 中必须提供合法的 orientation 四元数，驱动会严格按该姿态求 IK。
- 不再暴露 xyz-only 选项；上层如果只想移动 XYZ，应在提交 Goal 前自行把 orientation 设置为当前姿态。

## 速度/时间语义

- Goal 携带 `speed_scale`（`float32`，0~1）；值缺省或无效时使用固定默认值 `0.9`。
- 行进时间由关节差值与 `joint_command_rate_limit × speed_scale` 推算，沿用 `/robot_driver/action/joint` 的插值与限速逻辑，保证平滑。
- Goal 可携带 `timeout`，若缺省则内部固定使用 15 s 作为软超时；超时仍然走 JointCommand 的超时/取消流程。
- 不再额外暴露“目标时长”字段，避免与底层 ramp 冲突。

## 零重力处理

- 在发送 JointCommand 前由 `ZeroGravityManager` 自动检测并退出零重力；此流程没有可配置的 Goal 参数，所有笛卡尔动作都必须在常规力矩模式下执行。

## IK/FK 行为

- 新增 `driver/hardware/kinematics_solver.py`（或同目录文件）封装 `KinematicsSolver`，统一管理 IK/FK 调用。内部直接复用 `HardwareCommander` 初始化的 I2RT `self._kinematics` 对象。
- Action、实时末端 pose Topic、以及其它需要 IK/FK 的模块只与 `KinematicsSolver` 交互，不直接访问 `_kinematics`，便于统一日志与 fallback。
- 若硬件/SDK不可用，solver 回退到“返回最后一次命令的缓存姿态或 NaN 并记录错误”；IK 无解时 Action 以 `IK_FAILED` 中止，`result.last_error` 写明原因。

- 反馈阶段划分：`planning` → `waiting_joint` → `joint/<phase>`（透传 JointCommand）→ `complete`。
- 每条反馈除 JointState 与 Contact 数据外，还需附带当前末端 FK 位姿（position + orientation），由 `KinematicsSolver` 计算；若 FK 不可用则反馈 NaN 并记录 `last_error`。
- Result 中同样包含最终关节状态、接触信息以及最终末端 FK。
- `/robot_driver/end_effector_pose`（`geometry_msgs/PoseStamped`）由专门的发布器在后台周期性读取 `HardwareCommander` 当前关节状态并调用 `KinematicsSolver` 求 FK 后发布，周期由 `joint_state_rate` 推导，Action 反馈与该 Topic 使用同一个 solver，保持 pose 数据一致。

## 取消与抢占

- 仅支持 Action 客户端显式 cancel：收到 cancel 请求后转发给内部 `/robot_driver/action/joint` goal，并在结果中返回 `result_code='CANCELED'`。
- 暂不实现“新 Goal 自动抢占旧 Goal”的逻辑，以免干扰上层调度；如需抢占，由调用方 cancel 后发送新 Goal。

## 后续实现要点

1. **接口文件**：在 `action/RobotCommand.action` 中定义上述字段。
2. **参数**：`DriverParameters` 新增 `robot_command_action`，默认 `/robot_driver/action/robot`。
3. **节点逻辑**：
   - 新建 ActionServer，执行流程：校验 → `KinematicsSolver` 求 IK → 强制退出零重力 → 构造 `JointCommand.Goal`（`relative=false`、继承 `speed_scale` 与 `timeout`）→ 经 `ActionClient(JointCommand)` 执行动作 → 透传反馈/结果（并在反馈中附带 FK pose）。
   - Topic `/robot_driver/robot_command` 的迁移在 Action 完成后另行规划（本阶段不改）。
   - 所有 Action 执行日志额外落到 `log/action_robot/action_robot.log`（工作区根目录下的 `log/action_robot/`），记录 IK 输入/输出、zero-gravity 切换、JointAction 结果等关键事件。
4. **文档与测试**：更新 `robot_driver.md`、编写针对 IK 失败/零重力切换/反馈格式的单元或集成测试；并在文档中记录默认超时/速度。

此文档会在需求变动时同步更新，确保实现与期望保持一致。***
- `RobotCommand.action` 字段定义（约定）：
  - **Goal**：
    - `geometry_msgs/PoseStamped target`（frame_id=`base_link`，position+orientation 必填）
    - `float32 speed_scale`（默认 0.9）
    - `builtin_interfaces/Duration timeout`（默认 15 s）
    - `string label`
  - **Result**：
    - `bool success`
    - `string result_code`
    - `string last_error`
    - `sensor_msgs/JointState final_state`
    - `sensor_msgs/JointState target_state`
    - `robot_driver/JointContact[] contact_state`
    - `geometry_msgs/PoseStamped final_pose`
  - **Feedback**：
    - `string phase`
    - `float32 completion_ratio`
    - `builtin_interfaces/Duration remaining_time`
    - `sensor_msgs/JointState current_state`
    - `sensor_msgs/JointState target_state`
    - `robot_driver/JointContact[] contact_state`
    - `geometry_msgs/PoseStamped current_pose`
    - `string last_error`
