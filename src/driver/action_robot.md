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
| Action 类型 | `robot_driver/action/RobotCommand`（待新增） |
| 参考坐标系 | 固定 `base_link`，不支持其他 frame |
| 末端姿态模式 | 三种模式，详见下文 |
| 零重力 | 执行前若处于零重力，必须自动退出 |
| 速度语义 | Goal 中提供单个 `speed_scale`（0~1），结合关节差值推算平滑时间 |
| IK 失败 | 直接返回 `IK_FAILED`；无可行替代，只能依赖硬件/SDK |
| 反馈 | 每条反馈包含关节状态 + 末端 FK pose |
| 抢占 | 暂不做自动抢占；支持 Action cancel 即可 |

## 姿态模式说明

Action Goal 中将携带一个 `uint8 xyz_mode`，含义如下：

1. **`XYZ_MODE_USE_PARAM`**（默认）  
   - 行为与节点参数 `xyz_only_mode` 一致：当参数为 true 时只改 XYZ，姿态保持当前值；为 false 时执行完整位姿。
   - 适用于与现网行为一致的调用方。
2. **`XYZ_MODE_POSITION_ONLY`**  
   - 无论节点参数如何，始终忽略 Goal 中的 orientation，仅修改 XYZ。适合只关心末端位置的任务。
3. **`XYZ_MODE_FULL_POSE`**  
   - 强制按照 Goal 中的 orientation 求 IK，即使节点参数开启了 xyz-only。适合需要精确姿态的抓取/装配。

三种模式互斥，可通过 Goal 显式选择；未填写时走模式 1。

## 速度/时间语义

- Goal 携带 `speed_scale`（`float32`，0~1）；值为 0 或非有限数时回退到 1.0。
- 行进时间由关节差值与 `joint_command_rate_limit × speed_scale` 推算，沿用 `/robot_driver/action/joint` 的插值与限速逻辑，保证平滑。
- 不再额外暴露“目标时长”字段，避免与底层 ramp 冲突。

## 零重力处理

- 在发送 JointCommand 前调用 `ZeroGravityManager`：
  - 若当前处于零重力，一律先关闭；
  - Goal 可包含 `exit_zero_gravity`（bool），若为 true 则即便当前未开启也强制执行一次“关闭”流程，确保状态一致。

## IK 行为

- 通过现有 `HardwareCommander.solve_ik()` 解算。
- 若硬件/SDK不可用或 IK 无解，Action 直接以 `IK_FAILED` 中止，`result.last_error` 描述原因。
- 无软模拟方案：若需在无硬件环境下运行，仍需依赖 dummy commander 或提前注入离线 IK 解。

## 反馈与结果

- 反馈阶段划分：`planning` → `waiting_joint` → `joint/<phase>`（透传 JointCommand）→ `complete`。
- 每条反馈除 JointState 与 Contact 数据外，还需附带当前末端 FK 位姿（position + orientation）。  
  *实现建议：从 `HardwareCommander` 读取最新关节状态，调用 FK（若可用）或复用最近一次 `command_cartesian_pose` 的缓存；当 FK 不可用时，可在反馈中将 pose 设为 NaN 并记录 `last_error`。*
- Result 中同样包含最终关节状态、接触信息以及最终末端 FK。

## 取消与抢占

- 仅支持 Action 客户端显式 cancel：收到 cancel 请求后转发给内部 `/robot_driver/action/joint` goal，并在结果中返回 `result_code='CANCELED'`。
- 暂不实现“新 Goal 自动抢占旧 Goal”的逻辑，以免干扰上层调度；如需抢占，由调用方 cancel 后发送新 Goal。

## 后续实现要点

1. **接口文件**：在 `action/RobotCommand.action` 中定义上述字段。
2. **参数**：`DriverParameters` 新增 `robot_command_action`，默认 `/robot_driver/action/robot`。
3. **节点逻辑**：
   - 新建 ActionServer，执行流程：校验 → IK → 构造 JointCommand Goal → 透传反馈。
   - Topic `/robot_driver/robot_command` 在迁移期可作为 thin wrapper，后续移除。
4. **文档与测试**：更新 `robot_driver.md`、编写针对 IK 失败/零重力切换/反馈格式的单元或集成测试。

此文档会在需求变动时同步更新，确保实现与期望保持一致。***
