# robot_driver.md

该文档用于梳理 `robot_driver`（设备下发/上报驱动）的需求、边界与验收标准。

## 背景
- `robot_driver` 位于 `driver/src/robot_driver.py`（仓库路径 `L2/src/driver/src/`），承担机械臂/夹爪等执行器的底层通信、状态采集与命令下发。
- 上层模块（robot_skill、app 等）依赖该节点提供稳定的实时接口。

## 节点职责（目标）
1. 发布 `/joint_states`
   - 采集机械臂每个关节/电机状态并转成 `sensor_msgs/JointState`，供 MoveIt、RViz、上层控制统一使用；状态采集在独立定时器中运行，即使机械臂正在运动也持续输出，不与控制线程互斥。
2. 启动与管理 CAN 接口
   - 枚举/检测可用的 gs_usb 适配器，完成初始化、健康监控与掉线重连，确保硬件链路稳定。
3. 承载标准控制指令
   - 支持关节空间与笛卡尔空间的目标位姿/姿态指令，负责插值、限速与安全联锁后再写入 CAN。
4. 安全态归位
   - 监控指令心跳：超过 60 s 未收到上游指令即执行安全归位（`SAFE_POSE`），归位完成后默认切换至零重力模式，方便人工干预。
   - 订阅 `/robot_driver/safety_stop`，收到后立即执行安全回归流程但不自动切换零重力（留给人工决定）。
5. 零重力模式开关
   - 暴露接口（service/action）以启用或关闭零重力模式，并同步更新状态上报与安全限制。
   - 若节点处于零重力模式且收到新的运动指令，需先自动退出零重力模式，再恢复常规控制链路。
6. 生命周期管理
- 节点启动由 `driver/start_robot_driver.sh` 负责触发，脚本需拉起驱动、完成初始化并开始发布话题；关闭时安全停车、断开总线，保证流程幂等。
7. 诊断与告警
   - 通过 `diagnostic_msgs/DiagnosticArray` 或自定义话题上报 CAN 状态、力矩/温度/电流异常、模式切换结果。
8. 故障恢复
   - 对通信超时、驱动故障、电源波动等事件执行自动重试或降级策略，并将状态同步给上层。
9. 自检与标定入口
   - 提供触发自检、零位校准、夹爪检查的接口，确保上层在收到“ready”前不会下发运动任务。
10. 日志与追溯
  - 记录关键指令、模式切换、故障上下文（时间戳、指令来源、传感器状态）以便排障，并统一写入工作区根目录 `log/`（相对路径）下的子目录，便于集中收集与分析。

## ROS 接口设计

### 发布的 Topic
| 话题 | 类型 | 频率 | 说明 |
| --- | --- | --- | --- |
| `/joint_states` | `sensor_msgs/JointState` | 30 Hz（可配置） | 仅包含标准位置/速度/力矩（或电流）字段，确保与 MoveIt、RViz、Foxglove 兼容。 |
| `/robot_driver/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 1 Hz | 汇报命令执行状态、心跳/安全态、零重力模式、CAN 健康、温度/电流/自检等扩展信息，集中展示机械臂健康数据。 |
| `/robot_driver/end_effector_pose` | `geometry_msgs/PoseStamped` | 与 `/joint_states` 同频率 | 由独立 `EndEffectorPosePublisher` 以后台线程周期性读取 `HardwareCommander` 当前关节状态（通过 `KinematicsSolver` 求 FK）并发布，周期由 `joint_state_rate` 推导，frame 固定 `base_link`，与 `/robot_driver/action/robot` 反馈的 pose 保持数据来源一致。 |

### 订阅的 Topic
| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/robot_driver/robot_command` | `geometry_msgs/PoseStamped` | 机械臂基坐标系（默认 `base_link`）下的笛卡尔目标，`Pose.position` 为 XYZ，`orientation` 提供单位四元数；驱动执行 IK → 关节插值，并参考 SAFE_POSE 同款 ramp 逻辑自动按位移估算 0.25–5 s 的平滑运动。 |
| `/robot_driver/safety_stop` | `std_msgs/Empty` | 收到立即抢占当前运动、执行安全归位，但不切换零重力模式。 |
| `/robot_driver/joint_command` | `sensor_msgs/JointState` | 直接指定一个或多个关节位置/速度/力矩；用于调试、标定或单轴测试，节点始终订阅该话题。 |

> 以上三个 topic 名称固定，不再暴露 ROS 参数，节点在启动后会一直订阅它们。

`/robot_driver/robot_command` 示例：

```yaml
header:
  frame_id: base_link
  stamp: {sec: 0, nanosec: 0}
pose:
  position: {x: 0.35, y: 0.0, z: 0.32}
  orientation: {x: 0.0, y: 0.707, z: 0.0, w: 0.707}
```

- `frame_id` 默认 `base_link`，若使用其他坐标系需在 TF 中可解析；
- orientation 需是规范化四元数，若只关注 XYZ 可启用 `xyz_only_mode`，节点将沿用当前姿态；
- `position`（x/y/z）可达范围 *（基于 `robot_description.yaml` 与 `yam.urdf` 在关节极限内采样得到，仅反映几何工作空间，未考虑自碰撞与环境约束）*：
  - x ≈ `[-0.61, 0.60]` m，y ≈ `[-0.59, 0.61]` m，z ≈ `[-0.37, 0.72]` m；
  - 等价地，末端到 `base_link` 原点的距离 r ≈ `[0.04, 0.73]` m；
  - 驱动当前不会对 x/y/z 再做软件限幅，若目标超出可达区或对应姿态无 IK 解，将在日志中打印 `IK failed for target pose`，机械臂保持不动；
  - 实际安全工作空间应由上层（如 `robot_skill`）结合自碰撞、工装/桌面等因素进一步收紧，一般建议只在机器人底座周围"安全盒"内规划目标点；
- `orientation` 取值说明：
  - 使用单位四元数 `{x, y, z, w}` 表示姿态，四个分量数值通常落在 `[-1.0, 1.0]` 区间内；
  - 驱动在 `_quaternion_to_matrix()` 中会自动对四元数做归一化（除非传入全 0），因此只要是合法四元数即可；
  - 代码层面无额外“角度范围”限制，真正的限制来自关节位置极限与几何可达性——某些位置 + 姿态组合虽然消息可以发出，但可能求不出 IK 解；
  - 若在配置中启用 `xyz_only_mode=true`，则会忽略消息中的 `orientation`，只修改末端位置，姿态沿用当前值。
- 驱动会根据“当前关节-目标关节”最大夹角自动算 ramp 速度（同 `/robot_driver/safety_stop`），无需填写时间戳；若仍以 10–30 Hz 推送命令，最新命令会抢占旧命令。

`/robot_driver/joint_command` 示例（position 模式）：

```yaml
name: [joint1, joint3]
position: [0.5, 1.2]
velocity: []
effort: []
```

- `name` 必须与 `/joint_states` 中的名称一致；
- 未列出的关节保持当前值；
- 若 `joint_command_mode` 设为 velocity/effort，则分别读取对应字段并忽略其余字段；
- 关节位置会自动参考 `robot_description.yaml` 中的 `position_limits_rad` 做限幅，防止越界指令写入硬件；
- 该接口多借鉴 I2RT SDK 的 `command_joint_pos`/`command_joint_vel` 能力，适合单轴标定、零位校准或 I2RT 集成测试。
- TODO: 现场测试发现部分关节在特定姿态下存在机械耦合，导致其它关节的可达区间收窄。当前 `/robot_driver/joint_command` 仍按静态限位执行，后续需要为单轴扫描/调试模式增加“姿态相关限幅”或互锁逻辑。

### 服务与动作
| 名称 | 类型 | 用途 | 调用示例 |
| --- | --- | --- | --- |
| `/robot_driver/service/zero_gravity` | Service (`std_srvs/SetBool`) | 零重力模式开关，快速切换机械臂力矩控制状态。 | `ros2 service call /robot_driver/service/zero_gravity std_srvs/srv/SetBool "{data: true}"` |
| `/robot_driver/service/self_check` | Service (`std_srvs/Trigger`) | 触发自检流程：读取关节状态、回到安全位、校验到位情况。 | `ros2 service call /robot_driver/service/self_check std_srvs/srv/Trigger` |
| `/robot_driver/action/follow_joint_trajectory` | Action (`control_msgs/action/FollowJointTrajectory`) | 关节轨迹执行器，接收轨迹点序列并执行；支持反馈、取消与抢占。 | `ros2 action send_goal /robot_driver/action/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: ['joint1'], points: [{positions: [0.5], time_from_start: {sec: 1}}]}}" --feedback` |
| `/robot_driver/action/joint` | Action (`robot_driver/action/JointCommand`) | 点动/单段关节控制语法糖，允许指定任意子集的关节（含 `gripper`）并设置 `speed_scale` 等选项；自动在需要时退出/恢复零重力，内部仍复用 `FollowJointTrajectory` 执行链路。 | `ros2 action send_goal /robot_driver/action/joint robot_driver/action/JointCommand "{target: {joint_state: {name: ['joint2','joint3','gripper'], position: [-0.4,0.55,0.018]}, relative: false, speed_scale: 0.6}, options: {timeout: {sec: 5}, label: 'fine_adjust'}}" --feedback` |
| `/robot_driver/action/safety_pose` | Action (`robot_driver/action/SafetyPose`) | 包装：加载 `safe_pose_config.yaml` → 用固定 JointAction（`relative=false`、`speed_scale=0.8`、`timeout=10s`）回 SAFE_POSE，再等待/校验；执行前自动确保零重力关闭，唯一参数是“是否在结束后重新打开零重力”，结果统一记录 `L2/log/`。 | `ros2 action send_goal /robot_driver/action/safety_pose robot_driver/action/SafetyPose "{enable_zero_gravity_after: true}" --feedback` |
| `/robot_driver/action/reset` | Action | 编排：调用 `safety_pose` 成功后再调用 `zero_gravity`，实现"回安全位并开零重力"。 | 设计中 |
| `/robot_driver/action/robot` | Action (`robot_driver/action/RobotCommand`) | 笛卡尔控制：接收 `PoseStamped`（frame=`base_link`）→ `KinematicsSolver` 求 IK → 构造 `/robot_driver/action/joint` goal 执行；反馈/结果附带末端 FK pose。`speed_scale` 默认 0.9，软超时默认 15 s，并在执行前自动退出零重力。 | `ros2 action send_goal /robot_driver/action/robot robot_driver/action/RobotCommand "{target: {header: {frame_id: base_link}, pose: {position: {x: 0.35, y: 0.0, z: 0.32}, orientation: {x: 0.0, y: 0.707, z: 0.0, w: 0.707}}}" --feedback` |
| `/robot_driver/action/gripper` | Action (`robot_driver/action/GripperCommand`) | 夹爪/末端执行器控制，底层复用关节执行器（单通道或映射到 follow_joint_trajectory action）。 | 设计中 |

> **注意**：当前笛卡尔控制通过 Topic `/robot_driver/robot_command` 实现，未来会迁移至 `/robot_driver/action/robot`，两者将在过渡期共存；Topic 接口将成为 Action 的语法糖，内部调用 Action client。

### 接口分层（结构调整规划）
底层暴露 *两个* 原语接口，其他 Topic/Action 均在其之上包装：

1. `ZeroGravityService` → `/robot_driver/service/zero_gravity`
   - 职责：快速切换零重力/恢复力矩；同步返回结果。
   - 实现：通过 `ZeroGravityManager` 管理状态与 ROS 服务接口。
2. `FollowJointTrajectory` → `/robot_driver/action/follow_joint_trajectory`
   - 职责：统一的关节轨迹执行器，兼容单个或多个关节；提供反馈/取消，内部串行化 commander 的 `command_joint_positions` 调用。
   - 实现：由 `MotionController.execute_trajectory()` 处理，支持轨迹抢占与取消。

在此基础上构建上层接口：

1. `/robot_driver/action/safety_pose`
   - 流程：`STOP` → 读取 `safe_pose_config.yaml` → 以固定 JointAction（`relative=false`、`speed_scale=0.8`、`timeout=10s`）触发 `/robot_driver/action/joint`，动作前强制关闭零重力 → 等待/校验，随后根据 Goal 中 `enable_zero_gravity_after` 决定是否重新打开零重力。
   - 用途：看门狗、人工触发、测试脚本，共享同一执行器保证互斥，并统一记录在 `L2/log/robot_driver/`。
   - 当前状态：已实现，Action 类型为 `robot_driver/action/SafetyPose`，由 `robot_driver` 节点直接发布。
2. `/robot_driver/action/reset`
   - 流程：发送 `safety_pose` goal 成功 → 调用 `ZeroGravityService` 重新开启零重力。
   - 用途：安全停机后的"回位+漂浮"一站式指令。
   - 当前状态：规划中，将复用 `safety_pose` action 与 `zero_gravity` service。
3. `/robot_driver/action/robot`
   - 流程：接收笛卡尔目标 → `KinematicsSolver` 求 IK → 构造 `/robot_driver/action/joint` goal（强制退出零重力、限速插值）→ 透传 JointCommand 反馈/结果并附加末端 FK pose。
   - 用途：高层抓取/落位接口，便于 MoveIt/脚本统一调用；Topic `/robot_driver/robot_command` 将在过渡期内通过 thin wrapper 调用该 Action。
   - 当前状态：Action 已实现；Topic 版本仍保留但内部改为调用 Action（规划中）。
4. `/robot_driver/joint_command`（Topic）
   - 实现：对关节控制的轻量封装，单点命令由 `MotionController.handle_joint_command()` 处理并转发给 commander。
   - 当前状态：已实现，直接调用 `HardwareCommander` 的关节控制能力。
   - Action 版本：`/robot_driver/action/joint`，提供同等能力且支持反馈、抢占、zero-gravity 自动退出与 `contact_state` 触碰检测。
5. `/robot_driver/robot_command`（Topic）
   - 实现：接收笛卡尔目标，内部通过 `MotionController` 执行 IK + 插值，底层调用 commander。
   - 当前状态：已实现，未来将成为 `/robot_driver/action/robot` 的语法糖。
6. `/robot_driver/safety_stop`（Topic）
   - 实现：最高优先级触发，收到消息立即调用 `CommandWatchdog.handle_safety_stop()` 抢占当前运动并执行安全归位。
   - 当前状态：已实现，由 `CommandWatchdog` 统一管理安全停机逻辑。

优先级/互斥策略：
- `Action > Topic`：同一能力若同时提供 Action 与 Topic（例如 follow_joint_trajectory/robot），Action 请求优先；Topic 层未来将成为 Action 的语法糖。
- `最新命令抢占旧命令`：同一接口上，新命令会通过 `threading.Event` 机制抢占旧命令并从当前姿态续跑，保持单执行链路。
- `safety_stop` 最高优先级：触发后通过 `CommandWatchdog` 立即抢占其它一切运动，执行安全归位流程。
- `gripper`：规划为 `/robot_driver/action/gripper`，底层也将通过 `FollowJointTrajectory`（或映射到特定关节）执行，确保所有执行互斥策略一致。

> 备注：统一的 `FollowJointTrajectory` 让"落位"、"常规轨迹"、"单轴 Debug"都走同一执行链路、共享互斥与反馈，减少重复线程，实现优雅的抢占管理。

### `/robot_driver/action/joint` 参数约定

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `target.joint_state.name` | `string[]` | 要控制的关节名子集，必须与 `/joint_states` 中的名称一致；可以只填需要点动的关节，夹爪统一使用 `gripper`。 |
| `target.joint_state.position/velocity/effort` | `float[]` | 根据当前 `joint_command_mode` 读取对应字段；示例默认用 position。未提供的字段会被忽略。 |
| `target.relative` | `bool` | `true` 时在当前姿态上叠加 `joint_state` 中的增量（rad）；`false` 时表示绝对目标。 |
| `target.speed_scale` | `float` | 0‒1 的限速倍率，乘以参数 `joint_command_rate_limit` 后作为本次 ramp 的最大步长；例如默认 0.5 rad/s × 0.6 = 0.3 rad/s，用于平滑点动。 |
| `options.timeout` | `builtin_interfaces/Duration` | 软超时；到期尚未完成则抢占并返回失败。 |
| `options.label` | `string` | 可选标签，写入日志/反馈，便于上层跟踪命令来源。 |

**反馈 / 结果字段**

- `phase`：`queued` → `ramping` → `settling` 状态机，便于 UI/日志展示。
- `completion_ratio`：0‒1，按目标关节中最大剩余位移比例计算。
- `current_state`：最新关节状态（含 `name/position/velocity/effort`）。
- `target_state`：本次命令的目标关节值（与 Goal 对齐，便于比对）。
- `contact_state`：`map<string, JointContact>`，对所有正在控制的关节输出状态：
  - `in_contact`：速度趋近 0 且 effort 超阈值时为 `true`，用于判断夹爪/关节是否触碰到物体或发生阻挡；夹爪由 `gripper` 条目报告。
  - `effort`/`velocity`/`position`：最近一次采样值，便于上层做“握持完成”或“轴被阻塞”判定。
  - `note`：可选文本，记录限幅、碰撞、contact 触发说明等。
- `last_error`：若执行中出现限幅/IK 失败等异常，这里给出最后一条可读信息。

Action Result 会携带与最终 Feedback 相同的 `current_state` 与 `contact_state`，方便调用方在结果回调中直接决策（例如检测 `gripper` 是否已经 `in_contact=true` 且 effort 达到阈值）。

## 实现技术方案（对标 l1_stage1_device）
1. 启动链路（方案稳定，暂不调整）
   - `driver/start_robot_driver.sh`：仿照 `L1/start_stage1.sh`，完成以下步骤：
     1. 解析 `--can canX`、`--xyz-only`、`--zero-gravity` 等参数；
     2. 调用 `I2RT/i2rt/scripts/reset_all_can.sh` 强制清总线；
     3. 执行 `python3 -m driver.can_interface_manager --ensure --interface canX` 以确认 gs_usb 接口、比特率；
 4. 设定 `PYTHONPATH`（挂载 I2RT SDK）、ROS 环境、`L2/log/robot_driver/robot_driver.log` 输出，并以 `ros2 launch driver robot_driver.launch.py` 方式拉起节点；
     5. 统一杀掉旧进程 `robot_driver`，保证脚本幂等。
     6. 使用方式：在仓库根目录运行 `./src/driver/start_robot_driver.sh --can can0` 即可后台启动，若需要实时日志可追加 `--follow-log`，否则默认把输出写入 `L2/log/robot_driver/robot_driver_<timestamp>.log` 后静默运行。若需调试前台运行，可追加 `--foreground`。

2. 模块分层
   - `can_interface_manager`：借鉴 `l1_stage1_device/cartesian/can_monitor.py` 的思路，封装 gs_usb 设备检测、`ip -details link show` 解析、`I2RT/i2rt/scripts/reset_all_can.sh` 调用与 1 Mbps 配置校验，暴露 `ensure_ready(required=[canX])` API。
   - `hardware_commander`：类似 `CartesianCommander`，包装 I2RT SDK 的 joint/pose 读取、轨迹插值、零重力模式切换（SetZeroGravity）、回零/归位等原语，确保上层 ROS 节点不直接触碰 SDK。
   - `robot_driver`：ROS 2 进程主体，进一步拆成三个内部组件：
     - `state_publisher`：独立 callback group 或 executor，周期性读取 commander 缓存并发布 `/joint_states`、TF 以及 `/robot_driver/diagnostics` 中的运行指标，确保读路径只持有读锁、不阻塞控制线程。
     - `JointControler`：承载 `/robot_driver/robot_command`、`/robot_driver/joint_command` 以及 Action 接口（`/robot_driver/robot_command_action`、`/robot_driver/gripper_command_action`），负责命令抢占、IK + 插值、限速、写入 commander。
     - `command_watchdog`：挂在控制线程侧的子模块，维护 `last_command_stamp`；若 60 s（或 `command_timeout_s`）未收到控制指令则主动调用 `JointControler` 的安全归位流程，并在确认机械臂已落位 SAFE_POSE 后再触发 `hardware_commander.set_zero_gravity(true)`，也负责 `/robot_driver/safety_stop` 的统一抢占收敛。

   3. 指令调度与安全
      - 命令队列：沿用 Stage1 的“单工作线程+可抢占”模型，使用 `queue.Queue` 存储笛卡尔/关节指令，任何新区指令会清空旧队列并打断当前插值；
      - 插值策略：
        - 笛卡尔目标：通过 IK 求解后在关节空间作线性插值（同 Stage1 `_interpolate_to_goal`）；
        - Action 目标：`/robot_driver/robot_command_action` 直接复用笛卡尔插值并在反馈中输出进度/接触标志；`/robot_driver/gripper_command_action` 走夹爪状态机，支持力阈/行程阈结束条件；如需 MoveIt/ControllerManager 兼容，可继续保留 FollowJointTrajectory 执行链路，在内部与抢占机制共存。
        - 关节直控：当启用 `/robot_driver/joint_command` 时，节点根据消息中的关节名替换当前姿态中对应分量，再应用限速/限幅并写入硬件，逻辑复用 Commander 的 `command_joint_pos/vel` 能力。
   - 心跳/超时：由 `command_watchdog` 维护 `last_command_stamp`，若在 `timeout_s`（固定 60 s）内未收到任何控制指令，则触发安全归位；当确认机械臂到达 `SAFE_POSE` 后再启用零重力模式，重新收到命令时即刻退出零重力并恢复常规控制。
   - `/robot_driver/safety_stop`：节点订阅后立即抢占当前运动；若当前已处于零重力模式，先调用 `hardware_commander.set_zero_gravity(false)` 以恢复常规控制，再执行 `move_to_safe_pose()`，待确认 SAFE_POSE 落位成功后再次启用零重力；若原本不在零重力模式，则直接回到安全位并在完成后开启零重力。

4. 零重力模式钩子
   - 在 Commander 初始化时读取参数 `zero_gravity_default`，并提供 `set_zero_gravity(bool)` API，内部调用 I2RT SDK 的零重力接口（若缺失则通过写寄存器实现），状态变更同步到 `/robot_driver/diagnostics`。
  - 服务端做互斥处理：切模式期间暂停指令执行，并在成功/失败时写日志到 `L2/log/robot_driver/`。

5. 日志与诊断
- 日志滚动策略：`driver/start_robot_driver.sh` 创建 `L2/log/robot_driver/robot_driver_<date>.log`，ROS 节点内部使用 `logging` 模块写同一路径；
- 关键信息（CAN 重连、零重力切换、`/robot_driver/safety_stop` 触发、心跳超时、SAFE_POSE 校验误差）必须双写：一份进入 `/robot_driver/diagnostics`，一份落地日志文件，便于离线排查；
   - 发生异常时自动收集最近一次指令上下文（命令源 topic/action、目标值、IK 耗时）并写入日志。

6. 状态广播与调试
   - 复用 Stage1 中 TF 广播逻辑（`base_link -> tool0` + 可选 `world -> base_link`），确保上层可在 RViz 看到末端姿态；
   - 提供 `ros2 topic echo /robot_driver/diagnostics`、`ros2 action list` 等调试指令，并在 README 中文档化。

7. 状态发布（JointState/TF）
   - 将 Stage1 的 `_publish_tf` 定时器拆分为 `joint_state_timer` + `tf_timer`：
     - `joint_state_timer`：以 30 Hz 读取 I2RT SDK 缓存里的关节角、速度、电流，构造 `JointState` 并发布；读取使用轻量读锁，仅在写命令时短暂加写锁，确保运动与状态采集可并行。
     - `tf_timer`：保持 30 Hz（或配置值）读取末端位姿并广播 TF。
   - 若 SDK 提供 `get_joint_state_cached()` 之类非阻塞 API，则驱动优先使用缓存；否则在 commander 层维护 ring buffer，写命令时顺便更新最新状态，供 JointState 发布线程读取。

## 安全态标定脚本
- 新增 `driver/safe_pose_build.sh`（与其他脚本同级），提供交互式 SAFE_POSE 标定流程，方便把实机姿态写入配置：
  1. 启动后立即调用 `/robot_driver/service/zero_gravity` 进入零重力，提示操作者可手动拖动机械臂至期望安全位。
  2. 脚本阻塞等待回车（或 `ctrl+c` 退出），确保操作者完成调姿后再继续。
  3. 收集当前关节状态：通过 `ros2 topic echo /joint_states --once`（或 commander CLI）读取最新的关节名/角度，并打印到终端让操作者确认。
  4. 操作者确认后，脚本把最新结果写入固定文件 `config/safe_pose_config.yaml`，内容包括 `safe_pose.joints`、metadata 与 `.ready` 标记，便于立即在 `safe_pose_file` 中引用。
- 后续可把生成的 YAML 纳入版本管理或部署包，`command_watchdog` 与 `/robot_driver/safety_stop` 将依赖该 SAFE_POSE 数据。在补充具体参数前，此脚本可反复执行以更新标定结果。


## 配置与动态更新
**配置文件位置**
- 主配置文件：`driver/config/robot_driver_config.yaml`
- 通过 `ros2 launch driver robot_driver.launch.py` 自动加载
- 启动脚本 `start_robot_driver.sh` 支持通过参数覆盖部分配置

**关键参数说明**
| 参数名 | 类型 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `can_channel` | string | `can0` | CAN 总线接口名称 |
| `joint_state_rate` | float | `30.0` | 关节状态发布频率（Hz） |
| `diagnostics_rate` | float | `1.0` | 诊断信息发布频率（Hz） |
| `command_timeout_s` | float | `600.0` | 命令超时时间（秒），超时后触发安全归位 |
| `zero_gravity_default` | bool | `true` | 启动时是否默认开启零重力模式 |
| `xyz_only_mode` | bool | `false` | 笛卡尔控制是否仅修改位置（忽略姿态） |
| `log_dir` | string | `/home/jetson/L2/log/robot_driver` | 日志输出目录（相对于工作区根目录） |
| `safe_pose_file` | string | `config/safe_pose_config.yaml` | SAFE_POSE 配置文件路径 |
| `robot_description_file` | string | `robot_description.yaml` | 机器人描述文件路径（关节限位等） |
| `publish_tf` | bool | `true` | 是否发布 TF 变换 |
| `tf_base_frame` | string | `base_link` | TF 基坐标系名称 |
| `tf_tool_frame` | string | `tool0` | TF 工具坐标系名称 |
| `diag_hardware_id` | string | `robot_driver` | 诊断消息中的硬件 ID |
| `diagnostic_latch_sec` | float | `5.0` | 诊断状态锁存时间（秒） |
| `cartesian_command_rate_limit` | float | `0.15` | 笛卡尔命令速度限制（m/s） |
| `joint_command_mode` | string | `position` | 关节直控模式（position/velocity/effort） |
| `joint_command_rate_limit` | float | `0.5` | 关节直控速度限制（rad/s） |
| `enable_joint_velocity_fuse` | bool | `false` | 是否启用关节速度融合 |
| `can_reset_script` | string | `""` | CAN 总线复位脚本路径（空则跳过） |

**接口名称参数**
| 参数名 | 默认值 | 说明 |
| --- | --- | --- |
| `joint_state_topic` | `/joint_states` | 关节状态发布话题 |
| `diagnostics_topic` | `/robot_driver/diagnostics` | 诊断信息发布话题 |
| `follow_joint_trajectory_action` | `/robot_driver/action/follow_joint_trajectory` | 关节轨迹 action 接口 |
| `joint_command_action` | `/robot_driver/action/joint` | 单段关节 action 接口 |
| `zero_gravity_service` | `/robot_driver/service/zero_gravity` | 零重力服务接口 |
| `safety_pose_action` | `/robot_driver/action/safety_pose` | SAFE_POSE Action 接口 |

**SAFE_POSE 配置**
- `safe_pose_file`：指向具体的 SAFE_POSE YAML 文件
- `safe_pose_fallback`：嵌套参数，包含默认安全位（当文件加载失败时使用）
  - `joint_names`：关节名称列表
  - `positions`：对应的关节位置（弧度）
  - `ready`：标志位，指示该 fallback 是否已校验可用

**启动脚本参数覆盖**
`start_robot_driver.sh` 支持以下命令行参数：
- `--can canX`：指定 CAN 接口（覆盖 `can_channel`）
- `--zero-gravity` / `--no-zero-gravity`：控制启动后是否默认开启零重力
- `--xyz-only` / `--no-xyz-only`：控制笛卡尔命令模式
- `--params <file>`：指定自定义参数文件
- `--follow-log`：后台运行时实时显示日志
- `--foreground`：以前台形式运行，方便调试
- `--stop`：停止当前正在运行的守护进程

**运行时参数更新**
部分参数支持通过 `ros2 param set` 动态更新：
- `command_timeout_s`：实时更新安全心跳超时时间
- 其他参数更新需要重启节点

**配置文件示例**
参考 `driver/config/robot_driver_config.yaml` 查看完整配置示例。

### 目录结构（按功能域聚合）
```
driver/
├── src/driver/
│   ├── app/                 # ROS 入口节点
│   │   └── robot_driver.py       # ROS 2 主节点实现
│   ├── config/              # 参数、SAFE_POSE 解析
│   │   ├── parameter_schema.py        # ROS 参数声明与打包
│   │   └── safe_pose_loader.py        # SAFE_POSE YAML 解析
│   ├── control/             # 运动、Watchdog、状态发布、零重力管理
│   │   ├── joint_controler.py      # 命令处理与轨迹执行
│   │   ├── command_watchdog.py       # 安全超时与 safety_stop
│   │   ├── state_publisher.py        # JointState/TF/诊断发布
│   │   └── zero_gravity_manager.py   # 零重力服务与状态管理
│   ├── hardware/            # CAN、自检、Commander、robot_description
│   │   ├── can_interface_manager.py  # gs_usb 自检与配置
│   │   ├── hardware_commander.py     # I2RT SDK 封装
│   │   └── robot_description_loader.py # 关节限位等元数据
│   ├── safety/              # SAFE_POSE 执行辅助
│   │   └── safe_pose_executor.py     # 安全归位流程封装
│   ├── utils/               # 日志、路径等通用工具
│   │   ├── logging_utils.py          # 日志落盘辅助
│   │   └── path_utils.py             # 路径解析工具
│   └── tools/               # CLI 工具（预留）
├── config/                  # 默认参数 & SAFE_POSE YAML
│   ├── robot_driver_config.yaml      # 主配置文件
│   ├── safe_pose_default.yaml        # 默认安全位
│   └── safe_pose_config.yaml         # 最新 SAFE_POSE（safe_pose_build.sh 自动生成）
├── launch/                  # launch 入口
│   └── robot_driver.launch.py        # 启动配置
├── tests/                   # 单元测试
├── resource/                # ROS 资源索引
├── setup.py / setup.cfg     # 打包 & console_scripts
├── package.xml              # ROS 依赖
├── start_robot_driver.sh    # 实机启动脚本
└── safe_pose_build.sh       # SAFE_POSE 标定脚本
```

**注意**：
- 目录结构已完成重构，新代码在各子目录中，部分旧文件（如 `src/driver/robot_driver.py` 等）仍存在以保持向后兼容，待清理。
- 实际运行时使用 `driver.app.robot_driver` 作为入口。

### 源码模块划分（面向对象）
**核心节点**
- `driver.app.robot_driver.RobotDriverNode`
  - ROS 2 主节点，负责参数声明、组件装配、ROS 接口注册与生命周期管理。
  - 集成 `MotionController`、`StatePublisher`、`CommandWatchdog`、`ZeroGravityManager` 等组件。

**控制层**
- `driver.control.joint_controler.JointControler`
  - 统一处理 `/robot_driver/robot_command`、`/robot_driver/joint_command` 与 `FollowJointTrajectory` action。
  - 集成命令队列、限速、IK 求解、SAFE_POSE 落位能力。
- `driver.control.command_watchdog.CommandWatchdog`
  - 基于 `MotionController` 的活动心跳实现安全超时与 `/robot_driver/safety_stop` 处理。
  - 触发 SAFE_POSE 流程并管理零重力自动切换。
- `driver.control.state_publisher.StatePublisher`
  - 周期读取 `HardwareCommander` 的缓存，发布 `/joint_states`、诊断与 TF。
- `driver.control.zero_gravity_manager.ZeroGravityManager`
  - 封装零重力服务接口 `/robot_driver/service/zero_gravity`。
  - 管理零重力状态，提供便捷的 enable/disable 方法。

**硬件层**
- `driver.hardware.hardware_commander.HardwareCommander`
  - 封装 I2RT SDK/模拟后端，提供线程安全的读写 API。
  - 支持零重力控制、SAFE_POSE 校验、关节与笛卡尔控制。
- `driver.hardware.can_interface_manager`
  - `ensure_ready()` 等方法封装 gs_usb 自检、bitrate 配置与 reset 脚本调用。
- `driver.hardware.robot_description_loader.RobotDescriptionLoader`
  - 解析 `robot_description.yaml` 并暴露关节限位、元数据等信息。

**配置层**
- `driver.config.parameter_schema`
  - `DriverParameters` dataclass 与 `declare_and_get_parameters()` 函数。
  - 集中声明 ROS 参数并打包为类型安全的配置对象。
- `driver.config.safe_pose_loader.SafePoseLoader`
  - 统一 SAFE_POSE YAML 解析、fallback 与就绪校验。

**安全层**
- `driver.safety.safe_pose_executor.run_safe_pose_sequence()`
  - 封装"退出零重力→HALT→SAFE_POSE→等待→校验→可选恢复"的完整链路。
  - 可供控制层/外部工具共用。

**工具层**
- `driver.utils.logging_utils`
  - 日志落盘、格式化等辅助功能。
- `driver.utils.path_utils`
  - 路径解析等跨域工具模块。

### 节点配置（ROS 包）
- `package.xml`
  - `<buildtool_depend>ament_python</buildtool_depend>`、`<exec_depend>` 覆盖 `rclpy`、`sensor_msgs`、`geometry_msgs`、`trajectory_msgs`、`control_msgs`、`diagnostic_msgs`、`std_srvs`、`std_msgs` 等接口依赖。
  - 声明外部 SDK 依赖（I2RT）和脚本运行所需的 `python3` 模块（如 `pyyaml`）。
- `setup.py`
  - 使用 `setuptools.setup()` 注册 `packages=[]`（脚本位于 `src/`），通过 `py_modules=['robot_driver']` 或将源码改为包形式；
  - 配置 `entry_points={'console_scripts': ['robot_driver=robot_driver:main']}` 方便 `ros2 run driver robot_driver`。
- `setup.cfg`
  - `data_files` 中安装 `package.xml`、`resource/robot_driver`、`launch/` 与 `config/`；
  - `options.entry_points` 重复声明 console script，确保 `colcon build` 正确生成。
- `launch/robot_driver.launch.py`
  - 读取 `robot_driver_config.yaml` 与可选 `safe_pose` YAML，拼装 node 参数；
  - 暴露 `can_channel`、`zero_gravity_default`、`xyz_only_mode` 等 launch 参数，可供 `start_robot_driver.sh` 复用；
  - 可选：在 debug 模式下拉起 `ros2 run tf2_ros static_transform_publisher` 或状态可视化节点。
- `config/robot_driver_config.yaml`
  - 至少包含 `joint_state_rate`、`diagnostics_rate`、`command_timeout_s`、`safe_pose`、`log_dir` 等字段；
  - 默认引用 `config/safe_pose_config.yaml`（由 `safe_pose_build.sh` 维护），必要时可回滚到 `safe_pose_default.yaml`。

## 待讨论要点
1. 通信接口
   - ROS 2 话题/服务/动作的命名、数据结构
   - 底层总线（如 CAN、串口、以太网）协议映射
2. 状态上报
   - 采样频率、数据字段（关节、末端、诊断）
   - 故障/告警的上报机制
3. 命令下发
   - 支持的指令类型（关节、笛卡尔、技能触发）
   - 限速、插值、安全联锁策略
4. 运行管理
   - 启停流程、热插拔
   - 日志、监控与自检需求
5. 性能与可靠性指标
   - 延迟/抖动阈值
   - 容错与重连策略
6. 测试与验收
   - 单元测试/仿真/实机 checklist
   - 回归标准

## 下一步
- 补充 stakeholder 输入，敲定上述要素
- 形成节点级设计文档 & 实现计划
