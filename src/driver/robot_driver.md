# robot_driver.md

该文档用于梳理 `robot_driver_node`（设备下发/上报驱动）的需求、边界与验收标准。

## 背景
- `robot_driver_node` 位于 `driver/src/robot_driver_node.py`（仓库路径 `L2/src/driver/src/`），承担机械臂/夹爪等执行器的底层通信、状态采集与命令下发。
- 上层模块（move_controller、skill、app 等）依赖该节点提供稳定的实时接口。

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
   - 记录关键指令、模式切换、故障上下文（时间戳、指令来源、传感器状态）以便排障，并统一写入 `l2/log/`（相对路径）下的子目录，便于集中收集与分析。

## ROS 接口设计

### 发布的 Topic
| 话题 | 类型 | 频率 | 说明 |
| --- | --- | --- | --- |
| `/joint_states` | `sensor_msgs/JointState` | 30 Hz（可配置） | 仅包含标准位置/速度/力矩（或电流）字段，确保与 MoveIt、RViz、Foxglove 兼容。 |
| `/robot_driver/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 1 Hz | 汇报命令执行状态、心跳/安全态、零重力模式、CAN 健康、温度/电流/自检等扩展信息，集中展示机械臂健康数据。 |

### 订阅的 Topic
| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/robot_driver/robot_command` | `geometry_msgs/PoseStamped` | 机械臂基坐标系（默认 `base_link`）下的笛卡尔目标，`Pose.position` 为 XYZ，`orientation` 提供单位四元数；驱动执行 IK → 关节插值。 |
| `/robot_driver/safety_stop` | `std_msgs/Empty` | 收到立即抢占当前运动、执行安全归位，但不切换零重力模式。 |
| `/robot_driver/joint_command` | `sensor_msgs/JointState` | （可选）直接指定一个或多个关节位置/速度/力矩；用于调试、标定或单轴测试，默认关闭，通过参数启用。 |

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
- 上层以 10–30 Hz 推送命令即可触发连续插值，最新命令会抢占旧命令。

`/robot_driver/joint_command` 示例（position 模式）：

```yaml
name: [joint_1, joint_3]
position: [0.5, 1.2]
velocity: []
effort: []
```

- `name` 必须与 `/joint_states` 中的名称一致；
- 未列出的关节保持当前值；
- 若 `joint_command_mode` 设为 velocity/effort，则分别读取对应字段并忽略其余字段；
- 该接口多借鉴 I2RT SDK 的 `command_joint_pos`/`command_joint_vel` 能力，适合单轴标定、零位校准或 I2RT 集成测试。

### 服务与动作
| 名称 | 类型 | 用途 | 调用示例 |
| --- | --- | --- | --- |
| `/robot_driver/service/zero_gravity` | Service (`std_srvs/SetBool`) | 打开/关闭零重力模式。 | `ros2 service call /robot_driver/service/zero_gravity std_srvs/srv/SetBool "{data: true}"` |
| `/robot_driver/service/reset_can` | Service (`std_srvs/Trigger`) | 手动触发 CAN 复位并重新初始化 commander。 | `ros2 service call /robot_driver/service/reset_can std_srvs/srv/Trigger {}` |
| `/robot_driver/action/follow_joint_trajectory` | Action (`control_msgs/action/FollowJointTrajectory`) | 承载标准关节轨迹控制，供 MoveIt/controller_manager 直接发送轨迹。 | `ros2 action send_goal /robot_driver/action/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{goal: {trajectory: ...}}"` |

**Action 使用方法**
1. 规划器根据 `/joint_states` 的关节命名生成 `trajectory_msgs/JointTrajectory`；
2. 调用 `/robot_driver/action/follow_joint_trajectory` 发送目标（可使用 `ros2 action send_goal` 或 controller_manager）；
3. 驱动节点按 `time_from_start` 执行，并在结果里返回 `SUCCESS/ABORTED` 与失败原因；若过程中触发 `/robot_driver/safety_stop`、心跳超时或零重力切换，action 会立刻 `ABORTED` 并记录日志。

## 实现技术方案（对标 l1_stage1_device）
1. 启动链路（方案稳定，暂不调整）
   - `driver/start_robot_driver.sh`：仿照 `L1/start_stage1.sh`，完成以下步骤：
     1. 解析 `--can canX`、`--xyz-only`、`--zero-gravity` 等参数；
     2. 调用 `I2RT/i2rt/scripts/reset_all_can.sh` 强制清总线；
     3. 执行 `python3 -m driver.can_interface_manager --ensure --interface canX` 以确认 gs_usb 接口、比特率；
     4. 设定 `PYTHONPATH`（挂载 I2RT SDK）、ROS 环境、`l2/log/robot_driver/robot_driver.log` 输出，并以 `ros2 launch driver robot_driver.launch.py` 方式拉起节点；
     5. 统一杀掉旧进程 `robot_driver_node`，保证脚本幂等。

2. 模块分层
   - `can_interface_manager`：借鉴 `l1_stage1_device/cartesian/can_monitor.py` 的思路，封装 gs_usb 设备检测、`ip -details link show` 解析、`I2RT/i2rt/scripts/reset_all_can.sh` 调用与 1 Mbps 配置校验，暴露 `ensure_ready(required=[canX])` API。
   - `hardware_commander`：类似 `CartesianCommander`，包装 I2RT SDK 的 joint/pose 读取、轨迹插值、零重力模式切换（SetZeroGravity）、回零/归位等原语，确保上层 ROS 节点不直接触碰 SDK。
   - `robot_driver_node`：ROS 2 进程主体，进一步拆成三个内部组件：
     - `state_publisher`：独立 callback group 或 executor，周期性读取 commander 缓存并发布 `/joint_states`、TF 以及 `/robot_driver/diagnostics` 中的运行指标，确保读路径只持有读锁、不阻塞控制线程。
     - `motion_controller`：承载 `/robot_driver/robot_command`、`/robot_driver/joint_command`、`FollowJointTrajectory` action 等控制接口，负责命令抢占、IK + 插值、限速、写入 commander。
     - `command_watchdog`：挂在控制线程侧的子模块，维护 `last_command_stamp`；若 60 s（或 `command_timeout_s`）未收到控制指令则主动调用 `motion_controller` 的安全归位流程，并在确认机械臂已落位 SAFE_POSE 后再触发 `hardware_commander.set_zero_gravity(true)`，也负责 `/robot_driver/safety_stop` 的统一抢占收敛。

3. 指令调度与安全
   - 命令队列：沿用 Stage1 的“单工作线程+可抢占”模型，使用 `queue.Queue` 存储笛卡尔/关节指令，任何新区指令会清空旧队列并打断当前插值；
   - 插值策略：
     - 笛卡尔目标：通过 IK 求解后在关节空间作线性插值（同 Stage1 `_interpolate_to_goal`）；
     - 关节轨迹：遵循 `FollowJointTrajectory` action（`/robot_driver/action/follow_joint_trajectory`）的 `time_from_start`，逐点执行并校验速度/加速度；
     - 关节直控：当启用 `/robot_driver/joint_command` 时，节点根据消息中的关节名替换当前姿态中对应分量，再应用限速/限幅并写入硬件，逻辑复用 Commander 的 `command_joint_pos/vel` 能力。
   - 心跳/超时：由 `command_watchdog` 维护 `last_command_stamp`，若在 `timeout_s`（固定 60 s）内未收到任何控制指令，则触发安全归位；当确认机械臂到达 `SAFE_POSE` 后再启用零重力模式，重新收到命令时即刻退出零重力并恢复常规控制。
   - `/robot_driver/safety_stop`：节点订阅后立即抢占当前运动；若当前已处于零重力模式，先调用 `hardware_commander.set_zero_gravity(false)` 以恢复常规控制，再执行 `move_to_safe_pose()`，待确认 SAFE_POSE 落位成功后再次启用零重力；若原本不在零重力模式，则直接回到安全位并在完成后开启零重力。

4. 零重力模式钩子
   - 在 Commander 初始化时读取参数 `zero_gravity_default`，并提供 `set_zero_gravity(bool)` API，内部调用 I2RT SDK 的零重力接口（若缺失则通过写寄存器实现），状态变更同步到 `/robot_driver/diagnostics`。
   - 服务端做互斥处理：切模式期间暂停指令执行，并在成功/失败时写日志到 `l2/log/robot_driver/`。

5. 日志与诊断
   - 日志滚动策略：`driver/start_robot_driver.sh` 创建 `l2/log/robot_driver/robot_driver_<date>.log`，ROS 节点内部使用 `logging` 模块写同一路径；
   - 关键信息（CAN 重连、零重力切换、`/robot_driver/safety_stop` 触发、心跳超时）必须双写：一份进入 `/robot_driver/diagnostics`，一份落地日志文件，便于离线排查；
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
  4. 操作者确认后，脚本在当前目录生成 `safe_pose_<timestamp>.yaml`，内容包括 `joint_names`、`positions`、可选末端位姿与备注，便于直接拷贝至 `safe_pose` 配置数组。
- 后续可把生成的 YAML 纳入版本管理或部署包，`command_watchdog` 与 `/robot_driver/safety_stop` 将依赖该 SAFE_POSE 数据。在补充具体参数前，此脚本可反复执行以更新标定结果。


## 配置与动态更新
- 默认参数文件：`driver/config/robot_driver_config.yaml`（后续创建），通过 `ros2 launch driver robot_driver.launch.py params:=...` 注入；
- 关键参数：`can_channel`、`zero_gravity_default`、`joint_state_rate`、`diagnostics_rate`、`command_timeout_s`（60 s 默认）、`safe_pose` 数组、`log_dir`（默认 `l2/log/robot_driver/`）。
  - `safe_pose` 默认从 `config/safe_pose_default.yaml` 读入，可由 `safe_pose_build.sh` 输出的最新 `safe_pose_*.yaml` 替换。
- 关节直控相关参数：
  - `enable_joint_direct_ctrl`（bool，默认 `false`）：是否订阅 `/robot_driver/joint_command`；
  - `joint_command_topic`（string，默认 `/robot_driver/joint_command`）：话题名，可通过 launch/CLI 调整；
  - `joint_command_mode`（enum: position/velocity/effort，默认 position）：决定如何解读 `sensor_msgs/JointState` 中的 `position/velocity/effort` 字段；
  - `joint_command_rate_limit`（double，默认 0.5 rad/s）：单轴命令的限速，用于调试时保护电机。
- `driver/start_robot_driver.sh` 提供 CLI 覆盖：`--can canX`、`--zero-gravity`、`--xyz-only`、`--params <file>`，脚本会把结果传入 ROS 参数；
- 运行期允许通过 `ros2 param set` 更新白名单参数：
  - `can_channel`：触发内部回调 → 暂停命令 → 调用 `/robot_driver/service/reset_can` 逻辑 → 切换 gs_usb 接口；
  - `joint_state_rate`、`diagnostics_rate`、`command_timeout_s`：实时更新定时器和安全心跳；
  - 其他参数需通过 YAML 修改后重启节点。
- 该机制支持在调试阶段快速在 `can0/can1/can2` 之间切换，后续可扩展自动枚举逻辑以适配更多硬件组合。

### 目录结构
当前包目录规划如下：

```
driver/
├── src/
│   └── robot_driver_node.py
├── config/
│   ├── robot_driver_config.yaml
│   └── safe_pose_default.yaml
├── launch/
│   └── robot_driver.launch.py
├── resource/
│   └── robot_driver
├── setup.py
├── setup.cfg
├── package.xml
├── start_robot_driver.sh
└── safe_pose_build.sh
```

- `driver/start_robot_driver.sh`：驱动启动脚本，负责参数解析、环境配置与 `ros2 launch` 拉起。
- `driver/safe_pose_build.sh`：SAFE_POSE 标定脚本，生成 `safe_pose_*.yaml`。
- `driver/src/robot_driver_node.py`：ROS 2 主节点实现。
- `driver/package.xml`：ROS 2 包元数据，声明依赖与构建信息。
- `driver/setup.py` / `setup.cfg`：声明 `ament_python` 包、入口点、安装数据文件（config、launch、resource）。
- `driver/resource/robot_driver`：ROS 2 资源索引文件，确保 `ros2 run driver` 可找到节点。
- `driver/launch/robot_driver.launch.py`：标准 launch，加载参数 YAML、配置日志级别并启动节点（支持 CLI 传参）。
- `driver/config/robot_driver_config.yaml`：默认参数文件，可通过 launch 覆盖。
- `driver/config/safe_pose_default.yaml` 及 `safe_pose_*.yaml`：人工标定出的安全位姿快照，供 `safe_pose` 阵列引用。
- `l2/log/robot_driver/`：运行期日志目录，由启动脚本或节点创建。

### 源码模块划分（面向对象）
- `driver/src/robot_driver/main.py`
  - `RobotDriverNode(rclpy.node.Node)`：加载参数、实例化下述组件、注册 ROS 接口，并负责 executor 与生命周期钩子。
- `driver/src/robot_driver/can_interface_manager.py`
  - `CanInterfaceManager`：封装 gs_usb 检测、比特率校验、掉线重连，暴露 `ensure_ready()`、`reset()`。
- `driver/src/robot_driver/hardware_commander.py`
  - `HardwareCommander`：面向 I2RT SDK 的 OO 包装，提供 `read_joint_state()`、`read_cartesian_state()`、`command_pose()`、`command_joints()`、`move_to_safe_pose()`、`set_zero_gravity()` 等线程安全 API。
- `driver/src/robot_driver/state_publisher.py`
  - `StatePublisher`：独立 callback group，周期性读取 commander 缓存并发布 `/joint_states`、TF、`/robot_driver/diagnostics`；可组合 `DiagnosticsReporter` 处理告警聚合。
- `driver/src/robot_driver/motion_controller.py`
  - `MotionController`：处理 `/robot_driver/robot_command`、`/robot_driver/joint_command`、`FollowJointTrajectory`，内部集成命令队列、IK/插值、限速与 commander 写入；辅以 `TrajectoryPlanner` 工具类（可放同文件或 `trajectory.py`）。
- `driver/src/robot_driver/command_watchdog.py`
  - `CommandWatchdog`：维护 `last_command_stamp`，监听 controller 的活动事件；若超时或收到 `/robot_driver/safety_stop`，执行“若在零重力先退出→调用 commander.move_to_safe_pose() → 落位成功后再启零重力”的顺序，并向 diagnostics 写超时记录。
- `driver/src/robot_driver/safe_pose_loader.py`
  - `SafePoseLoader`：解析 `config/safe_pose_default.yaml` 或 `safe_pose_*.yaml`，校验关节名/角度范围，并向 controller/watchdog 提供 SAFE_POSE 数据。
- `driver/src/robot_driver/logging_utils.py`
  - 封装日志/事件落盘，统一写 `l2/log/robot_driver/` 与 ROS logger，支持关键事件双写。
- `driver/src/robot_driver/parameter_schema.py`
  - 以 dataclass/TypedDict 描述节点参数（rates、超时、安全姿态、限速等），集中做默认值填充与合法性校验。
- `driver/src/robot_driver/tools/safe_pose_capture.py`（可选）
  - 供 `safe_pose_build.sh` 调用的 CLI，负责切换零重力、等待操作者、读取 `/joint_states` 并输出 YAML。

`setup.py` 中的 console script 指向 `robot_driver.main:main`，确保各模块按需导入，易于单元测试（组件可独立实例化并用 mock commander 验证）。

### 节点配置（ROS 包）
- `package.xml`
  - `<buildtool_depend>ament_python</buildtool_depend>`、`<exec_depend>` 覆盖 `rclpy`、`sensor_msgs`、`geometry_msgs`、`trajectory_msgs`、`control_msgs`、`diagnostic_msgs`、`std_srvs`、`std_msgs` 等接口依赖。
  - 声明外部 SDK 依赖（I2RT）和脚本运行所需的 `python3` 模块（如 `pyyaml`）。
- `setup.py`
  - 使用 `setuptools.setup()` 注册 `packages=[]`（脚本位于 `src/`），通过 `py_modules=['robot_driver_node']` 或将源码改为包形式；
  - 配置 `entry_points={'console_scripts': ['robot_driver_node=robot_driver_node:main']}` 方便 `ros2 run driver robot_driver_node`。
- `setup.cfg`
  - `data_files` 中安装 `package.xml`、`resource/robot_driver`、`launch/` 与 `config/`；
  - `options.entry_points` 重复声明 console script，确保 `colcon build` 正确生成。
- `launch/robot_driver.launch.py`
  - 读取 `robot_driver_config.yaml` 与可选 `safe_pose` YAML，拼装 node 参数；
  - 暴露 `can_channel`、`zero_gravity_default`、`xyz_only_mode` 等 launch 参数，可供 `start_robot_driver.sh` 复用；
  - 可选：在 debug 模式下拉起 `ros2 run tf2_ros static_transform_publisher` 或状态可视化节点。
- `config/robot_driver_config.yaml`
  - 至少包含 `joint_state_rate`、`diagnostics_rate`、`command_timeout_s`、`safe_pose`、`log_dir`、`enable_joint_direct_ctrl` 等字段；
  - 默认引用 `config/safe_pose_default.yaml` 以集中维护姿态数据，可在标定后替换为新的 `safe_pose_*.yaml`。

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
