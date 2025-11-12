# robot_driver.md

该文档用于梳理 `robot_driver_node`（设备下发/上报驱动）的需求、边界与验收标准。

## 背景
- `robot_driver_node` 位于 `L2/src/driver/`，承担机械臂/夹爪等执行器的底层通信、状态采集与命令下发。
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
   - 订阅 `/safety_stop` 或等效控制口，收到后立即执行安全回归流程并同样进入零重力模式。
5. 零重力模式开关
   - 暴露接口（service/action）以启用或关闭零重力模式，并同步更新状态上报与安全限制。
   - 若节点处于零重力模式且收到新的运动指令，需先自动退出零重力模式，再恢复常规控制链路。
6. 生命周期管理
   - 节点启动由工作区根目录的 `start_robot_driver.sh` 负责触发，脚本需拉起驱动、完成初始化并开始发布话题；关闭时安全停车、断开总线，保证流程幂等。
7. 诊断与告警
   - 通过 `diagnostic_msgs/DiagnosticArray` 或自定义话题上报 CAN 状态、力矩/温度/电流异常、模式切换结果。
8. 故障恢复
   - 对通信超时、驱动故障、电源波动等事件执行自动重试或降级策略，并将状态同步给上层。
9. 自检与标定入口
   - 提供触发自检、零位校准、夹爪检查的接口，确保上层在收到“ready”前不会下发运动任务。
10. 日志与追溯
   - 记录关键指令、模式切换、故障上下文（时间戳、指令来源、传感器状态）以便排障，并统一写入 `l2/log/`（相对路径）下的子目录，便于集中收集与分析。

## 实现技术方案（对标 l1_stage1_device）
1. 模块分层
   - `can_interface_manager`：借鉴 `l1_stage1_device/cartesian/can_monitor.py` 的思路，封装 gs_usb 设备检测、`ip -details link show` 解析、`I2RT/i2rt/scripts/reset_all_can.sh` 调用与 1 Mbps 配置校验，暴露 `ensure_ready(required=[canX])` API。
   - `hardware_commander`：类似 `CartesianCommander`，包装 I2RT SDK 的 joint/pose 读取、轨迹插值、零重力模式切换（SetZeroGravity）、回零/归位等原语，确保上层 ROS 节点不直接触碰 SDK。
   - `robot_driver_node`：ROS 2 节点本体，负责接口定义、命令调度、多线程状态发布，与 `Start/Stop` 生命周期钩子。

2. 启动链路
   - `start_robot_driver.sh`（位于 `L2/` 根目录）：仿照 `L1/start_stage1.sh`，完成以下步骤：
     1. 解析 `--can canX`、`--xyz-only`、`--zero-gravity` 等参数；
     2. 调用 `I2RT/i2rt/scripts/reset_all_can.sh` 强制清总线；
     3. 执行 `python3 -m driver.can_interface_manager --ensure --interface canX` 以确认 gs_usb 接口、比特率；
     4. 设定 `PYTHONPATH`（挂载 I2RT SDK）、ROS 环境、`l2/log/robot_driver/robot_driver.log` 输出，并以 `ros2 launch driver robot_driver.launch.py` 方式拉起节点；
     5. 统一杀掉旧进程 `robot_driver_node`，保证脚本幂等。

3. ROS 接口设计
   - 话题：
     - `/joint_states` (`sensor_msgs/JointState`)：30 Hz 发布关节/速度/电流（若 SDK 支持），供 MoveIt/RViz 使用；
     - `/robot_driver/diagnostics` (`diagnostic_msgs/DiagnosticArray`)：1 Hz 汇报命令执行状态、CAN 健康、零重力模式、最近告警；
     - `/robot_driver/cartesian_cmd` (`geometry_msgs/PoseStamped`)：机械臂坐标系的笛卡尔目标；
     - `/safety_stop` (`std_msgs/Empty`)：收到即触发安全归位逻辑；
   - 行动/服务：
     - `control_msgs/action/FollowJointTrajectory`（命名 `/robot_driver/follow_joint_trajectory`）承载“标准控制”——MoveIt 或上层可直接投喂关节轨迹；
     - `std_srvs/SetBool /robot_driver/zero_gravity`：打开/关闭零重力模式，内部切换 SDK 状态并返回实际结果；
     - `std_srvs/Trigger /robot_driver/reset_can`：手动触发 CAN 复位 + 重新初始化。

4. 指令调度与安全
   - 命令队列：沿用 Stage1 的“单工作线程+可抢占”模型，使用 `queue.Queue` 存储笛卡尔/关节指令，任何新区指令会清空旧队列并打断当前插值；
   - 插值策略：
     - 笛卡尔目标：通过 IK 求解后在关节空间作线性插值（同 Stage1 `_interpolate_to_goal`）；
     - 关节轨迹：遵循 `FollowJointTrajectory` action 的 `time_from_start`，逐点执行并校验速度/加速度；
   - 心跳/超时：维护 `last_command_stamp`，若在 `timeout_s`（固定 60 s）内未收到任何控制指令，则触发安全归位（进入 `SAFE_POSE` 并启用零重力模式）；重新收到命令时即刻退出零重力并恢复常规控制。
   - `/safety_stop`：节点订阅后立即抢占当前运动并调用 `hardware_commander.move_to_safe_pose()`。

5. 零重力模式钩子
   - 在 Commander 初始化时读取参数 `zero_gravity_default`，并提供 `set_zero_gravity(bool)` API，内部调用 I2RT SDK 的零重力接口（若缺失则通过写寄存器实现），状态变更同步到 `/robot_driver/diagnostics`。
   - 服务端做互斥处理：切模式期间暂停指令执行，并在成功/失败时写日志到 `l2/log/robot_driver/`。

6. 日志与诊断
   - 日志滚动策略：`start_robot_driver.sh` 创建 `l2/log/robot_driver/robot_driver_<date>.log`，ROS 节点内部使用 `logging` 模块写同一路径；
   - 关键信息（CAN 重连、零重力切换、`safety_stop` 触发、心跳超时）必须双写：一份进入 `/robot_driver/diagnostics`，一份落地日志文件，便于离线排查；
   - 发生异常时自动收集最近一次指令上下文（命令源 topic/action、目标值、IK 耗时）并写入日志。

7. 状态广播与调试
   - 复用 Stage1 中 TF 广播逻辑（`base_link -> tool0` + 可选 `world -> base_link`），确保上层可在 RViz 看到末端姿态；
   - 提供 `ros2 topic echo /robot_driver/diagnostics`、`ros2 action list` 等调试指令，并在 README 中文档化。

8. 状态发布（JointState/TF）
   - 将 Stage1 的 `_publish_tf` 定时器拆分为 `joint_state_timer` + `tf_timer`：
     - `joint_state_timer`：以 30 Hz 读取 I2RT SDK 缓存里的关节角、速度、电流，构造 `JointState` 并发布；读取使用轻量读锁，仅在写命令时短暂加写锁，确保运动与状态采集可并行。
     - `tf_timer`：保持 30 Hz（或配置值）读取末端位姿并广播 TF。
   - 若 SDK 提供 `get_joint_state_cached()` 之类非阻塞 API，则驱动优先使用缓存；否则在 commander 层维护 ring buffer，写命令时顺便更新最新状态，供 JointState 发布线程读取。

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
