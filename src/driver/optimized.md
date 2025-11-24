# robot_driver 优化记录（candidate 清理项）

> 目的：记录当前 `robot_driver` 实现中“不会被实际走到 / 无行为影响”的逻辑和参数，后续按需统一清理或继续演进，避免反复分析。

## 1. 未接入当前节点流程的代码

### 1.1 CommandWatchdog 及依赖链

- 文件：`src/driver/src/driver/control/command_watchdog.py`
- 现状：
  - 类 `CommandWatchdog` 仅在文档和兼容 shim 中被提到，没有任何实际实例化点；
  - `rg "CommandWatchdog" src/driver` 只匹配到：
    - 定义文件本身；
    - `robot_driver.md` 中的设计描述。
- 影响：
  - 定时心跳 `_tick()`、`engage()`、`_run_safety_sequence()` 不会被调用；
  - `handle_safety_stop()` 也不会接收到 `/robot_driver/safety_stop`，因为节点中该 topic 已由 `RobotDriverNode._on_safety_stop()` 直接转 SafetyPose action。

**结论：**

- 在当前版本的节点中，`CommandWatchdog` 整体处于“悬空未使用”状态。
- 与其绑定的 SAFE_POSE 流程可以视为候选清理项或将来重构的基础。

### 1.2 JointControler.safe_pose_sequence()

- 文件：`src/driver/src/driver/control/joint_controler.py`
- 方法：`JointControler.safe_pose_sequence(...)`
- 现状：
  - 唯一调用点在 `CommandWatchdog._run_safety_sequence()`；
  - 节点其他地方只调用了 `halt_and_safe()`，没有走 `safe_pose_sequence()`。
- 设计目标：
  - 在 SAFE_POSE 运动后显式等待一段时间，并根据误差重新开启零重力；
  - 作为“watchdog 异常路径”使用。

**结论：**

- 只要 `CommandWatchdog` 不接入，`safe_pose_sequence()` 的逻辑就不会被实际走到；
- 当前 SAFE_POSE 主路径完全由 `RobotDriverNode._execute_safety_pose_action()` + `SafePoseManager` 负责。

### 1.3 SafePoseManager 的零重力透传接口

- 文件：`src/driver/src/driver/safety/safe_pose_manager.py`
- 方法：
  - `get_zero_gravity()`
  - `set_zero_gravity(enabled: bool)`
- 现状：
  - 全仓库没有任何地方调用这两个方法；
  - 零重力状态统一由 `ZeroGravityManager` + `HardwareCommander` 管理，并通过 `/robot_driver/service/zero_gravity` 暴露。

**结论：**

- 这两个方法目前只是简单透传，没有被实际使用，可视作“为未来扩展预留”的接口。

### 1.4 safe_pose_executor 相关 alias

- 多个兼容 shim 的 `_alias_map` 中包含：
  - `'safe_pose_executor': 'driver.safety.safe_pose_executor'`
  - 例如：
    - `src/driver/src/driver/robot_driver_node.py`
    - `src/driver/src/driver/command_watchdog.py`
    - `src/driver/src/driver/motion_controller.py`
    - `src/driver/src/driver/parameter_schema.py`
    - `src/driver/src/driver/state_publisher.py` 等。
- 实际情况：
  - 当前仓库没有 `src/driver/src/driver/safety/safe_pose_executor.py`；
  - 没有任何代码 `import driver.safe_pose_executor`。

**结论：**

- 这条 alias 是一个“残留的兼容映射”，不会被走到，即便被使用也会直接 ImportError。

## 2. 仅用于兼容旧 import 路径的 shim 模块

### 2.1 shim 模块列表

这类文件本身不实现业务逻辑，只做模块重定向：

- 典型结构（以 `src/driver/src/driver/hardware_commander.py` 为例）：
  - 定义 `_alias_map`，根据模块名映射到新的实现路径；
  - 通过 `import_module` 导入目标模块；
  - `globals().update(module.__dict__)` 直接导出所有符号。

当前存在的 shim 包括（但不限于）：

- `src/driver/src/driver/hardware_commander.py`
- `src/driver/src/driver/joint_controler.py`
- `src/driver/src/driver/can_interface_manager.py`
- `src/driver/src/driver/state_publisher.py`
- `src/driver/src/driver/path_utils.py`
- `src/driver/src/driver/logging_utils.py`
- `src/driver/src/driver/robot_description_loader.py`
- `src/driver/src/driver/safe_pose_loader.py`
- `src/driver/src/driver/command_watchdog.py`（shim，本体在 `control/command_watchdog.py`）
- `src/driver/src/driver/motion_controller.py`（单纯把 `MotionController` 重定向为 `JointControler`）

### 2.2 当前实际使用情况

- 仓库内部代码已经统一使用新的子包路径，例如：
  - `from driver.hardware.hardware_commander import HardwareCommander`
  - `from driver.control.joint_controler import JointControler`
  - `from driver.utils.logging_utils import get_logger`
- 唯一必须保留的 shim：
  - `driver.robot_driver_node`：被 `setup.py` 的 entry point 使用，
    - `robot_driver_node = driver.robot_driver_node:main`
    - 该 shim 再重定向到 `driver.app.robot_driver_node`.

**结论：**

- 除 `driver.robot_driver_node` 以外，其余 shim 当前仅作为“对旧 import 路径的保险兼容层”；
- 如果外部项目已全部切换到新的 `driver.<submodule>...` 路径，这些 shim 可以作为未来清理候选。

## 3. ROS 参数层面“有声明但无行为影响”的配置项

以下字段都在 `DriverParameters` / YAML 中声明和配置，但当前不会改变任何控制行为：

### 3.1 zero_gravity_default

- 定义：
  - `src/driver/src/driver/config/parameter_schema.py:29, 92`
  - YAML：`src/driver/config/robot_driver_config.yaml:7`
  - Launch / start 脚本会透传该参数。
- 现状：
  - Python 代码中没有对 `self._params.zero_gravity_default` 的任何访问；
  - 零重力策略由 `ZeroGravityManager` + `JointControler` 在运行时按需切换；
  - 启停时 SAFE_POSE 逻辑由 `start_robot_driver.sh` + SafetyPose action 编排，不依赖此参数。

**结论：**

- 当前版本中，该参数只存在于参数文件和声明层面，不改变任何行为。

### 3.2 cartesian_command_rate_limit

- 定义：
  - `parameter_schema.py:39, 102`
  - YAML：`robot_driver_config.yaml:17`
- 现状：
  - `HardwareCommander.command_cartesian_pose()` 及相关 IK/FK 逻辑没有读取该字段；
  - 所有速度限制均在关节层通过 `joint_limit_rate` 与插值逻辑实现。

**结论：**

- 该参数当前未参与任何速率计算，只是一个预留配置项。

### 3.3 enable_joint_velocity_fuse

- 定义：
  - `parameter_schema.py:49, 114`
  - YAML：`robot_driver_config.yaml:28`
- 现状：
  - 全仓库仅出现在参数声明与文档中；
  - 没有任何控制/估计逻辑根据它切换分支。

**结论：**

- “关节速度融合”相关逻辑尚未实现，该开关目前没有实际作用。

### 3.4 log_dir（参数字段）

- 定义：
  - `parameter_schema.py:31, 94`
  - YAML：`robot_driver_config.yaml:9`
- 实际日志路径决策：
  - `driver.utils.logging_utils.get_logger()` 默认 `log_dir='log/robot_driver'`；
  - 优先读取环境变量 `ROBOT_DRIVER_LOG_FILE`，由 `start_robot_driver.sh` 设置为
    `L2/log/robot_driver/robot_driver_<timestamp>.log`.
- 现状：
  - 代码中没有任何地方通过 `self._params.log_dir` 去覆写日志目录。

**结论：**

- ROS 参数里的 `log_dir` 对实际日志输出路径没有影响，仅环境变量 + 启动脚本起作用。

### 3.5 command_timeout_s

- 定义：
  - `parameter_schema.py:28, 91`
  - YAML：`robot_driver_config.yaml:6`
- 现状：
  - 实际逻辑中仅用于 diagnostics：
    - `src/driver/src/driver/control/state_publisher.py:109` 将其作为 `DiagnosticStatus` 的字段；
  - 原计划由 `CommandWatchdog` 读取并触发超时 SAFE_POSE，但 watchdog 没有被实例化。

**结论：**

- 当前版本里，该字段只影响诊断信息显示，不会触发任何安全动作。

## 4. start_robot_driver.sh 中的遗留选项/逻辑

### 4.1 `--safe-timeout` / `--no-exit-after-safe`

- 位置：
  - `src/driver/start_robot_driver.sh:181-203` 的 `build_child_args()`
- 现状：
  - 这两个选项仅在构造子进程参数时被过滤掉；
  - 脚本没有后续解析、也没有与之对应的行为逻辑。

**结论：**

- 即便在命令行传入这两个选项，也不会对启动流程产生任何影响；
+- 属于“未完成/遗留”的参数，可考虑统一从帮助/代码中移除或实现对应功能。

### 4.2 自动重启相关变量

- 变量：`ATTEMPT` / `RESTART_DELAY_SUCCESS` / `RESTART_DELAY_FAILURE` 等。
- 现状：
  - 主循环已经改为“记录一次退出原因后直接退出监督循环，不再自动重启”；
  - 这些变量更多是历史遗留，当前行为上已经固定为“无重启”。

**结论：**

- 若后续不再恢复 auto-restart，可以考虑顺手清理这些变量与相关注释，简化脚本逻辑。

## 5. 兼容层的 MotionController 名字

- 文件：`src/driver/src/driver/control/motion_controller.py`
- 内容：
  - 仅有 docstring 与：`from .joint_controler import *`
- 现状：
  - 当前节点实现中没有任何地方使用 `MotionController` 这个名字；
  - 实际控制逻辑均直接使用 `JointControler`。

**结论：**

- 若外部工程中不再引用 `driver.control.motion_controller`，则该文件可作为今后清理候选，仅保留 `JointControler`。

## 6. 后续建议（不改变现有行为的前提下）

1. 明确“预留但未实现”的配置项  
   - 在文档中标注 `zero_gravity_default` / `cartesian_command_rate_limit` / `enable_joint_velocity_fuse` / `log_dir` 目前不影响行为，避免误配置；  
   - 或在未来版本中真正接入相应逻辑。

2. 分阶段清理未使用代码  
   - 第一阶段（风险极低）：  
     - `safe_pose_executor` 的 alias；  
     - `start_robot_driver.sh` 中形同虚设的 CLI 选项与自动重启变量。  
   - 第二阶段（需要外部依赖确认）：  
     - `CommandWatchdog` + `JointControler.safe_pose_sequence()`；  
     - 各类 shim 模块（除 `driver.robot_driver_node`）。  

3. 若计划恢复 watchdog/超时 SAFE_POSE 方案  
   - 可以以本文件为基准，将超时时间直接绑定 `command_timeout_s`；  
   - 优先评估是否沿用 SafetyPose action 统一实现，避免 SAFE_POSE 流程分裂为两套逻辑。

