# gripper.md

该文档用于梳理 `/robot_driver/action/gripper` 的接口与行为约定。该 Action 只负责**夹爪开合**与**是否夹取到物体的判定**，底层统一复用已有的关节 Action `/robot_driver/action/joint`，不直接触碰硬件 SDK。

## 1. 设计目标

1. 为上层（robot_skill、脚本、UI）提供一个语义清晰的“夹爪 Action”：
   - 明确区分“打开”“夹取”“移动到指定开口度”三种模式；
   - 通过少量参数控制开合程度与速度。
2. 夹取过程中**基于接触 + 行程位置**判断“是否夹到物体”，并在认为夹到物体时停止继续夹紧，避免对软体物体或易碎物体持续施加过大力。
3. 完全复用现有 `/robot_driver/action/joint` 的插值、限速、抢占、零重力退出与 `JointContact` 接触检测逻辑，确保安全策略一致。

## 2. ROS 接口概览

- Action 名称：`/robot_driver/action/gripper`
- Action 类型：`robot_driver/action/GripperCommand`（新定义）
- 底层依赖：`/robot_driver/action/joint`（`robot_driver/action/JointCommand`）
- 控制关节：只操作 `JointState.name == 'gripper'` 对应的关节。

## 3. GripperCommand.action 约定

> 注意：以下为规划中的 `.action` 定义，实际落地时需在 `src/driver/action/GripperCommand.action` 中保持一致。

```text
# Goal
uint8 command
uint8 COMMAND_OPEN=0   # 打开夹爪
uint8 COMMAND_GRIP=1   # 夹取物体（向关闭方向，带“是否夹到物体”判断）
uint8 COMMAND_MOVE=2   # 移动到指定开口度（不带“夹取”语义）

float32 opening        # 归一化开口度 [0,1]，0=完全打开，1=完全闭合
                       # - OPEN: 若为 0，则使用参数中的默认开口度；>0 时覆盖默认值
                       # - GRIP: 通常填 0 或不填，表示“最大闭合行程”；可用于限制最大闭合位置
                       # - MOVE: 必填，表示目标开口度

float32 speed_scale    # [0,1]，复用 JointCommand 的 speed_scale，用于限速
bool   stop_on_contact # 为 true 时在“判定为夹到物体”后停止继续闭合

builtin_interfaces/Duration timeout
string label           # 可选标签，用于日志与调试
---
# Result
bool   success
string result_code     # 'SUCCESS' / 'TIMEOUT' / 'JOINT_FAILED' / 'NO_OBJECT' / 'OBJECT_GRASPED' 等
string last_error

bool   object_attached # 是否认为夹到了物体（高层最关心的布尔结论）
bool   in_contact      # 最终是否检测到接触（原始信号层面）

float32 final_opening    # 最终开口度 [0,1]
float32 contact_position # 发生接触时的 gripper 关节位置（rad，若无接触可设为 NaN）
float32 contact_effort   # 发生接触时的 effort（若无接触则为 0）
---
# Feedback
string  phase             # 'queued' / 'moving' / 'contact' / 'complete'
float32 completion_ratio  # 0~1 粗略进度
float32 current_opening   # 当前开口度 [0,1]
bool    in_contact        # 当前是否检测到接触
```

## 4. 行为语义

### 4.1 opening → 关节角度映射

夹爪内部使用一个统一的归一化开口度 `opening_norm ∈ [0,1]` 表示开合程度：

- 0：完全打开；
- 1：完全闭合（理论极限）。

通过 ROS 参数或配置文件给出 gripper 的位置范围：

- `gripper.position_open_rad`：完全打开时的关节角（rad）；
- `gripper.position_close_rad`：完全闭合时的关节角（rad，接近机械极限但留有安全裕度）。

映射关系：

```text
position_rad = position_open_rad
              + opening_norm * (position_close_rad - position_open_rad)
```

`Goal.opening` 被视为 `opening_norm`，若不在 [0,1] 范围内需要在实现中进行裁剪。

### 4.2 三种 command 模式

1. `COMMAND_OPEN`
   - 目标：将夹爪打开到某个开口度。
   - 开口度选择：
     - 若 Goal.opening > 0，则使用该值；
     - 否则使用参数 `gripper.opening_default`（0~1）。
   - 行为：
     - 按目标开口度计算目标关节角，通过 `JointCommand` 控制 `gripper` 关节到位；
     - 通常忽略接触信号（`stop_on_contact=false`），Result 中 `object_attached=false`。

2. `COMMAND_GRIP`
   - 目标：从当前开口度向闭合方向夹取物体，在**判断为夹到物体后停止**，并给出“是否夹到物体”的结论。
   - 最大闭合行程：
     - 若 Goal.opening > 0，则视为本次夹取允许的最大闭合开口度（一般为 1.0 或稍小）；
     - 否则使用参数 `gripper.grip_opening_max` 或默认 1.0。
   - 行为：
     - 内部构造一个 JointCommand.Goal，仅包含 `gripper` 关节，目标关节角对应上述“最大闭合开口度”；
     - 持续监听 JointCommand 的 Feedback 中 `contact_state` 里 `name=='gripper'` 的 `JointContact`：
       - 利用 `position` / `effort` / `in_contact`，配合配置参数判断“夹到物体”还是“空抓到底”（见 4.3）。
     - 若 Goal.stop_on_contact=true 且判定为“夹到物体”，则提前 `cancel` 底层 JointCommand：
       - 夹爪停止继续向内闭合，减少对软体物体的挤压；
       - Result 中 `object_attached=true`。
     - 若一直未判断为“夹到物体”，JointCommand 正常运行到目标闭合位置，Result 中 `object_attached=false`，通常返回 `result_code='NO_OBJECT'`。

3. `COMMAND_MOVE`
   - 目标：将夹爪移动到指定开口度，不做“夹取成功/失败”的语义判断。
   - 要求：Goal.opening 必须在 [0,1] 内；
   - 行为：
     - 纯位置控制，内部构造 JointCommand 并等待其完成；
     - Result 中 `object_attached=false`，但会按实况填充 `in_contact`、`final_opening` 等字段，供上层自行解释。

## 5. “是否夹到物体”的判定逻辑

### 5.1 可用信号

`JointControler` 已经定义了 `JointContact` 并通过 JointCommand 的 Feedback/Result 暴露出来：

- `bool in_contact`：
  - 内部定义为：关节速度接近 0 且 effort（力矩/电流）超过阈值 `_CONTACT_EFFORT_THRESHOLD`；
- `float32 position`：该关节当前角度；
- `float32 effort`：当前力矩/电流。

同时可从配置中获得：

- `position_open_rad` / `position_close_rad`；
- `gripper.no_object_margin_ratio`：用于区分“夹到物体”与“顶到机械极限”的行程裕度；
-（可选）`gripper.contact_effort_threshold`：若需要为 gripper 单独调节接触阈值，可覆盖默认 `_CONTACT_EFFORT_THRESHOLD`。

### 5.2 启发式规则

在 GRIP 模式下，GripperCommand server 将 JointCommand 的反馈转换为如下决策逻辑：

设：

- `q_close`：`position_close_rad`（完全闭合目标）；
- `q`：当前 gripper 关节角（rad）；
- `span = q_close - position_open_rad`（完整行程）；
- `dist = |q_close - q|`（当前距离闭合极限的距离）；
- `margin = gripper.no_object_margin_ratio * span`（行程裕度）。

1. **早期接触 → 认为夹到物体**
   - 条件：
     - `in_contact == true` 且 `dist > margin`；
   - 行为：
     - 若 Goal.stop_on_contact=true：
       - 立即取消底层 JointCommand；
       - Result：
         - `success = true`
         - `object_attached = true`
         - `in_contact = true`
         - `result_code = 'OBJECT_GRASPED'`
         - `contact_position = q`
         - `contact_effort = effort`。

2. **临近极限才接触 → 倾向认为空抓**
   - 条件：
     - `in_contact == true` 且 `dist <= margin`；
   - 行为：
     - 不提前停止，允许 JointCommand 完整运行到目标闭合位置；
     - 最终 Result：
       - `object_attached = false`
       - `result_code = 'NO_OBJECT'`（或其他约定标识）；
       - `in_contact` 视最终状态填写。

3. **整个过程中未检测到接触**
   - 行为：
     - JointCommand 正常执行到目标闭合位置；
     - Result 中：
       - `object_attached = false`
       - `in_contact = false`
       - `result_code = 'NO_OBJECT'`。

上述判定是启发式的近似：通过“接触是否发生在距离闭合极限足够远的位置”来区分“夹到物体”与“空抓到底”。实际部署时，可根据现场数据调整：

- `gripper.no_object_margin_ratio`（行程裕度比例）；
- `gripper.contact_effort_threshold`（接触力矩阈值）。

## 6. 典型调用示例（控制台）

以下示例假设 Action 名称为 `/robot_driver/action/gripper`，类型为 `robot_driver/action/GripperCommand`。

### 6.1 打开夹爪

将夹爪打开到 80% 开口，速度略慢：

```bash
ros2 action send_goal \
  /robot_driver/action/gripper \
  robot_driver/action/GripperCommand \
  "{command: 0,  # COMMAND_OPEN
    opening: 0.8,
    speed_scale: 0.6,
    stop_on_contact: false,
    timeout: {sec: 3, nanosec: 0},
    label: 'open_gripper'}" \
  --feedback
```

### 6.2 夹取物体（判定是否夹住）

从当前姿态向闭合方向夹取物体，检测到“早期接触”后立即停止：

```bash
ros2 action send_goal \
  /robot_driver/action/gripper \
  robot_driver/action/GripperCommand \
  "{command: 1,  # COMMAND_GRIP
    opening: 0.0,
    speed_scale: 0.4,
    stop_on_contact: true,
    timeout: {sec: 5, nanosec: 0},
    label: 'pick_object'}" \
  --feedback
```

- `opening: 0.0` 表示“本次夹取允许的最大闭合行程使用默认值”，实际闭合极限由参数决定；
- 执行结束后，可在 Result 中读取：
  - `object_attached`：是否认为夹到物体；
  - `result_code`：`OBJECT_GRASPED` / `NO_OBJECT` 等；
  - `final_opening` / `contact_position` / `contact_effort` 等详细信息。

### 6.3 移动到指定开口度

将夹爪移动到 30% 开口，不关心“是否夹到物体”语义，仅做位置控制：

```bash
ros2 action send_goal \
  /robot_driver/action/gripper \
  robot_driver/action/GripperCommand \
  "{command: 2,  # COMMAND_MOVE
    opening: 0.3,
    speed_scale: 0.5,
    stop_on_contact: false,
    timeout: {sec: 3, nanosec: 0},
    label: 'move_gripper'}" \
  --feedback
```

> 实现端应确保 `/robot_driver/action/gripper` 只通过内部 `ActionClient(JointCommand)` 操作 `gripper` 关节，统一走现有的关节执行链路与安全策略，避免额外的线程和互斥复杂度。

