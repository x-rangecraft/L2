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

float32 width_m        # 目标开口宽度（米），0=完全闭合，max_width_m=完全打开
                       # - OPEN: 若 <=0，则使用参数中的默认开口宽度；>0 时覆盖默认值
                       # - GRIP: 通常填 0 或不填，表示“本次夹取允许的最大闭合行程使用默认值”
                       #         也可以填一个 >0 的值，限制最大闭合宽度
                       # - MOVE: 必填，表示目标开口宽度

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

float32 final_width_m    # 最终开口宽度（米）
float32 contact_position # 发生接触时的 gripper 关节位置（rad，若无接触可设为 NaN）
float32 contact_effort   # 发生接触时的 effort（若无接触则为 0）
---
# Feedback
string  phase             # 'queued' / 'moving' / 'contact' / 'complete'
float32 completion_ratio  # 0~1 粗略进度
float32 current_width_m   # 当前开口宽度（米）
bool    in_contact        # 当前是否检测到接触
```

## 4. 行为语义

### 4.1 width_m → 归一化开口度与关节角映射

Action 对上层暴露的是**物理尺寸** `width_m`，表示两指之间的开口宽度（单位米）：

- 0：完全闭合；
- `max_width_m`：完全打开。

根据现场测量，当前夹爪的最大开口宽度：

- `max_width_m = 0.094`（单位：米）

内部实现可以先将 `width_m` 映射到一个归一化开口度 `opening_norm ∈ [0,1]`：

```text
width_m_clamped = clamp(width_m, 0.0, max_width_m)
opening_norm = width_m_clamped / max_width_m
```

- 因此：
  - `width_m = 0.0   → opening_norm = 0.0`（完全闭合）
  - `width_m = 0.094 → opening_norm = 1.0`（完全打开）

在 Joint 空间里，仍然通过 `position_open_rad` / `position_close_rad` 做线性插值：

```text
position_rad = position_close_rad
              + opening_norm * (position_open_rad - position_close_rad)
```

需要通过 ROS 参数或配置文件给出 gripper 的位置范围：

- `gripper.position_open_rad`：完全打开时的关节角（rad）；
- `gripper.position_close_rad`：完全闭合时的关节角（rad，接近机械极限但留有安全裕度）。

`Goal.width_m` 先在 `[0, max_width_m]` 范围内裁剪，再按上述公式映射到关节角。

### 4.2 三种 command 模式

1. `COMMAND_OPEN`
   - 目标：将夹爪打开到某个开口宽度。
   - 宽度选择：
     - 若 Goal.width_m > 0，则使用该值；
     - 否则使用参数 `gripper.open_width_default_m`（例如 0.06m）。
   - 行为：
     - 按目标开口宽度计算目标关节角，通过 `JointCommand` 控制 `gripper` 关节到位；
     - 通常忽略接触信号（`stop_on_contact=false`），Result 中 `object_attached=false`。

2. `COMMAND_GRIP`
   - 目标：从当前宽度向闭合方向夹取物体，在**判断为夹到物体后停止**，并给出“是否夹到物体”的结论。
   - 最大闭合行程：
     - 若 Goal.width_m > 0，则视为本次夹取允许的最大闭合宽度；
     - 否则使用参数 `gripper.grip_width_max_m` 或默认 `max_width_m`。
   - 行为：
     - 内部构造一个 JointCommand.Goal，仅包含 `gripper` 关节，目标关节角对应上述“最大闭合宽度”；
     - 持续监听 JointCommand 的 Feedback 中 `contact_state` 里 `name=='gripper'` 的 `JointContact`：
       - 利用 `position` / `effort` / `in_contact`，配合配置参数判断“夹到物体”还是“空抓到底”（见 4.3）。
     - 若 Goal.stop_on_contact=true 且判定为“夹到物体”，则提前 `cancel` 底层 JointCommand：
       - 夹爪停止继续向内闭合，减少对软体物体的挤压；
       - Result 中 `object_attached=true`。
     - 若一直未判断为“夹到物体”，JointCommand 正常运行到目标闭合位置，Result 中 `object_attached=false`，通常返回 `result_code='NO_OBJECT'`。

3. `COMMAND_MOVE`
   - 目标：将夹爪移动到指定开口宽度，不做“夹取成功/失败”的语义判断。
   - 要求：Goal.width_m 必须在 `[0, max_width_m]` 内（实现中应裁剪）。
   - 行为：
     - 纯位置控制，内部构造 JointCommand 并等待其完成；
     - Result 中 `object_attached=false`，但会按实况填充 `in_contact`、`final_width_m` 等字段，供上层自行解释。

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

### 5.2 滑动窗口 + 差值启发式（推荐）

考虑到软体物体在夹持过程中，绝对力矩值可能不大，但会在接触瞬间出现明显的“抬力 + 掉速”，GripperCommand 在 GRIP 模式下采用一个**短时间滑动窗口**来检测接触事件，而不是只看单帧数据。

#### 5.2.1 维护短时间窗口

在 GRIP 模式下，每次收到 `/robot_driver/action/joint` 的 Feedback 时，从 `contact_state` 中找到 `name == 'gripper'` 的那条，记录：

- `e[i] = |effort_i|`
- `v[i] = |velocity_i|`
- `p[i] = position_i`
- `t[i] = 时间戳（可选，仅用于调试和统计）`

内部维护一个固定大小的环形缓冲区，例如最近 `N = 10` 帧：

- `i = 0 .. N-1` 为窗口内的样本索引；
- 下文所有“最近几帧”均指该窗口内的最后 K 帧（如 K=3）。

#### 5.2.2 阶段 A：确认“自由闭合”阶段

在开始做接触检测前，先确认存在一段“正常闭合”的历史窗口，以避免刚启动时的噪声被误判为接触。

从窗口中选取“自由闭合段”的样本（例如最近若干帧中速度较大的帧）：

- 选取满足 `v[i] > v_free_min` 的样本，`v_free_min` 例如可取 `0.2 rad/s`；
- 要求这些样本的平均力矩较小：
  - `e_base = mean(e[i_free])`，通常期望 `< e_free_max`（如 0.3）。

若无法找到足够的满足条件的样本，接触检测逻辑暂不启用（例如夹爪一开始就卡住时，只依赖超时/位置兜底）。

#### 5.2.3 阶段 B：检测“疑似接触事件”

在滑动窗口中寻找“力矩抬升 + 速度骤降”的拐点，而不是看单帧绝对值。

从最近 K 帧（例如 3 帧）中计算：

- 力矩抬升量：
  - `e_now_max = max(e[j])`（最近 K 帧中的最大 |effort|）；
  - 要求：`Δeff = e_now_max - e_base >= gripper.effort_jump_threshold`；
- 速度下降量：
  - `v_now_min = min(v[j])`（最近 K 帧中最小 |velocity|，越小越接近静止）；
  - 要求：`Δv = v_base - v_now_min >= gripper.velocity_drop_threshold`。

典型参数建议（结合当前硬件与软体物体测试）：

- `gripper.effort_jump_threshold`：`0.2 ~ 0.4`；
- `gripper.velocity_drop_threshold`：`0.2 ~ 0.4`；
- `v_free_min`：`0.2`；
- `e_free_max`：`0.3`。

若同时满足 `Δeff` 和 `Δv` 的阈值条件，则在该窗口内标记一个“疑似接触事件”（`contact_candidate`）。

#### 5.2.4 阶段 C：确认接触（防止瞬时抖动）

为防止由个别异常样本造成误判，在检测到 `contact_candidate` 之后，还需在后续若干帧中验证：

- 速度持续较小：
  - 在接下来 2~3 帧内，`v[k] <= gripper.velocity_after_threshold`（例如 `0.05`）；  
- 力矩维持高位：
  - 在接下来 2~3 帧内，`e[k] >= e_base + gripper.effort_jump_keep`（略低于 jump 阈值，例如 `0.15`）。

当以上条件均被满足时，认为**确实夹到了某个物体**：

- 标记 `object_attached = true`；
- 记录 `contact_position = p_contact`、`contact_effort = e_contact`；
- 若 Goal.stop_on_contact=true：
  - 通过内部的 JointCommand ActionClient 请求 `cancel_goal_async()`，尝试停止继续闭合。

#### 5.2.5 用宽度区分“软物体 vs 空抓到极限”

为在“软物体夹紧”和“空抓顶到机械限位”之间做区分，引入一个简单的闭合极限阈值 `p_close_threshold`（例如 `0.005 ~ 0.01`，具体由现场标定）：

- 若接触事件发生时：
  - `p_contact > p_close_threshold`：认为在离机械极限尚有一定行程的位置夹到了物体：
    - `result_code = 'OBJECT_GRASPED'`；
  - `p_contact <= p_close_threshold`：倾向认为是空抓顶到机械极限：
    - `result_code = 'NO_OBJECT'` 或 `'LIMIT_REACHED'`（具体编码可约定）。

若整个 GRIP 过程中都未检测到接触事件， 且最终收敛到 `final_width_m` 非常接近 0（即 `p_final <= p_close_threshold`），则认为是空抓：

- `object_attached = false`；
- `result_code = 'NO_OBJECT'`。

#### 5.2.6 推荐的默认配置与窗口大小

参数建议（可根据现场数据微调）：

- 滑动窗口大小：`N = 10` 帧（约对应 0.1~0.3s，根据 JointAction feedback 频率估算）；
- 自由闭合阶段判定：
  - `v_free_min = 0.2` rad/s；
  - `e_free_max = 0.3`；
- 接触事件判定：
  - `gripper.effort_jump_threshold = 0.3`；
  - `gripper.velocity_drop_threshold = 0.3`；
  - `gripper.effort_min_for_contact = 0.3`（确保是有意义的受力）；
  - `gripper.velocity_after_threshold = 0.05`（接触后速度近似为 0）。
- 空抓/极限判定：
  - `gripper.no_object_margin_ratio`：行程裕度比例（如 0.1），用于估算 `p_close_threshold`；
  - 或直接配置 `gripper.p_close_threshold_m`（物理宽度上的接近 0 门限，例如 0.005m）。

实际实现时，可将上述阈值以 ROS 参数形式暴露，便于现场根据“空抓”和“夹物”时的 `effort/velocity` 曲线进行标定，从而达到较稳定的“夹到物体 vs 空抓到极限”的判定效果。

## 6. 典型调用示例（控制台）

以下示例假设 Action 名称为 `/robot_driver/action/gripper`，类型为 `robot_driver/action/GripperCommand`。

### 6.1 打开夹爪

将夹爪打开到 0.08m 开口（约接近最大 0.094m），速度略慢：

```bash
ros2 action send_goal \
  /robot_driver/action/gripper \
  robot_driver/action/GripperCommand \
  "{command: 0,  # COMMAND_OPEN
    width_m: 0.08,
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
    width_m: 0.0,
    speed_scale: 0.4,
    stop_on_contact: true,
    timeout: {sec: 5, nanosec: 0},
    label: 'pick_object'}" \
  --feedback
```

- `width_m: 0.0` 表示“本次夹取允许的最大闭合行程使用默认值”，实际闭合极限由参数决定（当前硬件最大开口约为 0.094m）；
- 执行结束后，可在 Result 中读取：
  - `object_attached`：是否认为夹到物体；
  - `result_code`：`OBJECT_GRASPED` / `NO_OBJECT` 等；
  - `final_width_m` / `contact_position` / `contact_effort` 等详细信息。

### 6.3 移动到指定开口度

将夹爪移动到 0.03m 开口，不关心“是否夹到物体”语义，仅做位置控制：

```bash
ros2 action send_goal \
  /robot_driver/action/gripper \
  robot_driver/action/GripperCommand \
  "{command: 2,  # COMMAND_MOVE
    width_m: 0.03,
    speed_scale: 0.5,
    stop_on_contact: false,
    timeout: {sec: 3, nanosec: 0},
    label: 'move_gripper'}" \
  --feedback
```

> 实现端应确保 `/robot_driver/action/gripper` 只通过内部 `ActionClient(JointCommand)` 操作 `gripper` 关节，统一走现有的关节执行链路与安全策略，避免额外的线程和互斥复杂度。
