# 动态抓取技能实现方案 (GraspRecord)

本文档描述在 `robot_skill` 节点中动态抓取功能的技术方案。

## 1. 需求概述

- **输入**：抓取候选数组（GraspCandidate[]），由 Contact-GraspNet 预计算
- **输出**：执行 **抓取 → 摄像机前旋转采样 → 放回原位 → 机械臂归位** 流程

## 2. 架构设计

```
┌─────────────────────────────────────────────────────────────┐
│                    RobotSkill (调度层)                       │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │  /skill_sequence Action Server                       │   │
│  │         ↓                                            │   │
│  │    StepExecutor (执行 YAML 定义的步骤)               │   │
│  └─────────────────────────────────────────────────────────┘   │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │  /grasp_record Action Server                         │   │
│  │         ↓                                            │   │
│  │  GraspExecutor (TF 转换 + IK 验证)                   │   │
│  │         ↓                                            │   │
│  │    StepExecutor (复用)                               │   │
│  └─────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
                          ↓
              robot_driver Actions
```

### 2.1 设计原则

1. **调度与执行分离**：`RobotSkill` 只做调度，具体执行逻辑在 `StepExecutor`
2. **执行器复用**：`skill_sequence` 和 `grasp_record` 共用同一个 `StepExecutor`
3. **外部抓取规划**：抓取姿态由 perception 的 Contact-GraspNet 计算，本节点只做 TF 转换和 IK 验证

## 3. Action 定义

**文件**: `action/GraspRecord.action`

```yaml
# Goal
perception/GraspCandidate[] grasp_candidates  # 抓取候选数组（相机坐标系，按置信度降序）
string context_id                             # 任务上下文标识
---
# Result
bool success
string error_code
string message
int32 steps_executed
int32 candidate_used                          # 实际使用的候选索引（-1 表示全部失败）
---
# Feedback
string phase                                  # 当前阶段: tf_converting / ik_testing / executing
int32 step_index
string step_desc
float32 progress
int32 current_candidate_index                 # 当前尝试的候选索引
```

### 3.1 GraspCandidate 消息结构

来自 `perception/msg/GraspCandidate.msg`：

```yaml
# 位置（相机坐标系）
geometry_msgs/Point position           # 夹爪中心位置 (x, y, z) 单位：米

# 姿态（相机坐标系）
geometry_msgs/Quaternion orientation   # 夹爪朝向四元数 (x, y, z, w)

# 抓取参数
float32 width                          # 建议夹爪开口宽度（米）
float32 confidence                     # 置信度 (0.0 - 1.0)

# 可选：4x4 变换矩阵
float64[16] transform_matrix           # 4x4 齐次变换矩阵（展平，行优先）
```

## 4. GraspExecutor 类

**文件**: `src/robot_skill_core/grasp_executor.py`

负责 TF 转换、IK 验证和步骤序列生成。

### 4.1 核心方法

```python
class GraspExecutor:
    """处理抓取候选的 TF 转换与 IK 验证"""
    
    def __init__(self, node, observe_step, approach_distance, gripper_width_margin, max_gripper_width):
        # 服务客户端
        self._tf_poses_client = node.create_client(TransformPoses, '/tf_tools/service/transform_poses')
        self._ik_client = node.create_client(SolveIK, '/robot_driver/service/solve_ik')
    
    async def find_executable_candidate(
        self, candidates: List[GraspCandidate], camera_frame: str
    ) -> Tuple[int, PoseStamped, PoseStamped, float]:
        """
        按顺序尝试候选，返回第一个 IK 可解的
        
        流程：
        1. 批量转换所有候选的位姿到 base_link（1 次服务调用）
        2. 按顺序验证每个候选的 IK（同时验证抓取位和预抓取位）
        3. 返回第一个成功的
        
        Returns: (candidate_index, grasp_pose, pre_grasp_pose, gripper_width)
                 若全部失败则 candidate_index = -1
        """
    
    def build_grasp_steps(
        self, grasp_pose, pre_grasp_pose, gripper_width
    ) -> List[SkillStep]:
        """生成抓取-观察-放回步骤序列"""
```

### 4.2 IK 验证逻辑

- 先验证预抓取位姿 IK（grasp_pose.z + 0.08m）
- 再验证抓取位姿 IK
- 两者都成功才算该候选可用

### 4.3 夹爪宽度处理

```python
gripper_width = min(candidate.width + DEFAULT_GRIPPER_WIDTH_MARGIN, DEFAULT_MAX_GRIPPER_WIDTH)
```

其中：
- `DEFAULT_GRIPPER_WIDTH_MARGIN = 0.02`（2cm 安全余量）
- `DEFAULT_MAX_GRIPPER_WIDTH = 0.094`（94mm 夹爪最大开口）

## 5. 执行流程

```
┌──────────────────────────────────────────────────────────────┐
│  GraspRecord Action 调用                                     │
│  Input: GraspCandidate[] (相机坐标系, 按置信度排序)           │
└────────────────────────────┬─────────────────────────────────┘
                             ↓
┌──────────────────────────────────────────────────────────────┐
│  Phase 1: 批量 TF 转换（1 次服务调用）                        │
│  调用 /tf_tools/service/transform_poses                      │
│  输入: 所有候选的 Pose[]                                     │
│  转换: camera_color_optical_frame → base_link                │
│  输出: 转换后的 Pose[] (base_link 坐标系)                    │
└────────────────────────────┬─────────────────────────────────┘
                             ↓
┌──────────────────────────────────────────────────────────────┐
│  Phase 2: 逐个 IK 验证（按顺序尝试）                          │
│  for i, pose_base in enumerate(transformed_poses):           │
│    1. 计算预抓取位姿 (pose_base.z + 0.08m)                   │
│    2. 调用 /robot_driver/service/solve_ik 验证预抓取位       │
│    3. 若预抓取 IK 失败 → continue 下一个                     │
│    4. 调用 /robot_driver/service/solve_ik 验证抓取位         │
│    5. 若抓取 IK 失败 → continue 下一个                       │
│    6. 两者都成功 → 使用该候选，退出循环                       │
│  若全部失败 → 返回 ALL_IK_FAILED                             │
└────────────────────────────┬─────────────────────────────────┘
                             ↓
┌──────────────────────────────────────────────────────────────┐
│  Phase 3: 执行步骤                                           │
│  1. safety_pose      → 安全位姿                              │
│  2. cartesian_move   → 预抓取位                              │
│  3. gripper_open     → 张开夹爪 (width + 2cm margin)         │
│  4. cartesian_move   → 抓取位                                │
│  5. gripper_close    → 闭合夹爪                              │
│  6. joint_move       → 观察位 (复用 move_record_observe)     │
│  7. joint_move       → joint6 旋转扫描                       │
│  8. cartesian_move   → 放回原位                              │
│  9. gripper_open     → 张开夹爪                              │
│  10. safety_pose     → 安全位姿                              │
└──────────────────────────────────────────────────────────────┘
```

## 6. 依赖服务

| 服务 | 用途 |
|------|------|
| `/tf_tools/service/transform_poses` | 批量位姿坐标转换（camera → base_link） |
| `/robot_driver/service/solve_ik` | IK 解算验证 |
| `/robot_driver/action/robot` | 笛卡尔运动执行 |
| `/robot_driver/action/joint` | 关节运动执行 |
| `/robot_driver/action/gripper` | 夹爪控制 |
| `/robot_driver/action/safety_pose` | 安全位姿 |

## 7. 错误处理

| 错误码 | 触发条件 | 处理 |
|--------|---------|------|
| `NO_CANDIDATES` | grasp_candidates 为空 | 直接返回失败 |
| `TF_SERVICE_UNAVAILABLE` | transform_poses 服务不可用 | 等待 5s 后失败 |
| `TF_FAILED` | 位姿转换失败 | 返回失败 |
| `IK_SERVICE_UNAVAILABLE` | solve_ik 服务不可用 | 等待 5s 后失败 |
| `ALL_IK_FAILED` | 所有候选的 IK 都失败 | 返回失败 |
| `EXECUTION_FAILED` | 步骤执行失败 | 返回失败 |
| `CANCELED` | 用户取消 | 返回取消状态 |

## 8. 文件清单

| 文件 | 说明 |
|------|------|
| `action/GraspRecord.action` | Action 定义 |
| `src/robot_skill_core/grasp_executor.py` | TF 转换 + IK 验证 + 步骤生成 |
| `src/robot_skill_core/robot_skill.py` | 主节点，包含执行逻辑 |
| `src/robot_skill_core/step_executor.py` | 步骤执行器 |
| `src/robot_skill_core/constants.py` | 常量定义 |

## 9. 使用示例

```python
# 调用 GraspRecord Action
import rclpy
from rclpy.action import ActionClient
from robot_skill.action import GraspRecord
from perception.msg import GraspCandidate
from geometry_msgs.msg import Point, Quaternion

# 创建 Action Client
client = ActionClient(node, GraspRecord, '/robot_skill/action/grasp_record')

# 构建 Goal
goal = GraspRecord.Goal()

# 添加抓取候选（通常来自 /perception/service/grasp 的响应）
candidate1 = GraspCandidate()
candidate1.position = Point(x=0.5, y=0.2, z=0.05)
candidate1.orientation = Quaternion(x=0.0, y=0.707, z=0.0, w=0.707)
candidate1.width = 0.05
candidate1.confidence = 0.95

candidate2 = GraspCandidate()
candidate2.position = Point(x=0.52, y=0.18, z=0.06)
candidate2.orientation = Quaternion(x=0.0, y=0.707, z=0.0, w=0.707)
candidate2.width = 0.04
candidate2.confidence = 0.88

goal.grasp_candidates = [candidate1, candidate2]
goal.context_id = 'grasp_001'

# 发送 Goal
future = client.send_goal_async(goal)
```

## 10. 扩展性

这种分层设计便于未来扩展：
- `PlaceObject.action` - 放置物体到指定位置
- `Handover.action` - 递交物体给人
- `Inspect.action` - 仅检查物体（不抓取）

每个高层 Action 都可以复用底层的 `StepExecutor` 执行器和 `GraspExecutor` 的 TF/IK 验证能力。
