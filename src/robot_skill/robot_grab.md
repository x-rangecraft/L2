# 动态抓取技能实现方案 (GraspRecord)

本文档描述在 `robot_skill` 节点中新增动态抓取功能的技术方案。

## 1. 需求概述

- **输入**：物体点云的边界坐标（boundary_points）
- **输出**：基于 Open3D OBB 计算最佳抓取姿态，执行 **抓取 → 摄像机前旋转采样 → 放回原位 → 机械臂归位** 流程

## 2. 架构设计

```
┌─────────────────────────────────────────────────────────────┐
│                    RobotSkill (调度层)                       │
│  ┌─────────────────────────────────────────────────────┐   │
│  │  /skill_sequence Action Server                       │   │
│  │         ↓                                            │   │
│  │    StepExecutor (执行 YAML 定义的步骤)               │   │
│  └─────────────────────────────────────────────────────┘   │
│  ┌─────────────────────────────────────────────────────┐   │
│  │  /grasp_record Action Server (新增)                  │   │
│  │         ↓                                            │   │
│  │  GraspPlanner (Open3D OBB 计算抓取姿态)              │   │
│  │         ↓                                            │   │
│  │    StepExecutor (复用)                               │   │
│  └─────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
                          ↓
              robot_driver Actions
```

### 2.1 设计原则

1. **调度与执行分离**：`RobotSkill` 只做调度，具体执行逻辑在 `StepExecutor`
2. **执行器复用**：`skill_sequence` 和 `grasp_record` 共用同一个 `StepExecutor`
3. **动态规划**：`GraspPlanner` 基于点云动态计算抓取姿态，不依赖硬编码位姿

## 3. 实现步骤

### 3.1 新增 Action 定义

**文件**: `action/GraspRecord.action`

```
# Goal
geometry_msgs/Point[] boundary_points     # 物体边界点云
float32 max_gripper_width                 # 夹爪最大张开宽度 (m)
string context_id
---
# Result
bool success
string error_code
string message
int32 steps_executed
---
# Feedback
string phase                              # planning / executing
int32 step_index
string step_desc
float32 progress
```

### 3.2 抽取 StepExecutor 类

**文件**: `src/robot_skill_core/step_executor.py`

从现有 `node.py` 中抽取步骤执行逻辑：

```python
class StepExecutor:
    """执行 SkillStep 序列的核心类"""
    
    def __init__(self, node: RclpyNode):
        self._node = node
        self._robot_client = ActionClient(node, RobotCommand, '/robot_driver/action/robot')
        self._joint_client = ActionClient(node, JointCommand, '/robot_driver/action/joint')
        self._gripper_client = ActionClient(node, GripperCommand, '/robot_driver/action/gripper')
        self._safety_client = ActionClient(node, SafetyPose, '/robot_driver/action/safety_pose')
    
    async def execute_step(self, step: SkillStep) -> Tuple[bool, str, str]:
        """执行单个步骤（迁移自原 node.py）"""
        ...
    
    async def execute_steps(
        self, 
        steps: List[SkillStep], 
        on_progress: Callable = None,
        check_cancel: Callable = None
    ) -> Tuple[bool, str, str, int]:
        """执行步骤序列，支持进度回调和取消检查"""
        ...
```

### 3.3 新增 GraspPlanner 类

**文件**: `src/robot_skill_core/grasp_planner.py`

基于 Open3D OBB（Oriented Bounding Box）计算最佳抓取姿态：

```python
import open3d as o3d
import numpy as np
from scipy.spatial.transform import Rotation

class GraspPlanner:
    """基于 Open3D OBB 计算抓取姿态并生成步骤序列"""
    
    def __init__(self, observe_pose: PoseStamped, approach_distance: float = 0.08):
        self._observe_pose = observe_pose
        self._approach_distance = approach_distance
    
    def compute_grasp_pose_from_boundary(
        self, 
        boundary_points: List[Point], 
        max_gripper_width: float
    ) -> Tuple[PoseStamped, float]:
        """
        基于边界点云计算最佳抓取姿态
        返回: (grasp_pose, gripper_width)
        """
        # 1. 转换为 numpy 数组
        points = np.array([[p.x, p.y, p.z] for p in boundary_points])
        
        # 2. 创建 Open3D 点云并计算 OBB
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        obb = pcd.get_oriented_bounding_box()
        
        center = obb.center           # 抓取中心 [x, y, z]
        R = np.array(obb.R)           # 旋转矩阵（三个主轴方向）
        extent = np.array(obb.extent) # 三个轴的尺寸 [dx, dy, dz]
        
        # 3. 选择最佳抓取轴（宽度 <= 夹爪最大宽度，优先最窄）
        graspable_axes = [
            (i, extent[i]) for i in range(3) 
            if extent[i] <= max_gripper_width
        ]
        
        if not graspable_axes:
            raise ValueError(f"物体最小尺寸 {min(extent):.3f}m > 夹爪最大宽度 {max_gripper_width:.3f}m")
        
        best_axis_idx, gripper_width = min(graspable_axes, key=lambda x: x[1])
        gripper_width = min(gripper_width + 0.02, max_gripper_width)  # 留余量
        
        # 4. 构建抓取姿态
        grasp_pose = self._build_grasp_pose(center, R, best_axis_idx)
        
        return grasp_pose, gripper_width
    
    def _build_grasp_pose(self, center: np.ndarray, R: np.ndarray, grasp_axis: int) -> PoseStamped:
        """
        构建抓取位姿
        - 位置：OBB 中心
        - 姿态：末端 Z 轴朝下，夹爪沿 grasp_axis 方向闭合
        """
        pose = PoseStamped()
        pose.header.frame_id = 'base_link'
        pose.pose.position.x = center[0]
        pose.pose.position.y = center[1]
        pose.pose.position.z = center[2]
        
        # 构建旋转矩阵：
        # - 末端 Z 轴朝下 (0, 0, -1)
        # - 夹爪闭合方向沿 OBB 的 grasp_axis
        grasp_dir = R[:, grasp_axis]  # 抓取方向（夹爪闭合方向）
        
        # 计算末端坐标系
        z_axis = np.array([0, 0, -1])  # 末端 Z 轴朝下
        x_axis = grasp_dir / np.linalg.norm(grasp_dir)  # 夹爪方向
        y_axis = np.cross(z_axis, x_axis)
        y_axis = y_axis / np.linalg.norm(y_axis)
        x_axis = np.cross(y_axis, z_axis)  # 重新正交化
        
        rot_matrix = np.column_stack([x_axis, y_axis, z_axis])
        quat = Rotation.from_matrix(rot_matrix).as_quat()  # [x, y, z, w]
        
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        
        return pose
    
    def compute_pre_grasp_pose(self, grasp: PoseStamped) -> PoseStamped:
        """预抓取位姿 = 抓取位姿上方"""
        pre = copy.deepcopy(grasp)
        pre.pose.position.z += self._approach_distance
        return pre
    
    def build_grasp_record_steps(
        self, 
        boundary_points: List[Point], 
        max_gripper_width: float
    ) -> List[SkillStep]:
        """生成完整的抓取-观察-放回步骤序列"""
        grasp_pose, gripper_width = self.compute_grasp_pose_from_boundary(
            boundary_points, max_gripper_width
        )
        pre_grasp_pose = self.compute_pre_grasp_pose(grasp_pose)
        
        return [
            SkillStep('safe_start', 'safety_pose', '回到安全位姿', {}),
            SkillStep('pre_grasp', 'cartesian_move', '移到预抓取位', 
                      {'target_pose': pre_grasp_pose, 'speed_scale': 1.0}),
            SkillStep('open_gripper', 'gripper', '张开夹爪',
                      {'command': 0, 'width_m': gripper_width}),
            SkillStep('grasp', 'cartesian_move', '移到抓取位',
                      {'target_pose': grasp_pose, 'speed_scale': 0.5}),
            SkillStep('close_gripper', 'gripper', '闭合夹爪', {'command': 1}),
            SkillStep('observe', 'cartesian_move', '移到观察位',
                      {'target_pose': self._observe_pose, 'speed_scale': 1.0}),
            SkillStep('sweep', 'joint_move', 'Joint6扫描',
                      {'joint_index': 5, 'positions': [-2.14, 2.14, -2.14]}),
            SkillStep('place', 'cartesian_move', '放回原位',
                      {'target_pose': grasp_pose, 'speed_scale': 0.5}),
            SkillStep('release', 'gripper', '张开夹爪', {'command': 0}),
            SkillStep('safe_end', 'safety_pose', '回到安全位姿', {}),
        ]
```

### 3.4 重构主节点

**重命名**:
- 文件：`node.py` → `robot_skill.py`
- 类名：`RobotSkillNode` → `RobotSkill`
- 节点名称保持：`robot_skill`

```python
# src/robot_skill_core/robot_skill.py

class RobotSkill(RclpyNode):
    """机器人技能调度节点"""
    
    def __init__(self):
        super().__init__('robot_skill')
        
        # 共享的步骤执行器
        self._step_executor = StepExecutor(self)
        
        # 抓取规划器
        self._grasp_planner = GraspPlanner(
            observe_pose=OBSERVE_POSE,
            approach_distance=DEFAULT_APPROACH_DISTANCE
        )
        
        # 技能库（用于 skill_sequence）
        self._skill_library = SkillLibrary(skill_sets_dir)
        
        # Action Server 1: skill_sequence（保持）
        self._skill_server = ActionServer(
            self, SkillSequence, '/robot_skill/action/skill_sequence',
            execute_callback=self._execute_skill_sequence)
        
        # Action Server 2: grasp_record（新增）
        self._grasp_server = ActionServer(
            self, GraspRecord, '/robot_skill/action/grasp_record',
            execute_callback=self._execute_grasp_record)
    
    async def _execute_skill_sequence(self, goal_handle):
        """执行 YAML 定义的技能"""
        skill_def = self._skill_library.load_skill(goal_handle.request.skill_id)
        success, code, msg, count = await self._step_executor.execute_steps(
            skill_def.steps,
            on_progress=lambda i, s: self._publish_skill_feedback(goal_handle, i, s),
            check_cancel=lambda: goal_handle.is_cancel_requested
        )
        # 返回结果...
    
    async def _execute_grasp_record(self, goal_handle):
        """执行动态抓取"""
        req = goal_handle.request
        
        # 1. 规划：基于 OBB 生成步骤序列
        try:
            steps = self._grasp_planner.build_grasp_record_steps(
                list(req.boundary_points),
                req.max_gripper_width
            )
        except ValueError as e:
            # 物体太大无法抓取
            return self._grasp_failure(goal_handle, 'PLANNING_FAILED', str(e))
        
        # 2. 执行：复用 StepExecutor
        success, code, msg, count = await self._step_executor.execute_steps(
            steps,
            on_progress=lambda i, s: self._publish_grasp_feedback(goal_handle, i, s),
            check_cancel=lambda: goal_handle.is_cancel_requested
        )
        # 返回结果...
```

## 4. 抓取算法详解

### 4.1 Open3D OBB 算法流程

```
输入: boundary_points[] + max_gripper_width
                    ↓
    ┌─────────────────────────────────────┐
    │ Open3D 计算 Oriented Bounding Box   │
    │ → center (中心点)                   │
    │ → R (旋转矩阵，三个主轴方向)         │
    │ → extent (三轴尺寸 [dx, dy, dz])    │
    └─────────────────────────────────────┘
                    ↓
    ┌─────────────────────────────────────┐
    │ 筛选可抓取轴                        │
    │ extent[i] <= max_gripper_width      │
    └─────────────────────────────────────┘
                    ↓
    ┌─────────────────────────────────────┐
    │ 选择最窄轴作为抓取方向               │
    │ → 更稳定的抓取                      │
    └─────────────────────────────────────┘
                    ↓
    ┌─────────────────────────────────────┐
    │ 构建抓取姿态                        │
    │ - 位置: OBB 中心                    │
    │ - 姿态: Z轴朝下，夹爪沿最窄轴闭合    │
    └─────────────────────────────────────┘
                    ↓
    生成步骤序列 → StepExecutor 执行
```

### 4.2 抓取方向选择逻辑

对于一个长方体物体，OBB 会给出三个主轴及其尺寸：
- 假设尺寸为 `[0.15, 0.05, 0.08]` (长、宽、高)
- 夹爪最大宽度为 `0.094m`

则：
1. 轴0 (0.15m) > 0.094m → 不可抓取
2. 轴1 (0.05m) <= 0.094m → 可抓取 ✓
3. 轴2 (0.08m) <= 0.094m → 可抓取 ✓

选择最窄的轴1作为抓取方向，夹爪张开宽度 = 0.05 + 0.02 = 0.07m

## 5. 执行流程

```
    ┌──────────────────────────────────────────────────────────────┐
    │ 1. safety_pose       → 安全位姿                              │
    │ 2. cartesian_move    → 预抓取位（OBB中心上方8cm）             │
    │ 3. gripper_open      → 张开夹爪（OBB最窄轴宽度+余量）         │
    │ 4. cartesian_move    → 抓取位（OBB中心，姿态自动计算）        │
    │ 5. gripper_close     → 闭合夹爪                              │
    │ 6. cartesian_move    → 观察位（固定）                        │
    │ 7. joint_move        → joint6 旋转扫描                       │
    │ 8. cartesian_move    → 放回原位                              │
    │ 9. gripper_open      → 张开夹爪                              │
    │ 10. safety_pose      → 安全位姿                              │
    └──────────────────────────────────────────────────────────────┘
```

## 6. 文件变更清单

| 文件 | 操作 | 说明 |
|------|------|------|
| `action/GraspRecord.action` | 新增 | 抓取任务 Action 定义 |
| `src/robot_skill_core/step_executor.py` | 新增 | 步骤执行器（从原 node.py 抽取） |
| `src/robot_skill_core/grasp_planner.py` | 新增 | Open3D OBB 抓取规划器 |
| `src/robot_skill_core/node.py` → `robot_skill.py` | 重命名+重构 | 类名改为 RobotSkill |
| `src/robot_skill_core/constants.py` | 修改 | 新增 OBSERVE_POSE 等常量 |
| `CMakeLists.txt` | 修改 | 注册新 Action |
| `setup.py` | 修改 | 更新入口点 |
| `package.xml` | 修改 | 添加依赖 |

## 7. 新增依赖

```xml
<!-- package.xml -->
<exec_depend>python3-open3d</exec_depend>
<exec_depend>python3-scipy</exec_depend>
```

安装命令：
```bash
pip install open3d scipy
```

## 8. 使用示例

```python
# 调用 GraspRecord Action
import rclpy
from rclpy.action import ActionClient
from robot_skill.action import GraspRecord
from geometry_msgs.msg import Point

# 创建 Action Client
client = ActionClient(node, GraspRecord, '/robot_skill/action/grasp_record')

# 构建 Goal
goal = GraspRecord.Goal()
goal.boundary_points = [
    Point(x=0.5, y=0.2, z=0.05),
    Point(x=0.55, y=0.2, z=0.05),
    Point(x=0.5, y=0.25, z=0.05),
    Point(x=0.55, y=0.25, z=0.05),
    # ... 更多边界点
]
goal.max_gripper_width = 0.094  # 夹爪最大张开 94mm
goal.context_id = 'grasp_001'

# 发送 Goal
future = client.send_goal_async(goal)
```

## 9. 扩展性

这种分层设计便于未来扩展：
- `PlaceObject.action` - 放置物体到指定位置
- `Handover.action` - 递交物体给人
- `Inspect.action` - 仅检查物体（不抓取）

每个高层 Action 都可以复用底层的 `StepExecutor` 执行器。

