# robot_skill 节点需求草案

本文件用于与用户一起逐步补齐机器人技能规划与执行相关的节点需求。当前为初稿，请指出需要增删的内容。

## 1. 背景与目标
- 节点名称：`robot_skill`。
- 硬件背景：单机械臂，型号 YAM 6-DoF（见 `driver/robot_description.yaml`），单 RealSense D435i 用于目标检测与定位。
- 输入特性：上游视觉模块输出基于 `base_link` 坐标系的目标位姿（XYZ + 三维姿态信息），本节点需以此为起点规划连续指令序列。
- 主要目标：根据目标位置姿态三维信息，规划并执行一连串的机械臂运动指令，通过发布ROS2话题到`/robot_driver/robot_command`（消息类型：`geometry_msgs/msg/PoseStamped`）来控制机械臂执行复合技能（如抓取、放置等）。
  - 示例发布命令：`ros2 topic pub --once /robot_driver/robot_command geometry_msgs/msg/PoseStamped "{pose: {position: {x: 0.5, y: 0.4, z: 0.28}, orientation: {x: 0.0, y: 0.7071, z: 0.0, w: 0.7071}}}"`
- 约束重点：
-  - 遵守关节位置/速度限制（当前各关节速度上限约 10 rad/s，关节范围见 `driver/robot_description.yaml`）。
-  - 轨迹平滑与可达性：需时间参数化，避免突变；加速度/jerk 限制待确认。
-  - 避障：当前阶段暂不考虑外部障碍规避，主要关注自碰撞与几何可达性，后续可扩展。
- 场景：抓取、放置、对位等单臂任务。
- Driver 对接：通过话题`/robot_driver/robot_command`（`geometry_msgs/msg/PoseStamped`）与driver节点通信。
  - 消息格式：
    - `header.frame_id`: 默认 `base_link`（若使用其他坐标系需在 TF 中可解析）
    - `pose.position`: 目标位置 XYZ（单位：米）
      - 可达范围：x ≈ [-0.61, 0.60] m, y ≈ [-0.59, 0.61] m, z ≈ [-0.37, 0.72] m
      - 末端到 base_link 原点距离 r ≈ [0.04, 0.73] m
    - `pose.orientation`: 目标姿态（单位四元数 {x, y, z, w}）
      - 需规范化四元数，若启用 `xyz_only_mode` 则忽略姿态，沿用当前值
  - 驱动行为：
    - 执行 IK 求解后在关节空间插值
    - 根据当前-目标关节最大夹角自动计算 ramp 速度（0.25–5 s）
    - 若 IK 失败或目标超出可达区，机械臂保持不动，日志输出 `IK failed for target pose`
    - 新指令会抢占旧指令（支持 10–30 Hz 连续推送）
  - 示例（与上述主要目标一致）：
    ```yaml
    header:
      frame_id: base_link
      stamp: {sec: 0, nanosec: 0}
    pose:
      position: {x: 0.35, y: 0.0, z: 0.32}
      orientation: {x: 0.0, y: 0.707, z: 0.0, w: 0.707}
    ```
