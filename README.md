# RealSense + Robotic Arm Integration Suite

## 项目简介
该项目旨在构建一套完整的 RealSense 深度相机与机械臂联动方案，通过统一的应用服务接口在局域网中对外暴露能力，便于上层 App 或业务系统调用，实现从视觉感知到执行控制的闭环。

## 工作区结构
- `src/`：ROS 2 工作空间源码，按模块拆分为 `bringup/`（统一 launch）、`description/`（URDF/Mesh 与 `robot_desc_node`）、`driver/`（`robot_driver_node`）、`move_controller/`（轨迹与逆解控制）、`skill/`（常用动作/技能封装）、`perception/`（2D 感知节点）、`vision/`（3D 映射与相机接入）、`tf_tools/`（静态 TF 配置发布）、`app/`（局域网服务端）、`web/`（Web/foxglove 可视化桥）、`viz/`（RViz2 配置）、`msgs/`（自定义消息/服务）。
- `build/`：`colcon build` 的中间产物，按包生成 `CMake` 与对象缓存。
- `install/`：`colcon` 安装结果，包含可执行体、资源与环境脚本。
- `log/`：`ros2 launch/run` 与 `colcon` 的日志输出，便于定位编译与运行问题。
- `start_robot.sh`：实机部署一键启动脚本（后续填入 bringup 组合 launch）。
- `start_sim.sh`：仿真环境入口脚本（预留用于 Gazebo/RViz 仿真流程）。

## 运行环境
- Ubuntu 22.04（LTS）作为基础操作系统，统一驱动与依赖版本。
- ROS 2 Humble 作为机器人中间件，提供节点管理、话题/服务/动作通信、TF 广播与调试工具链。
- RealSense 深度相机：已接入 Intel RealSense D435i（Bus 002 Device 008，Serial 253243061481），可通过 `lsusb | grep 435i` 或 `udevadm info --name=/dev/bus/usb/002/008` 即时校验硬件与序列号。
- 机械臂：I2RT YAM 6 轴 + 1 夹爪机械臂，使用 1 Mbps 的 CAN 2.0B 总线（当前 `can0~can2` 处于 `UP` 状态，默认 `can1` 供 follower 臂及 `linear_4310` 夹爪使用）。

## 传感器与执行器详情

### RealSense D435i
- 型号：Intel RealSense Depth Camera D435i（Product ID 8086:0b3a，固件版本 0x50f1）。
- 序列号：253243061481，可通过 `udevadm info --name=/dev/bus/usb/002/008 | grep ID_SERIAL` 获取并在上位应用中绑定。
- 节点配置：`realsense2_camera` 启动后使用 `rs435i` 默认 profile，自动发布 `/camera/color/*`、`/camera/depth/*` 与 `/tf`；若需多相机扩展请在 `device_type:=d435i` 基础上加 `serial_no:=253243061481` 以确保节点唯一性。
- 故障排查：当 `rs-enumerate-devices` 受限于 `librealsense2` 运行库时，可先通过 `lsusb`/`udevadm` 确认硬件，再补充 `librealsense2.so` 或使用容器镜像内自带驱动。

### I2RT YAM 机械臂（linear_4310 夹爪）

| 项目 | 数值 |
| --- | --- |
| 自由度 | 6 轴关节 + 1 夹爪 |
| 臂展 | ≈ 0.5 m |
| 额定载荷 | ≈ 1 kg |
| 重复定位精度 | ±2 mm |
| 通信 | CAN 2.0B @ 1 Mbps（默认使用 `can1`，`can0/can2` 备用或 leader/follower 双臂模式） |
| 电源 | 24 V DC |

#### 关节限制（弧度制）

| 关节 | 最小 | 最大 | 备注 |
| --- | --- | --- | --- |
| J1（基座） | -2.618 | 3.13 | 双向 327° |
| J2（肩部） | 0 | 3.65 | 单向，靠近机械限位 |
| J3（肘部） | 0 | 3.13 | 单向 |
| J4（腕 1） | -1.57 | 1.57 | 双向 180° |
| J5（腕 2） | -1.57 | 1.57 | 双向 180° |
| J6（腕 3） | -2.09 | 2.09 | 双向 240° |

#### 推荐工作空间

| 轴向 | 范围 (m) | 说明 |
| --- | --- | --- |
| X | 0.20 ~ 0.45 | 前向伸展 |
| Y | -0.15 ~ 0.15 | 以基座中心为零，左负右正 |
| Z | 0.25 ~ 0.45 | 工作高度 |

- 推荐初始位姿：`(0.35, 0.00, 0.35)`，夹爪闭合，确保进入轨迹规划前处于安全区间。
- 控制脚本：在 `/home/jetson/I2RT/i2rt` 中执行 `./start_move_to_position.sh`（交互模式）或 `python3 coordinate_control/move_to_position_demo.py --mode demo`（自动模式）即可复现 i2rt 官方示例；`python3 test_connection.py` 用于诊断 CAN 与电机状态。
- 安全要点：保持 `pkill -f "motor_chain_robot.py.*can1"` 清理旧进程，IK 失败会触发工作空间边界告警，必要时运行 `scripts/reset_all_can.sh` 重置总线设备。

## Node 拆分
| 模块 | Node | 描述 |
| --- | --- | --- |
| 相机 | `realsense2_camera` | RealSense 官方驱动节点，管理相机硬件、内参发布与话题输出。 |
| 视觉 | `object_perception_2d_node` | 负责 2D Mask 与物体识别/记忆，输出语义分割或检测结果。 |
|  | `vision_3d_mapper_node` | 将 2D Mask 投影到 3D 点云，生成世界/相机坐标系下的目标姿态。 |
| 机械臂 | `robot_driver_node` | 读写机械臂控制器：实时采集关节位置/速度/力矩，并对目标关节位姿做插值、限速与安全检查后下发。 |
|  | `move_controller_group_node` | 根据 3D 感知结果生成轨迹规划，负责逆解、避障与时序控制。 |
|  | `robot_desc_node` | 发布 URDF 与 Mesh 资源，供 RViz/MoveIt 等可视化与规划模块消费。 |
|  | `robot_skill_node` | 封装常用动作集（抓取、旋转采样等），对上层提供技能级指令。 |
| TF | `static_tf_config` (Action) | 生成手眼标定与全局静态坐标配置，保障多坐标系一致性。 |
|  | `static_tf_publisher` (Action) | 广播 `/tf` 与 `/tf_static`，同步机器人各坐标系给可视化、导航、控制栈。 |
| App | `robot_server` | 面向局域网对外的服务节点，处理任务下发、状态查询、告警通知等 API。 |
|  | `robot_web` | Web 可视化/管理节点，为前端 App 提供实时状态与交互界面。 |
| 可视化 | `foxglove_bridge` | 与 Foxglove Studio 对接，推送话题/服务给 Web 可视化客户端。 |
|  | `rviz2` | RViz2 实例节点，用于调试、场景复现与演示。 |

## 核心目标
1. **多模态感知**：利用 RealSense 提供 RGB-D 数据，完成场景建模、目标检测与位姿估计。
2. **联动控制**：将感知结果实时转化为机械臂的轨迹规划与动作执行，确保安全、稳定、低延迟。
3. **服务化接口**：封装标准化的局域网 API（REST/gRPC/WebSocket 等），供外部 App 调用，实现状态获取与任务下发。

若有额外需求（如跨平台支持、云端同步、视觉算法扩展），请在需求评审阶段确认并统筹进度。
