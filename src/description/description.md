# robot_desc_node 技术方案

## 1. 目标与背景
- 在 ROS 2 Humble（rclpy）环境中实现纯 Python 的 `robot_desc_node`，以字符串形式发布 `/robot_description` 话题，供 `robot_state_publisher`、`rviz2`、`Foxglove` 等工具消费。
- 支持加载单一 URDF 文件或由 xacro 生成的 URDF，解析过程中自动解析 mesh 资源。
- 节点启动时间不超过 1 s；发布频率保持在参数化（默认 1 Hz）以兼容热加载。

## 2. 实现步骤
- **Step 1 – 资产校验**：节点启动前执行 URDF 与 mesh 的一致性检查。
  - 解析 `urdf_path`（支持 `package://`），定位文件并确认可读；
  - 针对 `.xacro` 文件先调用 `xacro` 生成中间 URDF，再使用 `urdf_parser_py` 或 `urdfpy` 做 XML/引用校验；
  - 遍历 URDF 中 `<mesh filename="package://...">` 字段，确认目标文件相对 `share/description/meshes` 存在，缺失时提前报错而不是让节点空发布；
- `robot_desc_check`：新增的 CLI（`ros2 run description robot_desc_check -- --urdf-path …`）会完成上述检查，并把 JSON 日志写入 `../../log/robot_desc_check.log`，供 CI 与本地调试统一使用。`start_robot_desc.sh` 默认会先执行该 CLI，失败则阻止节点启动。
- **Step 2 – 部署 `robot_desc_node`**：
  - `launch/robot_desc.launch.py` 声明 `urdf_path`、`xacro_args`（逗号分隔字符串）、`frame_prefix`、`publish_rate`、`namespace`、`node_name` 等 Launch 参数，并以 `ParameterValue` 形式传入节点，满足仿真/真机多场景复用。
  - 节点初始化时立即调用 `_load_description()` 并注册 `reload_description` Trigger 服务，外部在更新 URDF/mesh 后调用即可热更新。
  - `start_robot_desc.sh` 统一入口负责：
    - 自动定位仓库根目录并写日志到 `../../log/robot_desc_node.log`；
    - 按顺序 source `install/setup.bash` 或 `/opt/ros/humble/setup.bash`；
    - 支持 `--urdf-path/--publish-rate/--frame-prefix/--xacro-arg` 等参数，同时允许运行者通过 `--skip-check` 跳过资产校验或在 `--` 之后拼接额外 `ros2 run` 参数；
    - 校验通过后再执行 `ros2 run description robot_desc_node --ros-args ...`，确保实际发布的话题与校验输入一致。
- **Step 3 – 发布 `/robot_description` 并对接 Foxglove**：
  - `robot_desc_node` 通过 QoS depth=1 的 `std_msgs/msg/String` 发布 URDF 字符串，Foxglove 3.2.1+ 在 `foxglove_bridge` WebSocket 中自动下发 `package://` mesh；
  - 若使用不支持 asset fetching 的桥接（例如传统 `rosbridge_server`），需把 mesh 托管到 HTTP 服务并在 xacro 中把 `package://` 动态改写成 URL；
  - 将 `/robot_description` 加入 `foxglove_bridge` 的 topic allowlist，同时在 Foxglove 3D 面板中确认模型完整、材质/缩放正确；
  - 在 bringup/CI 流程里订阅 `/robot_description` 做一次非空断言，保证 `robot_state_publisher`、可视化工具不会拉到空模型。

## 3. 运行与验证
- 使用 `colcon build` 安装后运行 `ros2 launch description robot_desc.launch.py`，确认日志打印 `Loaded robot description from …`；
- 执行 `ros2 service call /robot_desc_node/reload_description std_srvs/srv/Trigger {}` 验证热加载；
- `ros2 topic echo /robot_description --once`，确保消息包含 `<robot`；
- Foxglove 验证：
  - Browser/Studio 连接 `foxglove_bridge`（3.2.1）；
  - 在 3D Panel 中勾选 `/robot_description` 与 `/tf`，确认模型与 TF 对齐；
  - 若 mesh 缺失，优先查看 `../../log/robot_desc_check.log` 里的资产校验记录，再确认 HTTP/包路径可达。

## 4. 项目目录布局
```
description/
├── package.xml          # ament_python 元数据（依赖 rclpy、xacro 等）
├── setup.cfg / setup.py # Python 构建入口，注册 console_scripts
├── resource/description # ament 索引用，占位即可
├── config/              # YAML/说明文档（如 robot_description.txt, yam.xml）
├── urdf/                # 主 URDF/xacro 文件（yam.urdf 等）
├── meshes/              # STL/DAE/GLB 等网格文件
├── launch/              # Python launch，声明参数并启动节点
├── src/description/
│   ├── __init__.py
│   └── robot_desc_node.py # 纯 Python 实现的发布节点
└── description.md       # 技术方案文档
```
`setup.py` 将 `config/`、`urdf/`、`meshes/`、`launch/` 安装到 `share/description`；`robot_desc_node.py` 通过 `console_scripts` 暴露，launch 里直接引用。
