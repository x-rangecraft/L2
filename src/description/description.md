# robot_desc_node 技术方案

## 1. 目标与约束
- 在 ROS 2 Humble（rclpy）环境中实现纯 Python 的 `robot_desc_node`，以字符串形式发布 `/robot_description` 话题，供 `robot_state_publisher`、`rviz2`、`Foxglove` 等工具消费。
- 支持加载单一 URDF 文件或由 xacro 生成的 URDF，解析过程中自动解析 mesh 资源。
- 节点启动时间不超过 1 s；发布频率保持在参数化（默认 1 Hz）以兼容热加载。

## 2. 输入与输出
- **参数输入**：
  - `urdf_path`：URDF/xacro 主文件绝对路径或 `package://` URI。
  - `xacro_args`：可选，传入宏参数以动态配置尺寸、传感器等。
  - `publish_rate`：`/robot_description` 发布频率（Hz），默认 1。
  - `frame_prefix`：可选，批量为 link/joint 名称添加前缀。
- **文件依赖**：URDF 中引用的 mesh（STL/DAE/GLB）位于同一包 `meshes/` 目录，使用 `package://<pkg>/meshes/...` 形式。
- **输出**：`/robot_description`（`std_msgs::msg::String`）携带完整 URDF 文本，可被其他节点订阅；必要时同时发布 `robot_description_semantic`。

## 3. 节点架构
1. **资源解析层**
   - 使用 `ament_index_python.get_package_share_directory` 查找包路径，统一解析 `package://` URI 并校验 mesh/URDF 存在性。
   - 支持在工作空间内相对路径与安装后 share 目录下的绝对路径无缝切换。
2. **URDF 生成层**
   - `.xacro` 文件通过 Python `xacro` API 解析为字符串，必要时配置 `xacro_args` 动态替换参数。
   - URDF 文本可选追加 `frame_prefix`，并在加载后缓存至内存，避免重复 IO。
3. **发布层**
   - 使用 `rclpy` 定时器按照 `publish_rate` 发布 `std_msgs/String`；提供 `std_srvs/Trigger` 服务 `reload_description` 触发热加载。
   - 启动时立即发布一次；加载失败以 `rclpy.logging` 输出可诊断信息。
4. **监控与容错**
   - 通过参数事件或 service 反馈加载状态，必要时在 launch/监控节点中重试。
   - 预留后续与 ROS 2 Lifecycle 节点集成的接口，以便在更严格场景下管理状态机。

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
└── description.md       # 技术方案文档（可选安装到 share/description）
```
- `setup.py` 将 `config/`、`urdf/`、`meshes/`、`launch/` 安装到 `share/description`，方便运行期查找。
- `robot_desc_node.py` 通过 `console_scripts` 暴露，在 launch 中以 `description` 包直接引用。
- `config/` 除参数 YAML 外也可存放 Mujoco XML、硬件速查（`robot_description.txt`），launch 里根据需要加载。
- `src/description` 可继续扩展辅助脚本（如 mesh 校验器），保持纯 Python 架构。

## 5. 测试与验证
- **静态检查**：CI 中运行 `xacro --inorder`、`check_urdf` 或 `urdf_to_graphiz` 校验拓扑。
- **单元测试**：使用 `pytest` + `rclpy` 的测试基类，注入假参数/文件并验证 `/robot_description` 发布内容；mock mesh 缺失场景应返回错误。
- **集成测试**：`launch_testing` 启动 `robot_desc_node + robot_state_publisher + rviz2 --validate-config`，确保 TF 树完整并可在 Foxglove 中可视化。
- **可视化验证**：`rviz2` 中加载 `/robot_description`，确认 mesh 贴图与原始 CAD 一致。

## 6. 后续扩展
- 增加 `resource_spawner` 插件以监听 `/robot_description_updates`，当 URDF 有重大更新时触发其他节点重启。
- 支持 `rosbag2` 回放模式：在 bag metadata 中存储 URDF 哈希，用于一致性校验。
- 若需多机器人实例，结合 `namespace` 与 `use_sim_time`，在 launch 中复制多个 `robot_desc_node`。
