# robot_desc_node 技术方案

## 1. 目标与约束
- 在 ROS 2（rclcpp）环境中实现 `robot_desc_node`，以字符串形式发布 `/robot_description` 话题，供 `robot_state_publisher`、`rviz2` 等消费。
- 支持加载单一 URDF 文件或由 xacro 生成的 URDF，解析过程中自动解析 mesh 资源。
- 节点启动时间不超过 1 s；发布频率保持在参数化（默认 1 Hz）以兼容热加载。
- Mesh 资源与 URDF 保持包内相对路径，避免硬编码绝对路径。

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
   - 依赖 `ament_index_cpp::get_package_share_directory` 或 `ament_index_python` 查找包路径。
   - 对 `package://` URI 做正则拆解，转为绝对路径；同时校验 mesh 文件存在性，缺失则抛出可诊断异常。
2. **URDF 生成层**
   - 若 `urdf_path` 后缀为 `.xacro`，通过 `xacro` CLI 或 `libxacro` 运行生成临时 URDF 字符串；生成过程缓存于内存，避免磁盘中间文件。
   - 解析结果经 `urdfdom` 验证结构，必要时写入 `rclcpp::Logger`。
   - `frame_prefix` 通过 DOM 遍历批量更新 link/joint 名称。
3. **发布层**
   - 使用定时器按照 `publish_rate` 发布；额外提供 `std_srvs/Trigger` 服务 `reload_description` 触发重新加载（便于在 mesh/URDF 更新后热替换）。
   - 节点自检通过参数事件通知（`on_parameter_event`），便于 launch 文件检测加载是否成功。
4. **监控与容错**
   - 启动时打印摘要（link/joint 数）；mesh 丢失或解析失败进入 `lifecycle` `errorprocessing` 状态，等待外部管理器 reset。

## 4. 项目目录布局
```
description/
├── CMakeLists.txt     # ROS 构建配置（可执行与安装规则）
├── package.xml        # 包元数据，声明依赖/版本/维护者
├── urdf/              # 主 URDF 或 xacro 模型及宏库
├── meshes/            # STL/DAE/GLB 等网格文件，按 link 分类
├── config/            # YAML 等参数文件（publish_rate、frame_prefix 等）
├── launch/            # Python/XML launch，组合 robot_desc_node 与 rsp/rviz
├── src/               # C++/Python 源码；robot_desc_node、工具脚本
```
- 该布局与 ROS 2 ament/colcon 约定兼容，可直接 `colcon build`。
- `launch/` 中通过 `DeclareLaunchArgument` 显式暴露 `urdf_path`、`xacro_args`、`frame_prefix`，并把 `config/` 下的参数 YAML 加载进节点。
- mesh、URDF 使用包内相对路径，可在 `install` 后通过 `ament_index` 正确查找。
- 如需扩展额外资源（如 `rviz/` 配置、`tests/`），可在同级追加目录但保持核心结构不变。

## 5. 测试与验证
- **静态检查**：CI 中运行 `xacro --inorder`、`check_urdf` 校验拓扑。
- **单元测试**：利用 `rclcpp::Test` 检查节点在参数变化时是否重新发布；mock mesh 缺失场景验证错误路径。
- **集成测试**：Launch test 启动 `robot_desc_node + robot_state_publisher + rviz2 --validate-config`，确保 TF 树完整。
- **可视化验证**：`rviz2` 中加载 `/robot_description`，确认 mesh 贴图与原始 CAD 一致。

## 6. 后续扩展
- 增加 `resource_spawner` 插件以监听 `/robot_description_updates`，当 URDF 有重大更新时触发其他节点重启。
- 支持 `rosbag2` 回放模式：在 bag metadata 中存储 URDF 哈希，用于一致性校验。
- 若需多机器人实例，结合 `namespace` 与 `use_sim_time`，在 launch 中复制多个 `robot_desc_node`。
