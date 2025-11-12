# TF Tools 静态 TF 配置模块文档

本文档聚焦当前阶段已经敲定的需求与技术方案，目标是让团队在实现前对目标、边界和交付物有清晰共识。

## 1. 需求与背景
### 1.1 目标
- 以统一的配置驱动 world→base_link、world→camera_link 等静态坐标关系。
- 根据现场输入的平移/旋转参数自动生成 `tf_static` 所需的配置文件。
- 通过静态 TF 发布流程，使 Foxglove / RViz2 中的设备模型与真实世界坐标对齐。

### 1.2 输入
- 交互式输入/修改：
  - 机械臂 `base_link` 在 `world` 坐标系下的平移 (x, y, z)。
  - 机械臂姿态：
    - 先按 `world` 坐标系 X/Y/Z 轴的旋转（基于全局轴）。
    - 再按自身坐标系 X/Y/Z 轴的旋转（基于当前姿态的轴）。
  - 相机 `camera_link` 相同的一组参数。
- 输入单位：
  - 平移使用米（m）。
  - 角度使用度（deg），符合常见工业标定习惯，方便人工输入；计算时再转换为弧度。

### 1.3 输出
- `static_tf_config.yaml`（实为 JSON 兼容的 YAML）。内容需包含：
  - TF tree 列表：world→base_link、world→camera_link。
  - 每条变换的平移、原始旋转参数、变换顺序记录。
  - 计算得到的四元数：
    - `parent → child` 正解。
    - `child → parent` 逆解。
  - 逆向平移（child 坐标到 parent 坐标）。
  - 生成时间、单位等元数据，便于追溯。

## 2. 技术方案
### 2.1 配置文件结构
- 位置固定为 `static_tf_config.yaml`，存放在仓库根目录。
- 采用 JSON 语法输出（YAML 的子集），方便脚本用标准库读写。
- 结构示例：
  ```yaml
  metadata:
    generated_at: "2025-11-12T08:30:00Z"
    translation_units: "m"
    rotation_units: "deg"
  tf_tree:
    - parent: world
      child: base_link
    - parent: world
      child: camera_link
  transforms:
    world_to_base_link:
      parent_frame: world
      child_frame: base_link
      translation_m: {x: 0.0, y: 0.0, z: 0.0}
      rotation_deg:
        about_world_axes: {x: 0.0, y: 0.0, z: 0.0}
        about_body_axes:  {x: 0.0, y: 0.0, z: 0.0}
      transform_sequence:
        - {type: world_axis, axis: X, angle_deg: 0.0}
        - {type: body_axis,  axis: X, angle_deg: 0.0}
      quaternion:
        forward_parent_to_child: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
        inverse_child_to_parent: {x: -0.0, y: -0.0, z: -0.0, w: 1.0}
      inverse_translation_m: {x: -0.0, y: -0.0, z: -0.0}
    world_to_camera_link: { ... 类似结构 ... }
  ```

### 2.2 `static_tf_config_build.sh`
- Bash 入口脚本，交互式生成配置：
  1. 如已有 `static_tf_config.yaml`，自动读取现有值作为默认；用户可敲回车保持不变。
  2. 依次提示 base_link 的平移/旋转、camera_link 的平移/旋转。
     - 世界轴旋转顺序固定为 X → Y → Z（外在轴）。
     - 自身轴旋转顺序固定为 X → Y → Z（内在轴）。
  3. 收集完成后展示概览，确认后写入配置。
- 计算逻辑（在脚本内部调用 Python 标准库完成）：
  - 将度数转换为弧度，按顺序将世界轴旋转和自身轴旋转相乘，得到最终四元数。
  - 使用同一四元数计算逆变换和平移，保证正解/逆解一致。
  - 记录完整的变换顺序（包含 6 个步骤，即 3 个世界轴 + 3 个自身轴），即使角度为 0 也会注明。
- 输出文件覆盖写入，并带时间戳与生成工具信息，便于审计。

### 2.3 `static_tf_config_publish.sh`
- 根据 `static_tf_config.yaml` 中的四元数，调用 ROS 2 自带的 `static_transform_publisher`：
  - 每条 transform 启动一个 `ros2 run tf2_ros static_transform_publisher` 进程。
  - 这些进程本质上是静态节点，需要保持运行以在 `/tf_static` 上持续维持 latched 消息。
  - 脚本负责：
    - 解析配置并输出参数。
    - 启动所有发布进程并记录 PID。
    - 捕获 Ctrl+C，自动清理子进程。
- 运行前提：用户已 `source` 对应 ROS 2 环境，且系统可访问 `ros2` 命令。

### 2.4 操作流程
1. 运行 `./static_tf_config_build.sh`，根据提示输入或保持默认参数，确认后生成/更新配置。
2. 在已加载 ROS 2 环境的终端执行 `./static_tf_config_publish.sh`，脚本会启动两个静态 TF 发布者（world→base_link & world→camera_link）。
3. 启动 Foxglove 或 RViz2，加载对应模型，即可看到与真实世界对齐的坐标系关系。

### 2.5 暂不纳入范围
- 动态 TF、frame_prefix/namespace、多机器人模板等高级需求暂不处理。
- 不提供 GUI/CSV 导入，仅支持终端交互。
- 不实现自动验证或 TF tree 可视化工具，后续视需要补充。

上述内容即为当前已确定的需求背景与技术方案，后续若有变更请先更新本文档再推进实现。
