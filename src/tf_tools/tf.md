# TF Tools 静态 TF 配置与发布

该目录用于梳理并实现 "world→base_link / world→camera_link" 的静态 TF 配置流程，并将 URDF 中的连杆关系统一发布到 `/tf_static`，确保 Foxglove / RViz2 能看到与真实设备对齐的坐标系。

## 使用流程
1. 运行 `./static_tf_config_build.sh`，按提示输入或保留默认值（平移单位：米，角度单位：度）。
2. 脚本确认后会生成/更新 `static_tf_config.yaml`：
   - 包含 world→base_link、world→camera_link 的平移、世界/自身轴旋转、变换顺序以及正/逆四元数。
   - 固定从 `../description/urdf/yam.urdf` 解析所有 joint，记录 parent/child、origin xyz/rpy（米+弧度）并写入 `urdf_chain.links`。
3. 在已 source ROS 2 环境的终端执行 `./static_tf_config_publish.sh`：
   - 运行前如检测不到 `ros2` 命令会立即报错。
   - 对配置中的 world→X 变换及 URDF 中所有关节逐条调用 `ros2 run tf2_ros static_transform_publisher`。
4. 打开 Foxglove 或 RViz2，加载对应模型即可查看 world→base_link / world→camera_link 以及 URDF 各连杆的静态 TF。

目录速览：
```
tf_tools/
├── static_tf_config_build.sh
├── static_tf_config_publish.sh
├── static_tf_config.yaml
└── tf.md  # 本说明
```

---

## 需求与背景
### 目标
- 用统一配置管理 world→base_link、world→camera_link 等静态关系。
- 自动生成 `tf_static` 所需配置，使虚拟场景与真实设备对齐。
- 发布阶段复用官方 `static_transform_publisher`，无需自研节点。

### 输入
- 交互式参数：
  - base_link/camera_link 在 world 下的平移 (x, y, z)。
  - 世界轴旋转顺序 X→Y→Z（外在轴）。
  - 自身轴旋转顺序 X→Y→Z（内在轴）。
- 单位：平移米（m），角度度（deg），内部运算自动转弧度。

### 输出
- `static_tf_config.yaml`（JSON 兼容）：
  - TF tree 列表、原始平移/旋转、变换顺序。
  - 计算得到的正/逆四元数及逆向平移。
  - `metadata` 记录生成时间、单位、旋转顺序、URDF 源路径。
  - `urdf_chain.links` 保存 URDF 中每个 joint 的 parent/child、origin xyz/rpy。

---

## 技术方案
### static_tf_config_build.sh
1. 读取旧配置作为默认值；逐项提示 base_link 与 camera_link 的平移/旋转。
2. 交互完成后调用 Python：
   - 将世界轴与自身轴旋转依序累乘，输出正/逆四元数与逆向平移。
   - 记录完整变换序列，即使角度为 0 也会写入。
3. 固定使用 `../description/urdf/yam.urdf`：
   - 文件缺失即报错终止，避免误用其他模型。
   - 解析所有 joint，直接保留 origin 的米/弧度数值写入 `urdf_chain`。
4. 最终写出 `static_tf_config.yaml`，含 metadata 与 TF tree。

### static_tf_config_publish.sh
- 根据配置内容调用 ROS 2 官方 `static_transform_publisher`：
  - world→base_link、world→camera_link。
  - `urdf_chain.links` 中的全部 parent→child （数量由 URDF 决定，不做强制限制）。
- 启动前强制检查 `ros2` 命令；未加载 ROS 2 环境会直接退出。
- 记录子进程 PID，Ctrl+C 时统一清理。
- 本阶段确认无需常驻自研节点。

---

## 操作提示
- 运行脚本前请确保 `../description/urdf/yam.urdf` 存在并与现场设备版本同步。
- 发布脚本需在已 `source /opt/ros/<distro>/setup.bash` 的终端执行。
- 若需推广至其他模型或多机器人场景，请先更新本 `tf.md`，再扩展脚本。
