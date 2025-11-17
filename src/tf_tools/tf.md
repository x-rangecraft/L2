# TF Tools 静态 TF 配置与发布

## 最新改动（2025-11-12）
- `static_tf_config_build.sh` / `static_tf_config_publish.sh` 现在都会先定位脚本自身目录，再拼出 `static_tf_config.yaml` 与 `../description/urdf/yam.urdf` 的绝对路径，因此**可以在任何工作目录下直接调用** `./src/tf_tools/static_tf_config_build.sh` / `publish.sh`。
- 发布脚本新增守护（后台）模式：`--daemon/--start` 会为每条 TF 启动单独的 `static_transform_publisher` 常驻进程，并在脚本结束前打印当前 `static_tf_config.yaml` 内容；`--stop` 和 `--status` 用于管理后台进程。后台运行信息保存在 `src/tf_tools/.static_tf_publish/` 目录（PID 文件 + 每条 TF 的日志）。

该目录用于梳理并实现 "world→base_link / world→camera_link" 的静态 TF 配置流程，并将 URDF 中的连杆关系统一发布到 `/tf_static`，确保 Foxglove / RViz2 能看到与真实设备对齐的坐标系。

## 使用流程
> 💡 以下命令都可在仓库任意目录执行，脚本会自动定位自身路径。

1. 运行 `./src/tf_tools/static_tf_config_build.sh`，按提示输入或保留默认值（平移单位：米，角度单位：度）。
2. 脚本确认后会生成/更新 `src/tf_tools/static_tf_config.yaml`：
   - 包含 world→base_link、world→camera_link 的平移、世界/自身轴旋转、变换顺序以及正/逆四元数。
   - 固定从 `src/description/urdf/yam.urdf` 解析所有 joint，记录 parent/child、origin xyz/rpy（米+弧度）并写入 `urdf_chain.links`。
3. 在已 `source` ROS 2 环境的终端执行 `./src/tf_tools/static_tf_config_publish.sh [模式参数]`。
4. 打开 Foxglove 或 RViz2，加载对应模型即可查看 world→base_link / world→camera_link 以及 URDF 各连杆的静态 TF。

### static_tf_config_publish.sh 使用模式
| 模式 | 命令 | 行为 |
| --- | --- | --- |
| 前台阻塞（默认） | `./src/tf_tools/static_tf_config_publish.sh` | 实时打印每条 TF，脚本 `wait` 子进程，Ctrl+C 时触发清理并终止所有 `static_transform_publisher`。 |
| 后台常驻 | `./src/tf_tools/static_tf_config_publish.sh --daemon` 或 `--start` | 每条 TF 通过 `nohup` 常驻，PID 记录在 `.static_tf_publish/pids`，日志写入 `.static_tf_publish/logs/*.log`。脚本结束前会打印最新的 `static_tf_config.yaml` 供核对；后台进程会一直发布，直到手动 `--stop`。 |
| 停止后台 | `./src/tf_tools/static_tf_config_publish.sh --stop` | 读取 PID 文件并尝试 `kill` 每个后台 TF 进程，随后删除 PID 文件。 |
| 查看状态 | `./src/tf_tools/static_tf_config_publish.sh --status` | 列出 PID 文件中每条 TF 的运行状态（在/不在运行）。 |
| 帮助 | `./src/tf_tools/static_tf_config_publish.sh --help` | 输出上述说明。 |

**TODO**：当前实现沿用 ROS 2 官方 `static_transform_publisher`，每条变换必须由一个独立进程发布，因而后台模式会看到多个 `static_transform_publisher_*` 节点。后续如需减少节点数量，可自研单节点广播器，读取 `static_tf_config.yaml` 内所有变换并在同一进程内批量发布。

后台数据目录说明：
- `src/tf_tools/.static_tf_publish/pids`：记录 `PID TF名称`，供 `--stop/--status` 读取。
- `src/tf_tools/.static_tf_publish/logs/<tf_name>.log`：对应 TF 的 stdout/stderr，若 ROS 环境缺依赖（例如缺 `librcl_action.so`）可在这里排查。

### 常见注意事项
- 两个脚本都要求 `yam.urdf` 存在：路径写死为 `src/description/urdf/yam.urdf`，缺失会立即报错。
- 发布脚本运行前必须 `source /opt/ros/<distro>/setup.bash`，否则会提示找不到 `ros2` 命令；若出现 `librcl_action.so` 等共享库缺失，请按报错链接修复 ROS 环境再重启后台进程。
- 后台模式不会自动重启失败的 TF：如果日志里出现 ROS 加载失败，请 `--stop`、修好环境后再 `--daemon`。

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
- 运行脚本前请确保 `src/description/urdf/yam.urdf` 与现场设备版本同步。
- 发布脚本需在已 `source /opt/ros/<distro>/setup.bash` 的终端执行。
- 若需推广至其他模型或多机器人场景，请在此文档记录约定后再扩展脚本。
