# Camera 启动目标

为 Intel RealSense（当前聚焦 D435i）提供单一入口脚本，读取统一配置并启动 `realsense2_camera` 节点，对整个系统持续输出一致的图像、点云与 TF。所有后续拓展（多相机、动态重配置、标定等）都应围绕该目标展开。

## start_camera.sh 使用方法
- `./start_camera.sh --start [配置文件]`：默认读取 `config/cameras/realsense2_d435i.yaml`，若检测到 `/camera/realsense2_camera` 和 `/camera/color/image_raw` 已存在则跳过重启；否则后台以 `nohup` 拉起节点，PID 写入 `.realsense_camera.pid`，日志写入 `realsense2_camera.log`。脚本内部通过 `~/` remap 保证话题统一为单层 `/camera/...`，不会出现 `camera/camera/...`。
- `./start_camera.sh --stop`：读取 PID 文件并尝试优雅停止，若进程仍在则 fallback 到 `pkill -f realsense2_camera_node`，确保所有 RealSense 相关节点关闭。
- 运行前需先 `source` 对应 ROS 2 环境，让 `ros2` CLI 可用；若传入自定义配置路径需确保文件存在且帧 ID 不带 `camera_` 前缀，避免生成 `camera_camera_link`。
