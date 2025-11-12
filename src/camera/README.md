# camera 包

该目录用于承载相机节点（目前聚焦 Intel RealSense D435i）的 ROS 2 启动、参数与调试脚本：
- `realsense2_camera` 的 launch 文件与参数集中管理，避免与视觉算法混杂。
- 后续可在此扩展多相机实例、动态重配置或相机标定工具。
- 与 `vision/` 目录解耦：vision 只消费相机话题并进行 3D 推理。

创建包或 launch 文件时，请确保在此目录下完成，并在 `README.md` 的工作区结构中同步说明。
