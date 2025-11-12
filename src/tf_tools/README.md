# TF Tools 静态 TF 配置阶段

该目录暂用于梳理并实现 "world→base_link / world→camera_link" 的静态 TF 配置与发布流程，确保 Foxglove / RViz2 能看到正确姿态的设备模型。

## 当前交付
- `requirements.md`：背景、需求与技术方案说明（含单位定义、流程描述）。
- `static_tf_config_build.sh`：交互式生成 `static_tf_config.yaml` 的脚本。
- `static_tf_config_publish.sh`：基于配置启动 ROS 2 `static_transform_publisher` 的脚本。
- `static_tf_config.yaml`：最新一次生成的配置示例，可直接被发布脚本使用。

## 使用流程
1. 运行 `./static_tf_config_build.sh`，按提示输入或保留默认值（平移单位：米，角度单位：度）。
2. 确认后生成/更新 `static_tf_config.yaml`，文件中包含 TF tree、原始旋转参数、顺序记录以及正/逆四元数。
3. 在已 source ROS 2 环境的终端执行 `./static_tf_config_publish.sh`，脚本会为配置中的每条变换启动一个 `static_transform_publisher` 节点。
4. 打开 Foxglove 或 RViz2，可直接查看 world→base_link / world→camera_link 的静态 TF。

## 目录速览
```
tf_tools/
├── README.md
├── requirements.md
├── static_tf_config_build.sh
├── static_tf_config_publish.sh
└── static_tf_config.yaml
```

如需扩展其他文档或脚本，保持放在当前目录即可，先更新 `requirements.md` 同步新的约束与方案。
