# Web 前端点击交互规划

## 现状梳理

- **Stage2 Segmentation**：原实现中 `l1_stage2_segmentation/visualization/web_gui/segmentation_gui.py` 把 ROS 订阅、点击发布与 Flask/Socket.IO/HTML 全部糅合在一起，如今已变成 thin wrapper，真正的逻辑迁移到 `web_interactive_gui` 包。
- **Stage3 Grasping**：目前只在 README 中约定复用 `/click_point`，真正的 3D 点云/抓取节点还未消费点击事件。因此 Stage3 若想直接接入 Web 点击 UI，需要等 Stage2 节点运行。
- **L2 src/web 目录**：尚无代码，正好可以承载一个面向多 Stage 的通用 Web 前端与 ROS 桥接实现。

## 剥离目标

1. **职责分离**：将“Web 图像流 + 点击采集 + `/click_point` 发布”抽成独立包，Stage2/Stage3 通过参数即可复用，后续如需更多 Web 控件也可在此扩展。
2. **模板解耦**：把 HTML/CSS/JS 放到独立模板和静态资源中，减少 Python 文件内联字符串，方便迭代 UI。
3. **数据源可配置**：保留现有 QoS 设定，但通过参数/YAML 指定输入图像话题、覆盖层话题、输出点击话题、Frame ID、HTTP 端口等，动态适配 Stage2 或 Stage3。
4. **线程安全复用**：沿用 Queue + Timer 的跨线程发布方式，确保 Flask 线程与 ROS 线程隔离；同时提供订阅者数量检测与日志，便于调试。
5. **渐进迁移**：Stage2 继续提供一个薄包装（import 新模块 + 传参），保证原有 launch 文件 `enable_web_gui:=true` 仍可工作；Stage3 在需要点击 UI 时直接依赖新包，无需再引用 Stage2 代码。

## 拆分思路

| 模块 | 说明 |
| --- | --- |
| `web_interactive_gui/node.py` | ROS Node + Flask/Socket.IO 入口，负责参数声明、订阅/发布、线程管理。|
| `web_interactive_gui/templates/interactive_viewer.html` | 浏览器端 UI，包含点击坐标换算、状态展示、FPS 统计。|
| `web_interactive_gui/static/socket.io.min.js` | Socket.IO 本地备份脚本，CDN 不可用时兜底。|
| `webInteractiveGUI.sh` | Shell 脚本，统一 start/stop 节点、落盘日志与 PID。|

## 接入方式

1. **Stage2**：在 `segmentation_gui.py` 中改为导入 `web_gui_server.WebInteractiveClickApp`（示例），仅传入话题名称与 GUI 参数；launch 不变。
2. **Stage3**：新增 launch（例如 `stage3_web_debug.launch.py`），引用同一 Node，但订阅 Stage3 点云/overlay 或禁用 overlay，仅用于点击收集。
3. **未来 L2 Web**：若要扩展到多页面，可让 `src/web` 提供 FastAPI/Flask blueprint，或嵌入 React/Vue 静态资源，同时保持 `/click` REST 接口兼容。

## 当前进展

- 在 `src/web/web_interactive_gui` 内完成 ROS 2 ament_python 包：`package.xml`、`setup.py`、模板与静态资源齐备，`web_interactive_gui` 节点负责订阅图像/overlay、推流到 Socket.IO，并将点击转成 `/click_point` 的 `PointStamped`。
- 前端模板 `templates/interactive_viewer.html` 与 `static/socket.io.min.js` 已拆分；UI 复用 Stage2 交互逻辑并加入 Socket.IO CDN + 本地备份。
- `src/web/webInteractiveGUI.sh` 可一键 `start|stop` 节点（后台运行、日志写入 `log/web_interactive_gui.log`、PID 保存）。
- Stage2 `l1_stage2_segmentation` 里的 `segmentation_gui.py` 已变成 thin wrapper，如未构建 `web_interactive_gui` 会直接提示；`package.xml` 增加 `exec_depend`，Launch 配置保持不变。

## 后续步骤

1. 为 Stage3 添加示例 launch/README 章节：演示如何只运行 `web_interactive_gui` 与 Stage3 节点，实现点击→点云链路调试。
2. 编写基础自动化测试（Python/pytest 或 `launch_testing`）：mock 图像话题 + HTTP `/click` 请求，校验 `/click_point` 发布。
3. 扩充文档（本文件或独立 README），列出节点参数、端口/话题 remap 示例，以及 `webInteractiveGUI.sh` 日志与常见问题排查。

完成上述收尾，即可把 L2 Web 前端完全交付到多 Stage 流水线。
