---
description: 为当前任务加载 L1/L2/L3 上下文到会话
argument-hint: <task-id>
---

# 上下文加载命令

为指定任务加载相关的 Playbook（L1）、Blueprint（L2）与 Journal（L3）文档。

**任务 ID**：$ARGUMENTS

## 操作

1. **识别任务上下文**
   - 若提供 task ID：从 `/DEV_TRACKING/ACTIVE/$1/` 加载
   - 若未提供：查找活跃任务并列出可选项

2. **加载 L3（Journal）**
   - 读取 `01_PLAN.md`
   - 读取 `02_CONTEXT_LOG.md`
   - 读取 `03_CHECKLIST.md`
   - 展示当前状态与下一步

3. **加载相关 L2（Blueprint）**
   - 从计划中提取受影响模块
   - 读取每个模块的 `ARCHITECTURE.md`
   - 读取每个模块的 `API_INTERFACE.md`
   - 展示 API 契约与设计约束

4. **加载相关 L1（Playbook）**
   - 根据计划识别所需技能
   - 读取对应的 Playbook 文档
   - 展示适用的标准与模式

5. **上下文摘要**
   输出简明摘要：
   - 当前任务状态
   - 最近完成的清单项
   - 接下来 3 个行动
   - 来自 L1/L2 的关键约束
   - 任何阻塞项或待定决策

## 示例输出

```
📋 已加载 Task: YOLO_INTEGRATION_001 的上下文

## L3（Journal）状态
- 计划：实现 YOLOv8 + TensorRT 检测服务
- 进度：60% 完成（10 项清单完成 6 项）
- 最近完成：YoloTRTEngine 类实现
- 下一步：实现 DetectObject service callback

## L2（Blueprint）约束
- 模块：M1_Perception
- API 契约：DetectObject service（string -> bool, PoseStamped）
- Frame ID：camera_color_optical_frame
- 依赖：/camera/color/image_raw Topic

## L1（Playbook）要求
- ML_MODEL_INTEGRATION.md：TensorRT 优化模式
- ROS2_BEST_PRACTICES.md：服务回调、执行器管理
- ERROR_HANDLING_LOGGING.md：超时与错误模式

## 准备继续
下一步：实现包含完整错误处理的 service callback
```

现在为指定任务加载上下文。
