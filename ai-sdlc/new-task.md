---
description: 以正确的 L3 日志结构初始化新任务
argument-hint: <task-name>
---

# 新任务初始化

创建一个新任务，带完整的 Journal（L3）结构，并引导整个 AI-Native SDLC 流程。

**任务名称**：$ARGUMENTS

## 工作流

### 1. 创建任务目录

生成唯一任务 ID：`TASK_[NAME]_[TIMESTAMP]`

创建目录结构：
```
/DEV_TRACKING/ACTIVE/[TASK_ID]/
├── 01_PLAN.md（由 Agent-P 创建）
├── 02_CONTEXT_LOG.md（用模板初始化）
└── 03_CHECKLIST.md（由 Agent-P 创建）
```

### 2. 初始化上下文日志

用以下模板创建 `02_CONTEXT_LOG.md`：

```markdown
# 上下文日志：[任务名称]

**任务 ID**：[TASK_ID]
**创建时间**：[日期/时间]
**状态**：规划中

---

## 初始需求

[待填写]

## 验收标准

[待填写]

---

## 开发日志

[后续迭代会将更新写在此处]

---

## 下一步

1. 完成需求分析
2. 与 Agent-P 生成实现计划
3. 进行人工计划评审（HPR）
```

### 3. 引导 SDLC

**阶段 1：需求分析**
- 引导用户定义验收标准
- 识别受影响模块
- 确定需要的 L1/L2 上下文

**阶段 2：规划**
- 调用 Agent-P 生成计划：`/agent-p [task description]`
- 安排人工计划评审（HPR）

**阶段 3：准备实施**
- 确认计划获批
- 加载上下文：`/load-context [TASK_ID]`
- 由 Agent-I 开始执行

## 输出

初始化完成后展示：

```
✅ 已初始化任务：[TASK_ID]

**目录**：/DEV_TRACKING/ACTIVE/[TASK_ID]/

**下一步**：
1. 定义验收标准（必要时使用 AskUserQuestion）
2. 生成计划：/agent-p [task description]
3. 进行 HPR（人工计划评审）
4. 开始实施：/load-context [TASK_ID]

**已准备好继续需求分析**
```

现在初始化该任务。
