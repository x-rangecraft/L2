# 统一 TF 发布器 - 改动总结

## 🎯 改动目标

**问题**：使用分离的 `static_transform_publisher` 导致 `ros2 node list` 出现大量节点

**解决方案**：所有 TF（静态+动态）统一由 `robot_tf_publisher` 节点管理

## ✨ 主要改动

### 1. `robot_tf_publisher.py` - 核心改动

**新增功能**：
- ✅ 添加 `StaticTransformBroadcaster` 支持
- ✅ 添加 `static_tf_config` 参数
- ✅ 添加 `_load_and_publish_static_tf()` 方法
- ✅ 从 `static_tf_config.yaml` 读取静态 TF 配置
- ✅ 统一发布静态和动态 TF

**改动前**：
```python
# 只发布动态 TF
self.tf_broadcaster = TransformBroadcaster(self)
```

**改动后**：
```python
# 同时发布静态和动态 TF
self.tf_broadcaster = TransformBroadcaster(self)
self.static_tf_broadcaster = StaticTransformBroadcaster(self)

# 加载静态TF配置
if static_config_path:
    self._load_and_publish_static_tf(Path(static_config_path))
```

### 2. `src/dynamic_tf_publish.sh` - 启动脚本更新

**新增功能**：
- ✅ 添加 `STATIC_TF_CONFIG` 路径
- ✅ 传递 `static_tf_config` 参数给节点
- ✅ 更新帮助信息

**改动前**：
```bash
python3 "$ROBOT_TF_PUBLISHER" \
  --ros-args \
  -p urdf_path:="$URDF_PATH" \
  -p publish_rate:=50.0
```

**改动后**：
```bash
python3 "$ROBOT_TF_PUBLISHER" \
  --ros-args \
  -p urdf_path:="$URDF_PATH" \
  -p static_tf_config:="$STATIC_TF_CONFIG" \
  -p publish_rate:=50.0
```

### 3. `tf_publisher.sh` - 简化为包装器

**改动**：
- ✅ 大幅简化（从 369行 → 75行）
- ✅ 现在只是 `src/dynamic_tf_publish.sh` 的包装器
- ✅ 去掉了复杂的静态/动态分离逻辑
- ✅ 统一接口，更易使用

**优势**：
- 代码更简洁
- 维护更容易
- 用户体验一致

## 📊 效果对比

### 改动前：

```bash
$ ros2 node list
/robot_driver
/robot_desc_node
/static_transform_publisher_xxx1    ← 8个独立节点！
/static_transform_publisher_xxx2
/static_transform_publisher_xxx3
/static_transform_publisher_xxx4
/static_transform_publisher_xxx5
/static_transform_publisher_xxx6
/static_transform_publisher_xxx7
/static_transform_publisher_xxx8
/robot_tf_publisher
...
```

### 改动后：

```bash
$ ros2 node list
/robot_driver
/robot_desc_node
/robot_tf_publisher                 ← 只有1个节点！
...
```

**节点数量减少**：从 9个 → 1个

## 🚀 使用方式

### 基本使用（完全一致）

```bash
# 后台启动
./src/tf_tools/tf_publisher.sh --daemon

# 查看状态
./src/tf_tools/tf_publisher.sh --status

# 停止
./src/tf_tools/tf_publisher.sh --stop
```

### 发布的 TF

**静态 TF**（从 `static_tf_config.yaml`）：
- world → base_link
- world → camera_link
- 其他自定义静态变换

**动态 TF**（从 URDF + joint_states）：
- base_link → link_1 → link_2 → link_3 → link_4 → link_5 → link_6

**所有 TF 都由 `robot_tf_publisher` 节点统一发布！**

## 🔧 技术实现

### TF 发布流程

```
┌─────────────────────────┐
│ robot_tf_publisher 节点  │
├─────────────────────────┤
│                         │
│ 1. 启动时加载           │
│    static_tf_config     │
│    ↓                    │
│    StaticTFBroadcaster  │
│    发布静态 TF (一次)    │
│                         │
│ 2. 订阅 joint_states    │
│    ↓                    │
│    计算正向运动学        │
│    ↓                    │
│    TFBroadcaster        │
│    发布动态 TF (持续)    │
│                         │
└─────────────────────────┘
```

### 静态 TF 加载

```python
def _load_and_publish_static_tf(self, config_path: Path):
    # 1. 读取 JSON 配置
    with open(config_path, 'r') as f:
        config = json.load(f)

    # 2. 构造 TransformStamped 消息
    for name, tf_data in config.get('transforms', {}).items():
        transform = TransformStamped()
        transform.header.frame_id = tf_data['parent_frame']
        transform.child_frame_id = tf_data['child_frame']
        # ... 设置平移和旋转
        transforms.append(transform)

    # 3. 一次性发布所有静态 TF
    self.static_tf_broadcaster.sendTransform(transforms)
```

## 📝 配置要求

### `static_tf_config.yaml` 格式

必须包含 `transforms` 字段，格式如下：

```json
{
  "transforms": {
    "world_to_base_link": {
      "parent_frame": "world",
      "child_frame": "base_link",
      "translation_m": {"x": 0.0, "y": 0.0, "z": 0.0},
      "quaternion": {
        "forward_parent_to_child": {
          "x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0
        }
      }
    }
  }
}
```

## 🔍 迁移步骤

### 从旧系统迁移

1. **停止旧的 TF 发布器**：
   ```bash
   pkill -f static_transform_publisher
   ```

2. **启动新的统一发布器**：
   ```bash
   ./src/tf_tools/tf_publisher.sh --daemon
   ```

3. **验证**：
   ```bash
   # 检查节点列表（应该只有 robot_tf_publisher）
   ros2 node list | grep tf

   # 检查 TF 发布（应该包含静态和动态）
   ros2 topic hz /tf
   ros2 topic hz /tf_static
   ```

## ⚠️  注意事项

1. **配置文件必须存在**
   - `static_tf_config.yaml` 必须在 `tf_tools/` 目录
   - 如果文件不存在，静态 TF 将不会发布（但动态 TF 正常）

2. **向后兼容**
   - 旧的独立静态 TF 脚本仍然保留
   - 如需使用旧模式，可以手动运行静态 TF 脚本

3. **日志位置**
   - 统一日志：`L2/log/tf_tools/robot_tf_publisher.log`
   - 包含静态和动态 TF 的日志

## 🎉 优势总结

✅ **节点数量减少**：从多个 → 1个
✅ **管理更简单**：统一启动/停止
✅ **性能更好**：减少进程开销
✅ **日志统一**：便于调试
✅ **代码更简洁**：易于维护

---

**更新时间**：2025-11-17
**版本**：3.0.0（统一发布器）
**状态**：✅ 实现完成，待测试
