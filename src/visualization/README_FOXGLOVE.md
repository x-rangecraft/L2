# Foxglove Bridge 服务文档

## 概述

Foxglove Bridge 是一个 ROS 2 到 WebSocket 的桥接服务，允许通过 Foxglove Studio 可视化工具远程连接和监控 ROS 2 系统。本文档描述了自动化启动脚本的使用方法和内部逻辑。

## 文件说明

- `start_foxglove_bridge.sh` - Foxglove Bridge 后台启动脚本
- 日志目录：`/home/jetson/L2/log/foxglove_bridge.log`

## 快速开始

### 启动服务

```bash
cd /home/jetson/L2
./src/visualization/start_foxglove_bridge.sh
```

脚本会显示 WebSocket 连接地址，例如：
```
========================================
Foxglove WebSocket Connection Info:
----------------------------------------
Server listening on all interfaces (0.0.0.0:9090)

Connect using one of these URLs:
  ws://192.168.1.100:9090
  ws://localhost:9090
========================================

✓ Foxglove Bridge started successfully (PID: 12345)
```

### 停止服务

```bash
# 方法1：使用进程名（推荐）
pkill -f 'foxglove_bridge.*port:=9090'

# 方法2：使用 PID（从启动输出中获取）
kill <PID>
```

### 查看日志

```bash
# 实时查看日志
tail -f /home/jetson/L2/log/foxglove_bridge.log

# 查看最近的日志
tail -n 100 /home/jetson/L2/log/foxglove_bridge.log
```

### 检查服务状态

```bash
# 查看进程是否运行
ps aux | grep foxglove_bridge

# 检查端口占用
netstat -tuln | grep 9090
# 或
ss -tuln | grep 9090
```

## 配置选项

脚本支持通过环境变量自定义配置：

### 监听地址

```bash
# 只监听本地（默认 0.0.0.0 监听所有接口）
FOXGLOVE_ADDRESS=127.0.0.1 ./src/visualization/start_foxglove_bridge.sh
```

### 端口号

```bash
# 使用自定义端口（默认 9090）
FOXGLOVE_PORT=8080 ./src/visualization/start_foxglove_bridge.sh
```

### ROS 域 ID

```bash
# 设置 ROS_DOMAIN_ID（默认 0）
ROS_DOMAIN_ID=42 ./src/visualization/start_foxglove_bridge.sh
```

### 组合使用

```bash
FOXGLOVE_ADDRESS=0.0.0.0 FOXGLOVE_PORT=8080 ROS_DOMAIN_ID=1 ./src/visualization/start_foxglove_bridge.sh
```

## 连接 Foxglove Studio

1. **下载 Foxglove Studio**
   - 访问 https://foxglove.dev/download
   - 选择适合你操作系统的版本

2. **连接到服务**
   - 打开 Foxglove Studio
   - 选择 "Open connection"
   - 选择 "Rosbridge (ROS 1 & 2)"
   - 输入 WebSocket URL，例如：`ws://192.168.1.100:9090`
   - 点击 "Open"

3. **开始可视化**
   - 连接成功后可以看到所有可用的 ROS 话题
   - 添加面板（3D、图表、日志等）来可视化数据

## 脚本逻辑说明

### 启动流程

```
1. 环境检查
   ├─ 检查 ROS 2 Humble 是否安装
   └─ 加载 ROS 环境配置

2. 工作空间设置
   ├─ 尝试加载工作空间 setup.bash
   └─ 如果不存在，使用系统级 foxglove_bridge

3. 端口冲突处理
   ├─ 检查目标端口是否被占用
   ├─ 如果是旧的 foxglove_bridge 进程
   │  ├─ 显示进程信息
   │  ├─ 尝试优雅停止 (SIGTERM)
   │  └─ 如果失败，强制停止 (SIGKILL)
   └─ 如果是其他进程，提示错误并退出

4. 启动服务
   ├─ 使用 nohup 在后台启动
   ├─ 将日志输出到文件
   ├─ 获取进程 PID
   └─ 验证启动是否成功

5. 显示连接信息
   ├─ 列出所有可用的网络接口 IP
   ├─ 生成 WebSocket 连接 URL
   ├─ 显示日志文件路径
   └─ 提供停止服务的命令
```

### 关键特性

#### 1. 自动端口清理

脚本会自动检测并清理端口占用：

```bash
# 检测端口占用
if netstat -tuln 2>/dev/null | grep -q ":${PORT} " || ss -tuln 2>/dev/null | grep -q ":${PORT} "; then
  # 查找并停止旧的 foxglove_bridge 进程
  EXISTING_PIDS=$(pgrep -f "foxglove_bridge.*port:=${PORT}" || true)
  # ... 清理逻辑
fi
```

#### 2. 后台持久运行

使用 `nohup` 确保服务在终端关闭后继续运行：

```bash
nohup ros2 run foxglove_bridge foxglove_bridge \
  --ros-args -p address:=${ADDRESS} -p port:=${PORT} \
  >> "${LOG_FILE}" 2>&1 &
```

#### 3. 启动验证

启动后等待 2 秒并验证进程是否正常运行：

```bash
sleep 2
if ps -p ${BRIDGE_PID} > /dev/null 2>&1; then
  echo "✓ Foxglove Bridge started successfully"
else
  echo "✗ Failed to start Foxglove Bridge"
  exit 1
fi
```

#### 4. 智能 IP 地址显示

当监听 `0.0.0.0` 时，自动列出所有可用的网络接口：

```bash
LOCAL_IPS=$(hostname -I 2>/dev/null | tr ' ' '\n' | grep -v '^$' || echo "")
```

## 故障排查

### 问题：端口被占用

**症状：**
```
Port 9090 is already in use by another process
```

**解决方法：**
```bash
# 查找占用端口的进程
lsof -i :9090
# 或
ss -tulpn | grep 9090

# 停止进程
kill <PID>

# 或使用不同端口
FOXGLOVE_PORT=8080 ./src/visualization/start_foxglove_bridge.sh
```

### 问题：无法连接到服务

**检查清单：**

1. 确认服务正在运行
   ```bash
   ps aux | grep foxglove_bridge
   ```

2. 检查防火墙设置
   ```bash
   sudo ufw status
   # 如果需要，开放端口
   sudo ufw allow 9090/tcp
   ```

3. 验证网络连接
   ```bash
   # 从本地测试
   curl -i -N -H "Connection: Upgrade" -H "Upgrade: websocket" \
     http://localhost:9090
   ```

4. 查看日志寻找错误信息
   ```bash
   tail -f /home/jetson/L2/log/foxglove_bridge.log
   ```

### 问题：ROS 话题不显示

**可能原因：**

1. **ROS_DOMAIN_ID 不匹配**
   ```bash
   # 检查当前域 ID
   echo $ROS_DOMAIN_ID

   # 使用相同的域 ID 启动
   ROS_DOMAIN_ID=<your_id> ./src/visualization/start_foxglove_bridge.sh
   ```

2. **话题没有发布者**
   ```bash
   # 列出所有话题
   ros2 topic list

   # 检查话题是否有数据
   ros2 topic echo /your_topic
   ```

3. **工作空间未构建**
   - 如果使用自定义消息类型，需要先构建工作空间
   ```bash
   cd /home/jetson/L2
   colcon build
   source install/setup.bash
   ```

### 问题：服务意外停止

**诊断步骤：**

1. 查看日志文件
   ```bash
   tail -n 200 /home/jetson/L2/log/foxglove_bridge.log
   ```

2. 检查系统资源
   ```bash
   # 查看内存使用
   free -h

   # 查看系统日志
   journalctl -xe
   ```

3. 手动启动进行调试
   ```bash
   # 在前台运行以查看实时输出
   cd /home/jetson/L2
   source /opt/ros/humble/setup.bash
   ros2 run foxglove_bridge foxglove_bridge \
     --ros-args -p address:=0.0.0.0 -p port:=9090
   ```

## 依赖项

### 系统依赖

- **ROS 2 Humble**：`/opt/ros/humble/setup.bash`
- **foxglove_bridge**：ROS 2 包（系统或工作空间安装）
- **网络工具**：`netstat` 或 `ss`（用于端口检测）

### 验证依赖

```bash
# 检查 ROS 2
test -f /opt/ros/humble/setup.bash && echo "✓ ROS 2 Humble found" || echo "✗ ROS 2 Humble missing"

# 检查 foxglove_bridge
source /opt/ros/humble/setup.bash
ros2 pkg list | grep foxglove_bridge && echo "✓ foxglove_bridge found" || echo "✗ foxglove_bridge missing"

# 检查网络工具
command -v netstat >/dev/null 2>&1 && echo "✓ netstat found" || echo "✗ netstat missing"
command -v ss >/dev/null 2>&1 && echo "✓ ss found" || echo "✗ ss missing"
```

### 安装缺失的依赖

```bash
# 安装 foxglove_bridge（如果缺失）
sudo apt update
sudo apt install ros-humble-foxglove-bridge

# 安装网络工具（如果缺失）
sudo apt install net-tools
```

## 高级用法

### 自动启动（开机启动）

创建 systemd 服务文件：

```bash
sudo nano /etc/systemd/system/foxglove-bridge.service
```

内容：
```ini
[Unit]
Description=Foxglove Bridge Service
After=network.target

[Service]
Type=forking
User=jetson
WorkingDirectory=/home/jetson/L2
ExecStart=/home/jetson/L2/src/visualization/start_foxglove_bridge.sh
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

启用服务：
```bash
sudo systemctl daemon-reload
sudo systemctl enable foxglove-bridge.service
sudo systemctl start foxglove-bridge.service
```

### 监控脚本

创建一个简单的监控脚本来确保服务始终运行：

```bash
#!/bin/bash
# monitor_foxglove.sh

while true; do
  if ! pgrep -f "foxglove_bridge.*port:=9090" > /dev/null; then
    echo "$(date): Foxglove Bridge not running, restarting..."
    /home/jetson/L2/src/visualization/start_foxglove_bridge.sh
  fi
  sleep 30
done
```

### 多实例运行

如果需要同时运行多个 Foxglove Bridge 实例（不同端口）：

```bash
# 实例 1：默认端口 9090
./src/visualization/start_foxglove_bridge.sh

# 实例 2：端口 9091
FOXGLOVE_PORT=9091 ./src/visualization/start_foxglove_bridge.sh

# 实例 3：端口 9092，不同域
ROS_DOMAIN_ID=1 FOXGLOVE_PORT=9092 ./src/visualization/start_foxglove_bridge.sh
```

## 性能考虑

### 网络带宽

- 图像话题会消耗大量带宽
- 考虑使用压缩图像话题（`/compressed` 或 `/theora`）
- 在 Foxglove Studio 中降低订阅频率

### 系统资源

- 每个连接的客户端会增加 CPU 和内存使用
- 监控系统资源：
  ```bash
  # 查看 foxglove_bridge 资源使用
  ps aux | grep foxglove_bridge
  top -p $(pgrep -f foxglove_bridge)
  ```

## 相关资源

- **Foxglove Studio 官网**：https://foxglove.dev
- **Foxglove Bridge GitHub**：https://github.com/foxglove/ros-foxglove-bridge
- **ROS 2 Humble 文档**：https://docs.ros.org/en/humble/
- **WebSocket 协议**：https://datatracker.ietf.org/doc/html/rfc6455

## 维护日志

| 日期 | 版本 | 变更说明 |
|------|------|----------|
| 2025-11-11 | 1.0 | 初始版本，支持后台启动、自动端口清理、日志记录 |

## 贡献

如有问题或改进建议，请联系项目维护者或提交 issue。
