#!/usr/bin/env bash
# Robot TF 启动脚本
# 统一管理 TF 发布和转换服务
#
# 用法：
#   ./start_robot_tf.sh --start       # 后台启动节点（默认）
#   ./start_robot_tf.sh --stop        # 停止节点
#   ./start_robot_tf.sh --status      # 查看状态
#   ./start_robot_tf.sh --foreground  # 前台运行（调试）

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
URDF_PATH="${SCRIPT_DIR}/../description/urdf/yam.urdf"
STATIC_TF_CONFIG="${SCRIPT_DIR}/static_tf_config.yaml"
LOG_DIR="${SCRIPT_DIR}/../../log/tf_tools"
PID_FILE="${LOG_DIR}/robot_tf.pid"
LOG_FILE="${LOG_DIR}/robot_tf.log"
ROBOT_TF_SCRIPT="${SCRIPT_DIR}/src/robot_tf.py"

usage() {
  cat <<'EOF'
用法：start_robot_tf.sh [选项]

选项：
  --start       后台启动 robot_tf 节点（默认）
  --stop        停止后台运行的节点
  --status      查看节点运行状态
  --foreground  前台运行，按 Ctrl+C 停止
  --help        显示此帮助信息

功能说明：
  robot_tf 节点提供以下功能：
  
  1. TF 发布
     - 静态 TF：发布 world→base_link, world→camera_link 等固定变换
     - 动态 TF：根据 joint_states 发布机器人关节变换
  
  2. TF 转换服务
     - /tf_tools/transform_points：坐标转换服务
     - 支持将点从任意坐标系转换到目标坐标系

示例：
  ./start_robot_tf.sh                 # 后台启动（默认）
  ./start_robot_tf.sh --foreground    # 前台运行（调试）
  ./start_robot_tf.sh --stop          # 停止
  ./start_robot_tf.sh --status        # 查看状态

日志位置：
  ${LOG_DIR}/robot_tf.log
EOF
}

ensure_log_dir() {
  mkdir -p "$LOG_DIR"
}

check_ros_env() {
  if ! command -v ros2 >/dev/null 2>&1; then
    echo "[ERROR] 未检测到 ros2 命令，请先 source ROS 2 环境" >&2
    exit 1
  fi
}

source_workspace() {
  # Source ROS 2 工作空间以加载 tf_tools 包
  local ws_setup="${SCRIPT_DIR}/../../install/setup.bash"
  if [[ -f "$ws_setup" ]]; then
    # 临时禁用 set -u，因为 setup.bash 中可能有未设置的变量
    set +u
    source "$ws_setup"
    set -u
  else
    echo "[WARN] 未找到工作空间 setup.bash: $ws_setup" >&2
    echo "[WARN] 请确保已编译 tf_tools 包: colcon build --packages-select tf_tools" >&2
  fi
}

check_urdf() {
  if [[ ! -f "$URDF_PATH" ]]; then
    echo "[ERROR] URDF 文件不存在：$URDF_PATH" >&2
    exit 1
  fi
}

check_script() {
  if [[ ! -f "$ROBOT_TF_SCRIPT" ]]; then
    echo "[ERROR] robot_tf.py 不存在：$ROBOT_TF_SCRIPT" >&2
    exit 1
  fi
}

start_foreground() {
  echo "[INFO] 前台启动 robot_tf..."
  echo "[INFO] URDF: $URDF_PATH"
  echo "[INFO] 静态TF配置: $STATIC_TF_CONFIG"
  echo "[INFO] 按 Ctrl+C 停止"
  echo ""
  python3 "$ROBOT_TF_SCRIPT" \
    --ros-args \
    -p urdf_path:="$URDF_PATH" \
    -p static_tf_config:="$STATIC_TF_CONFIG" \
    -p publish_rate:=50.0 \
    -p base_frame:=base_link
}

start_daemon() {
  ensure_log_dir

  # Check if already running
  if [[ -f "$PID_FILE" ]]; then
    pid=$(cat "$PID_FILE")
    if kill -0 "$pid" 2>/dev/null; then
      echo "[ERROR] robot_tf 已在运行 (PID: $pid)" >&2
      echo "[INFO] 使用 --stop 停止现有进程，或 --status 查看状态" >&2
      exit 1
    fi
  fi

  echo "[INFO] 后台启动 robot_tf..."
  echo "[INFO] URDF: $URDF_PATH"
  echo "[INFO] 静态TF配置: $STATIC_TF_CONFIG"
  echo "[INFO] 日志: $LOG_FILE"

  nohup python3 "$ROBOT_TF_SCRIPT" \
    --ros-args \
    -p urdf_path:="$URDF_PATH" \
    -p static_tf_config:="$STATIC_TF_CONFIG" \
    -p publish_rate:=50.0 \
    -p base_frame:=base_link \
    > "$LOG_FILE" 2>&1 &

  pid=$!
  echo "$pid" > "$PID_FILE"

  # Wait a moment and check if process is still running
  sleep 1
  if kill -0 "$pid" 2>/dev/null; then
    echo "[INFO] robot_tf 已启动 (PID: $pid)"
    echo "[INFO] 查看日志: tail -f $LOG_FILE"
    echo "[INFO] 停止服务: $0 --stop"
  else
    echo "[ERROR] robot_tf 启动失败，请查看日志: $LOG_FILE" >&2
    rm -f "$PID_FILE"
    exit 1
  fi
}

stop_daemon() {
  if [[ ! -f "$PID_FILE" ]]; then
    echo "[INFO] robot_tf 未运行"
    return 0
  fi

  pid=$(cat "$PID_FILE")
  if kill -0 "$pid" 2>/dev/null; then
    echo "[INFO] 停止 robot_tf (PID: $pid)..."
    kill "$pid" 2>/dev/null || true

    # Wait for process to stop
    for i in {1..10}; do
      if ! kill -0 "$pid" 2>/dev/null; then
        break
      fi
      sleep 0.5
    done

    # Force kill if still running
    if kill -0 "$pid" 2>/dev/null; then
      echo "[WARN] 进程未响应，强制终止..."
      kill -9 "$pid" 2>/dev/null || true
    fi

    echo "[INFO] robot_tf 已停止"
  else
    echo "[INFO] robot_tf 未运行 (PID 文件存在但进程不在)"
  fi

  rm -f "$PID_FILE"
}

show_status() {
  if [[ ! -f "$PID_FILE" ]]; then
    echo "[INFO] robot_tf: 未运行"
    return 1
  fi

  pid=$(cat "$PID_FILE")
  if kill -0 "$pid" 2>/dev/null; then
    echo "[INFO] robot_tf: 运行中 (PID: $pid)"
    echo "[INFO] 日志文件: $LOG_FILE"

    # Show last few lines of log
    if [[ -f "$LOG_FILE" ]]; then
      echo ""
      echo "最近的日志输出："
      echo "----------------------------------------"
      tail -n 10 "$LOG_FILE"
      echo "----------------------------------------"
    fi

    # Check if service is available
    echo ""
    echo "服务状态："
    if ros2 service list 2>/dev/null | grep -q "/tf_tools/transform_points"; then
      echo "  ✅ /tf_tools/transform_points 服务可用"
    else
      echo "  ⚠️  /tf_tools/transform_points 服务未就绪"
    fi

    return 0
  else
    echo "[INFO] robot_tf: 未运行 (PID 文件存在但进程已退出)"
    return 1
  fi
}

# Parse arguments
MODE="start"
if [[ $# -gt 0 ]]; then
  case "$1" in
    --start|--daemon)
      MODE="start"
      ;;
    --stop)
      stop_daemon
      exit 0
      ;;
    --status)
      show_status
      exit $?
      ;;
    --foreground)
      MODE="foreground"
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[ERROR] 未知参数: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
fi

# Check prerequisites
check_ros_env
source_workspace
check_urdf
check_script

# Execute based on mode
case "$MODE" in
  foreground)
    start_foreground
    ;;
  start)
    start_daemon
    ;;
  *)
    echo "[ERROR] 未知模式: $MODE" >&2
    exit 2
    ;;
esac

