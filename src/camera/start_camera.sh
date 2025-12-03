#!/usr/bin/env bash
# RealSense bringup helper: start/stop realsense2_camera with health checks.
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}" )" && pwd)
DEFAULT_CONFIG="$SCRIPT_DIR/config/cameras/realsense2_d435i.yaml"
LOG_FILE="$SCRIPT_DIR/realsense2_camera.log"
CAMERA_NODE_NAME="/camera/realsense2_camera"
COLOR_TOPIC="/camera/color/image_raw"

usage() {
  cat <<USAGE
用法:
  $0 --start [配置文件]   启动 RealSense 节点（可选自定义参数文件）
  $0 --stop               停止 RealSense 节点
USAGE
  exit 1
}

ros2_available() {
  command -v ros2 >/dev/null 2>&1
}

ensure_ros2() {
  if ! ros2_available; then
    echo "未检测到 ros2 CLI，请先 source 对应 ROS 2 环境 (setup.bash)。" >&2
    exit 1
  fi
}

node_running() {
  if ! ros2_available; then
    return 1
  fi
  local nodes
  if ! nodes=$(ros2 node list 2>/dev/null); then
    return 1
  fi
  if grep -Fxq "$CAMERA_NODE_NAME" <<<"$nodes"; then
    return 0
  fi
  return 1
}

topic_ready() {
  if ! ros2_available; then
    return 1
  fi
  local topics
  if ! topics=$(ros2 topic list 2>/dev/null); then
    return 1
  fi
  if grep -Fxq "$COLOR_TOPIC" <<<"$topics"; then
    return 0
  fi
  return 1
}

print_banner() {
  local config="$1"
  cat <<INFO
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Camera Bringup
  - Config: $config
  - Namespace: /camera
  - Topics: /camera/color/image_raw, /camera/aligned_depth_to_color/image_raw
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
日志: $LOG_FILE
INFO
}

wait_for_ready() {
  local retries=20
  for ((i=0; i<retries; i++)); do
    if node_running && topic_ready; then
      return 0
    fi
    sleep 1
  done
  return 1
}

stop_processes() {
  if pgrep -u "$USER" -f "realsense2_camera_node" >/dev/null 2>&1; then
    pkill -u "$USER" -f "realsense2_camera_node" || true
    echo "RealSense 节点已停止。"
  else
    echo "未发现运行中的 RealSense 节点。"
  fi
}

start_camera() {
  ensure_ros2
  local config="${1:-$DEFAULT_CONFIG}"

  if [[ ! -f "$config" ]]; then
    echo "未找到配置文件：$config" >&2
    exit 1
  fi

  if node_running && topic_ready; then
    echo "检测到 RealSense 节点已运行且话题稳定，跳过启动。"
    return 0
  fi

  if node_running; then
    echo "检测到节点存在但话题缺失，先尝试停止后重启。"
    stop_processes >/dev/null
  fi

  print_banner "$config"

  nohup ros2 run realsense2_camera realsense2_camera_node \
    --ros-args \
    --params-file "$config" \
    -r __ns:=/camera \
    -r __node:=realsense2_camera \
    -r '~/color/image_raw:=color/image_raw' \
    -r '~/color/camera_info:=color/camera_info' \
    -r '~/aligned_depth_to_color/image_raw:=aligned_depth_to_color/image_raw' \
    -r '~/aligned_depth_to_color/camera_info:=aligned_depth_to_color/camera_info' \
    -r '~/depth/image_rect_raw:=depth/image_rect_raw' \
    -r '~/depth/camera_info:=depth/camera_info' \
    >>"$LOG_FILE" 2>&1 &

  echo "RealSense 节点启动中..."

  if wait_for_ready; then
    echo "RealSense 节点已就绪，开始发布 $COLOR_TOPIC。"
  else
    echo "警告：在超时时间内未检测到 $COLOR_TOPIC，请查看 $LOG_FILE。"
    return 1
  fi
}

main() {
  local action="${1:-}"
  case "$action" in
    --start|start)
      shift || true
      start_camera "$@"
      ;;
    --stop|stop)
      stop_processes
      ;;
    *)
      usage
      ;;
  esac
}

main "$@"
