#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_FILE="${SCRIPT_DIR}/static_tf_config.yaml"
STATE_DIR="${SCRIPT_DIR}/.static_tf_publish"
PID_FILE="${STATE_DIR}/pids"
LOG_DIR="${STATE_DIR}/logs"

usage() {
  cat <<'EOF'
用法：static_tf_config_publish.sh [--daemon|--stop|--status]
  (默认) 无参数：前台阻塞式发布，按 Ctrl+C 停止。
  --daemon / --start：后台常驻发布，脚本退出后进程继续运行。
  --stop：停止后台常驻发布的所有 TF 进程。
  --status：查看后台常驻发布当前状态。
EOF
}

ensure_state_dir() {
  mkdir -p "$STATE_DIR" "$LOG_DIR"
}

stop_daemon() {
  if [[ ! -f "$PID_FILE" ]]; then
    echo "[INFO] 未发现后台 TF 进程。"
    return 0
  fi
  local stopped=0
  while read -r pid name; do
    [[ -z "${pid:-}" ]] && continue
    if kill -0 "$pid" 2>/dev/null; then
      kill "$pid" 2>/dev/null || true
      echo "[INFO] 已停止 ${name} (PID ${pid})"
      stopped=1
    else
      echo "[INFO] ${name} (PID ${pid}) 已不在运行"
    fi
  done <"$PID_FILE"
  rm -f "$PID_FILE"
  if [[ $stopped -eq 0 ]]; then
    echo "[INFO] 未找到存活的后台进程。"
  fi
}

status_daemon() {
  if [[ ! -f "$PID_FILE" ]]; then
    echo "[INFO] 未检测到后台 TF 进程。"
    return 1
  fi
  echo "[INFO] 后台 TF 进程状态："
  local has_running=0
  while read -r pid name; do
    [[ -z "${pid:-}" ]] && continue
    if kill -0 "$pid" 2>/dev/null; then
      echo "  - ${name}: 运行中 (PID ${pid})"
      has_running=1
    else
      echo "  - ${name}: 已退出 (PID ${pid})"
    fi
  done <"$PID_FILE"
  if [[ $has_running -eq 0 ]]; then
    return 1
  fi
}

show_config() {
  echo "========== static_tf_config.yaml =========="
  cat "$CONFIG_FILE"
  echo "==========================================="
}

MODE="foreground"
if [[ $# -gt 0 ]]; then
  case "$1" in
    --daemon|--start)
      MODE="daemon"
      shift
      ;;
    --stop)
      stop_daemon
      exit 0
      ;;
    --status)
      status_daemon
      exit 0
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[ERROR] 未知参数：$1" >&2
      usage >&2
      exit 2
      ;;
  esac
fi

if [[ ! -f "$CONFIG_FILE" ]]; then
  echo "[ERROR] 未找到 $CONFIG_FILE，请先运行 static_tf_config_build.sh" >&2
  exit 1
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "[ERROR] 未检测到 ros2 命令，请先 source 对应的 ROS 2 环境" >&2
  exit 1
fi

generate_transform_lines() {
  CONFIG_PATH="$CONFIG_FILE" python3 <<'PY'
import json, math, os
from pathlib import Path
path = Path(os.environ["CONFIG_PATH"])
with path.open() as f:
    data = json.load(f)
transforms = data.get('transforms', {})
order = []
seen = set()
for edge in data.get('tf_tree', []):
    key = f"{edge.get('parent')}_to_{edge.get('child')}"
    if key in transforms and key not in seen:
        order.append((key, transforms[key]))
        seen.add(key)
for key, value in transforms.items():
    if key not in seen:
        order.append((key, value))
        seen.add(key)
for name, t in order:
    try:
        parent = t['parent_frame']
        child = t['child_frame']
        tr = t['translation_m']
        quat = t['quaternion']['forward_parent_to_child']
    except KeyError as exc:
        raise SystemExit(f"配置缺失字段: {exc}")
    print(
        f"{parent} {child} {tr['x']} {tr['y']} {tr['z']} "
        f"{quat['x']} {quat['y']} {quat['z']} {quat['w']} {name}"
    )
chain = data.get('urdf_chain', {})
links = chain.get('links', [])
def rpy_to_quat(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )
for link in links:
    parent = link.get('parent')
    child = link.get('child')
    if not parent or not child:
        continue
    origin_xyz = link.get('origin_xyz', {})
    origin_rpy = link.get('origin_rpy', {})
    tx = origin_xyz.get('x', 0.0)
    ty = origin_xyz.get('y', 0.0)
    tz = origin_xyz.get('z', 0.0)
    roll = origin_rpy.get('roll', 0.0)
    pitch = origin_rpy.get('pitch', 0.0)
    yaw = origin_rpy.get('yaw', 0.0)
    qx, qy, qz, qw = rpy_to_quat(roll, pitch, yaw)
    name = f"urdf_{link.get('joint', f'{parent}_to_{child}')}"
    print(f"{parent} {child} {tx} {ty} {tz} {qx} {qy} {qz} {qw} {name}")
PY
}

readarray -t TRANSFORM_LINES < <(generate_transform_lines)

if [[ ${#TRANSFORM_LINES[@]} -eq 0 ]]; then
  echo "[ERROR] 配置中未找到任何 transforms" >&2
  exit 1
fi

pids=()
cleanup() {
  if [[ ${#pids[@]} -gt 0 ]]; then
    printf "\n正在停止静态 TF 发布进程...\n"
    for pid in "${pids[@]}"; do
      kill "$pid" 2>/dev/null || true
    done
    if [[ ${#pids[@]} -gt 0 ]]; then
      wait "${pids[@]}" 2>/dev/null || true
    fi
  fi
}

start_foreground() {
  trap cleanup INT TERM
  echo "启动静态 TF 发布..."
  for line in "${TRANSFORM_LINES[@]}"; do
    read -r parent child tx ty tz qx qy qz qw name <<<"$line"
    echo "  - ${name}: ${parent} -> ${child}"
    ros2 run tf2_ros static_transform_publisher \
      "$tx" "$ty" "$tz" "$qx" "$qy" "$qz" "$qw" \
      "$parent" "$child" &
    pids+=("$!")
  done
  echo "所有静态 TF 节点已启动，按 Ctrl+C 停止。"
  wait "${pids[@]}"
}

start_daemon() {
  ensure_state_dir
  : >"$PID_FILE"
  echo "后台启动静态 TF 发布..."
  for line in "${TRANSFORM_LINES[@]}"; do
    read -r parent child tx ty tz qx qy qz qw name <<<"$line"
    log_file="${LOG_DIR}/${name}.log"
    nohup ros2 run tf2_ros static_transform_publisher \
      "$tx" "$ty" "$tz" "$qx" "$qy" "$qz" "$qw" \
      "$parent" "$child" >"$log_file" 2>&1 &
    pid=$!
    echo "${pid} ${name}" >>"$PID_FILE"
    echo "  - ${name}: ${parent} -> ${child} (PID ${pid}, 日志：${log_file})"
  done
  echo "[INFO] 后台 TF 发布器已启动。PID 文件：${PID_FILE}"
  echo "[INFO] 如需停止可执行：static_tf_config_publish.sh --stop"
  show_config
}

case "$MODE" in
  foreground)
    start_foreground
    ;;
  daemon)
    start_daemon
    ;;
  *)
    echo "[ERROR] 未知模式：$MODE" >&2
    exit 2
    ;;
esac
