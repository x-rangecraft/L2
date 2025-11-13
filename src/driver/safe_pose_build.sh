#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
INSTALL_SETUP="${WORKSPACE_ROOT}/install/setup.bash"
DEFAULT_OUTPUT_DIR="$(pwd)"
JOINT_TOPIC="/joint_states"
CAPTURE_TIMEOUT=10
ZERO_G_SERVICE="/robot_driver/service/zero_gravity"
SKIP_ZERO_G=0

usage() {
    cat <<'USAGE'
Usage: safe_pose_build.sh [options]

Options:
  --output-dir <dir>     保存 safe_pose_*.yaml 的目录（默认当前工作目录）
  --topic <name>         JointState 话题名（默认 /joint_states）
  --timeout <sec>        采集超时时间，默认 10 秒
  --service <name>       零重力 service 名称，默认 /robot_driver/service/zero_gravity
  --skip-zero-gravity    跳过自动切换零重力
  -h, --help             显示帮助
USAGE
}

info() { printf '[safe_pose] %s\n' "$1"; }
warn() { printf '[safe_pose][WARN] %s\n' "$1" >&2; }

resolve_path() {
    local target="$1"
    if [[ "$target" = /* ]]; then
        printf '%s\n' "$target"
    else
        python3 -c 'import os,sys; print(os.path.abspath(sys.argv[1]))' "$target"
    fi
}

source_if_exists() {
    local file="$1"
    if [[ -f "$file" ]]; then
        local had_nounset=0
        if [[ $- == *u* ]]; then
            had_nounset=1
            set +u
        fi
        # shellcheck source=/dev/null
        source "$file"
        if [[ ${had_nounset} -eq 1 ]]; then
            set -u
        fi
        return 0
    fi
    return 1
}

source_ros_env() {
    local sourced=0
    if source_if_exists "${ROS_SETUP}"; then
        sourced=1
    fi
    if source_if_exists "${INSTALL_SETUP}"; then
        sourced=1
    fi
    if [[ ${sourced} -eq 0 ]]; then
        warn "未找到 ROS 2 环境: ${ROS_SETUP} or ${INSTALL_SETUP}"
        exit 1
    fi
}

ensure_command() {
    if ! command -v "$1" >/dev/null 2>&1; then
        warn "Missing required command: $1"
        exit 1
    fi
}

ensure_command python3
ensure_command ros2

OUTPUT_DIR="${DEFAULT_OUTPUT_DIR}"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --output-dir)
            if [[ $# -lt 2 ]]; then
                warn "--output-dir 缺少参数"
                exit 1
            fi
            OUTPUT_DIR="$(resolve_path "$2")"
            shift 2
            ;;
        --output-dir=*)
            value="${1#*=}"
            OUTPUT_DIR="$(resolve_path "$value")"
            shift
            ;;
        --topic)
            if [[ $# -lt 2 ]]; then
                warn "--topic 缺少参数"
                exit 1
            fi
            JOINT_TOPIC="$2"
            shift 2
            ;;
        --topic=*)
            JOINT_TOPIC="${1#*=}"
            shift
            ;;
        --timeout)
            if [[ $# -lt 2 ]]; then
                warn "--timeout 缺少参数"
                exit 1
            fi
            CAPTURE_TIMEOUT="$2"
            shift 2
            ;;
        --timeout=*)
            CAPTURE_TIMEOUT="${1#*=}"
            shift
            ;;
        --service)
            if [[ $# -lt 2 ]]; then
                warn "--service 缺少参数"
                exit 1
            fi
            ZERO_G_SERVICE="$2"
            shift 2
            ;;
        --service=*)
            ZERO_G_SERVICE="${1#*=}"
            shift
            ;;
        --skip-zero-gravity)
            SKIP_ZERO_G=1
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            warn "未知参数: $1"
            usage
            exit 1
            ;;
    esac
done

mkdir -p "${OUTPUT_DIR}"
source_ros_env

if [[ ${SKIP_ZERO_G} -eq 0 ]]; then
    info "切换零重力模式 (${ZERO_G_SERVICE}) ..."
    if ! ros2 service call "${ZERO_G_SERVICE}" std_srvs/srv/SetBool "{data: true}" >/dev/null 2>&1; then
        warn "零重力 service 调用失败，继续执行"
    else
        info "零重力请求已发送。"
    fi
else
    info "已跳过自动零重力开关。"
fi

info "请将机械臂拖到期望 SAFE_POSE，完成后按回车继续。"
read -r _

info "正在采集 ${JOINT_TOPIC} ..."
set +e
JOINT_STATE_JSON="$(SAFE_POSE_TOPIC="${JOINT_TOPIC}" SAFE_POSE_TIMEOUT="${CAPTURE_TIMEOUT}" python3 <<'PY'
import json
import os
import sys
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

TOPIC = os.environ.get("SAFE_POSE_TOPIC", "/joint_states")
TIMEOUT = float(os.environ.get("SAFE_POSE_TIMEOUT", "10"))

class JointStateCapture(Node):
    def __init__(self):
        super().__init__('safe_pose_capture_cli')
        self._future = self.create_future()
        self.create_subscription(JointState, TOPIC, self._callback, 10)

    def _callback(self, msg: JointState):
        if not self._future.done():
            self._future.set_result(msg)

rclpy.init()
node = JointStateCapture()
executor = rclpy.executors.SingleThreadedExecutor()
executor.add_node(node)
start_time = time.time()
msg = None
try:
    while rclpy.ok():
        executor.spin_once(timeout_sec=0.1)
        if node._future.done():
            msg = node._future.result()
            break
        if time.time() - start_time > TIMEOUT:
            node.get_logger().error(f"Timeout waiting for {TOPIC}")
            rclpy.shutdown()
            sys.exit(2)
finally:
    executor.remove_node(node)

if msg is None:
    rclpy.shutdown()
    sys.exit(1)

data = {
    "names": list(msg.name),
    "positions": list(msg.position),
    "velocities": list(msg.velocity),
    "efforts": list(msg.effort),
    "frame_id": msg.header.frame_id,
    "stamp_sec": msg.header.stamp.sec,
    "stamp_nanosec": msg.header.stamp.nanosec,
}
print(json.dumps(data))
rclpy.shutdown()
PY)"
CAPTURE_STATUS=$?
set -e

if [[ ${CAPTURE_STATUS} -ne 0 ]]; then
    warn "采集 JointState 失败 (exit ${CAPTURE_STATUS})"
    exit ${CAPTURE_STATUS}
fi

if [[ -z "${JOINT_STATE_JSON}" ]]; then
    warn "未收到 JointState 数据"
    exit 1
fi

JOINT_STATE_JSON="${JOINT_STATE_JSON}" python3 <<'PY'
import json
import os
from itertools import zip_longest

data = json.loads(os.environ['JOINT_STATE_JSON'])
print("捕获关节姿态:")
for name, pos in zip_longest(data.get('names', []), data.get('positions', []), fillvalue=0.0):
    if name is None:
        continue
    print(f"  {name}: {pos:.6f} rad")
print()
PY

read -rp "确认写入 safe_pose YAML? [y/N]: " confirm
if [[ ! "${confirm}" =~ ^[Yy]$ ]]; then
    info "用户取消写入。"
    exit 0
fi

TIMESTAMP="$(date +'%Y%m%d_%H%M%S')"
OUTPUT_FILE="${OUTPUT_DIR}/safe_pose_${TIMESTAMP}.yaml"
JOINT_STATE_JSON="${JOINT_STATE_JSON}" OUTPUT_FILE="${OUTPUT_FILE}" CAPTURE_TOPIC="${JOINT_TOPIC}" python3 <<'PY'
import json
import os
from datetime import datetime

data = json.loads(os.environ['JOINT_STATE_JSON'])
path = os.environ['OUTPUT_FILE']
topic = os.environ.get('CAPTURE_TOPIC', '/joint_states')
now = datetime.utcnow().isoformat() + 'Z'

joint_names = data.get('names', [])
positions = data.get('positions', [])
frame_id = data.get('frame_id', '')

def fmt_list(values):
    lines = []
    for value in values:
        if isinstance(value, float):
            lines.append(f"  - {value:.6f}")
        else:
            lines.append(f"  - {value}")
    return "\n".join(lines)

content = [
    "# SAFE_POSE captured via safe_pose_build.sh",
    f"# Captured at {now}",
    "joint_names:",
    fmt_list(joint_names),
    "positions:",
    fmt_list(positions),
    "metadata:",
    f"  frame_id: {frame_id if frame_id else 'base_link'}",
    f"  source_topic: {topic}",
    f"  captured_at: {now}",
    "  zero_gravity_requested: true",
    "  notes: ''",
]

with open(path, 'w', encoding='utf-8') as f:
    for line in content:
        if line:
            f.write(f"{line}\n")
        else:
            f.write("\n")

print(f"已写入 {path}")
PY

info "SAFE_POSE YAML 已生成：${OUTPUT_FILE}"
info "可将内容复制到 config/safe_pose_default.yaml 的 safe_pose 列表中。"
