#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
INSTALL_SETUP="${WORKSPACE_ROOT}/install/setup.bash"
DEFAULT_OUTPUT_DIR="${SCRIPT_DIR}/config"
JOINT_TOPIC="/joint_states"
CAPTURE_TIMEOUT=10
ZERO_G_SERVICE="/robot_driver/service/zero_gravity"
CAN_CHANNEL="can0"
MARK_READY_TARGET=""

usage() {
    cat <<'USAGE'
Usage: safe_pose_build.sh [options]

Options:
  --output-dir <dir>     保存 safe_pose_*.yaml 的目录（默认 driver/config）
  --topic <name>         JointState 话题名（默认 /joint_states）
  --timeout <sec>        采集超时时间（默认 10 秒）
  --service <name>       零重力 service 名称（默认 /robot_driver/service/zero_gravity）
  --can <canX>           指定要检查的 CAN 通道（默认 can0）
  --mark-ready <file>    仅标记已有 safe_pose YAML 为可用（写 metadata.ready 并创建 .ready）
  -h, --help             显示帮助

说明:
  此脚本必须在 robot_driver_node 运行的情况下使用。
  脚本会自动启动零重力模式，让你可以手动拖动机械臂到安全姿态。
  请确保先运行: ./src/driver/start_robot_driver.sh
USAGE
}

info() { printf '[safe_pose] %s\n' "$1"; }
warn() { printf '[safe_pose][WARN] %s\n' "$1" >&2; }

mark_safe_pose_ready_file() {
    local target="$1"
    if [[ ! -f "$target" ]]; then
        warn "未找到 safe_pose 文件: $target"
        exit 1
    fi

    if ! SAFE_POSE_MARK_TARGET="$target" python3 <<'PY'
import os
import sys
import yaml

path = os.environ['SAFE_POSE_MARK_TARGET']
with open(path, 'r', encoding='utf-8') as f:
    data = yaml.safe_load(f) or {}

wrapped = False
node = data
if isinstance(data, dict) and 'safe_pose' in data:
    node = data['safe_pose']
    wrapped = True

if not isinstance(node, dict):
    raise SystemExit(f'无法解析 {path}，请确认 safe_pose YAML 格式正确')

metadata = node.setdefault('metadata', {})
metadata['ready'] = True

payload = {'safe_pose': node} if wrapped else node
with open(path, 'w', encoding='utf-8') as f:
    yaml.safe_dump(payload, f, allow_unicode=True, sort_keys=False)
PY
    then
        warn "写入 metadata.ready 失败"
        exit 1
    fi

    touch "${target}.ready"
    info "已标记 ${target} 可用，并创建 ${target}.ready"
}

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
        --can)
            if [[ $# -lt 2 ]]; then
                warn "--can 缺少参数"
                exit 1
            fi
            CAN_CHANNEL="$2"
            shift 2
            ;;
        --can=*)
            CAN_CHANNEL="${1#*=}"
            shift
            ;;
        --mark-ready)
            if [[ $# -lt 2 ]]; then
                warn "--mark-ready 缺少参数"
                exit 1
            fi
            MARK_READY_TARGET="$(resolve_path "$2")"
            shift 2
            ;;
        --mark-ready=*)
            value="${1#*=}"
            MARK_READY_TARGET="$(resolve_path "$value")"
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

if [[ -n "${MARK_READY_TARGET}" ]]; then
    mark_safe_pose_ready_file "${MARK_READY_TARGET}"
    exit 0
fi

ensure_command ros2

mkdir -p "${OUTPUT_DIR}"
source_ros_env

info "检查 robot_driver_node 是否运行..."
if ! ros2 node list 2>/dev/null | grep -q "/robot_driver"; then
    warn "robot_driver_node 未运行！"

    # robot_driver 未运行，检查是否有其他进程占用 CAN 接口
    info "检查 CAN 接口 (${CAN_CHANNEL}) 占用情况..."
    CAN_USERS=$(sudo lsof 2>/dev/null | grep -i "${CAN_CHANNEL}" | awk '{print $2}' | sort -u || true)
    I2RT_PIDS=$(ps aux | grep -E "python.*(i2rt|stage1_device)" | grep -v grep | awk '{print $2}' || true)

    if [[ -n "${CAN_USERS}" || -n "${I2RT_PIDS}" ]]; then
        warn "发现其他进程可能占用了 CAN 接口或 I2RT SDK"
        if [[ -n "${CAN_USERS}" ]]; then
            warn "  占用 ${CAN_CHANNEL} 的进程: ${CAN_USERS}"
        fi
        if [[ -n "${I2RT_PIDS}" ]]; then
            warn "  I2RT 相关进程: ${I2RT_PIDS}"
        fi

        CLEANUP_SCRIPT="${SCRIPT_DIR}/scripts/cleanup_can_interface.sh"
        if [[ -x "${CLEANUP_SCRIPT}" ]]; then
            read -rp "是否清理这些进程后再启动 robot_driver? [y/N]: " confirm_cleanup
            if [[ "${confirm_cleanup}" =~ ^[Yy]$ ]]; then
                info "清理占用进程..."
                if ! bash "${CLEANUP_SCRIPT}" "${CAN_CHANNEL}" --force; then
                    warn "清理失败"
                    exit 1
                fi
                info "清理完成，请先运行: ./src/driver/start_robot_driver.sh"
            else
                warn "用户取消清理"
            fi
        fi
    else
        info "未发现占用 CAN 接口的进程"
    fi

    warn "请先启动 robot_driver: ./src/driver/start_robot_driver.sh"
    exit 1
fi

info "robot_driver_node 运行正常"

info "切换零重力模式 (${ZERO_G_SERVICE}) ..."
set +e
SERVICE_RESULT="$(timeout 5 ros2 service call "${ZERO_G_SERVICE}" std_srvs/srv/SetBool "{data: true}" 2>&1)"
SERVICE_EXIT=$?
set -e
if [[ ${SERVICE_EXIT} -eq 124 ]]; then
    warn "零重力 service 调用超时！"
    warn "没有零重力模式无法手动调整机械臂姿态。"
    exit 1
elif [[ ${SERVICE_EXIT} -ne 0 ]]; then
    warn "零重力 service 调用失败 (exit ${SERVICE_EXIT})！"
    printf '%s\n' "${SERVICE_RESULT}"
    warn "没有零重力模式无法手动调整机械臂姿态。"
    exit 1
elif ! printf '%s' "${SERVICE_RESULT}" | grep -qiE "success[:=][[:space:]]*true"; then
    warn "零重力 service 返回失败："
    printf '%s\n' "${SERVICE_RESULT}"
    warn "请检查 robot_driver_node 日志。"
    exit 1
else
    info "零重力模式已启动。"
fi

info "请将机械臂拖到期望 SAFE_POSE，完成后按回车继续。"
read -r _

info "正在采集 ${JOINT_TOPIC} ..."
set +e
JOINT_STATE_JSON="$(
SAFE_POSE_TOPIC="${JOINT_TOPIC}" SAFE_POSE_TIMEOUT="${CAPTURE_TIMEOUT}" \
python3 <<'PY'
import json
import os
import sys
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.task import Future

TOPIC = os.environ.get("SAFE_POSE_TOPIC", "/joint_states")
TIMEOUT = float(os.environ.get("SAFE_POSE_TIMEOUT", "10"))

class JointStateCapture(Node):
    def __init__(self):
        super().__init__('safe_pose_capture_cli')
        self._future = Future()
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

print("RAW JointState message:", file=sys.stderr)
print(msg, file=sys.stderr)

data = {
    "names": list(msg.name),
    "positions": list(msg.position),
    "velocities": list(msg.velocity),
    "efforts": list(msg.effort),
    "frame_id": msg.header.frame_id,
    "stamp_sec": msg.header.stamp.sec,
    "stamp_nanosec": msg.header.stamp.nanosec,
}
pretty_json = json.dumps(data, indent=2)
print("Formatted JointState JSON:", file=sys.stderr)
print(pretty_json, file=sys.stderr)
print(json.dumps(data))
rclpy.shutdown()
PY
)"
CAPTURE_STATUS=$?
set -e

if [[ ${CAPTURE_STATUS} -ne 0 ]]; then
    warn "采集 JointState 失败 (exit ${CAPTURE_STATUS})"
    case "${CAPTURE_STATUS}" in
        1)
            warn "错误原因: JointState 消息为空，Python 捕获脚本返回 1"
            ;;
        2)
            warn "错误原因: 等待 ${JOINT_TOPIC} 超时 (${CAPTURE_TIMEOUT}s)"
            ;;
        *)
            warn "错误原因: Python 捕获脚本异常，详见上方输出"
            ;;
    esac
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
    "  ready: true",
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
mark_safe_pose_ready_file "${OUTPUT_FILE}"

info "SAFE_POSE YAML 已生成并标记为 ready：${OUTPUT_FILE}"
info "可直接在 driver/config/robot_driver_config.yaml 的 safe_pose_file 字段引用该文件。"
