#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
INSTALL_SETUP="${WORKSPACE_ROOT}/install/setup.bash"
JOINT_TOPIC="/joint_states"
ZERO_G_SERVICE="/robot_driver/service/zero_gravity"
CAPTURE_TIMEOUT=10
OUTPUT_FILE="${WORKSPACE_ROOT}/src/driver/config/safe_pose_config.yaml"
REQUIRED_JOINTS=(joint1 joint2 joint3 joint4 joint5 joint6 gripper)
REQUIRED_JOINTS_STR="$(IFS=,; printf '%s' "${REQUIRED_JOINTS[*]}")"

info() { printf '[safe_pose] %s\n' "$1"; }
warn() { printf '[safe_pose][WARN] %s\n' "$1" >&2; }

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
        warn "未找到 ROS 2 环境: ${ROS_SETUP} 或 ${INSTALL_SETUP}"
        exit 1
    fi
}

ensure_command() {
    if ! command -v "$1" >/dev/null 2>&1; then
        warn "缺少命令: $1"
        exit 1
    fi
}

check_robot_driver_node() {
    info "[Step 1/5] 检查 /robot_driver 节点..."
    local nodes
    nodes="$(ros2 node list 2>/dev/null || true)"
    if ! printf '%s\n' "$nodes" | grep -qx "/robot_driver"; then
        warn "未检测到 /robot_driver 节点，请先启动 robot_driver。"
        exit 1
    fi
    info "[Step 1/5] 完成：/robot_driver 在线。"
}

check_joint_states_topic() {
    info "[Step 2/5] 检查 ${JOINT_TOPIC} 发布状态..."
    if ! ros2 topic list 2>/dev/null | grep -qx "${JOINT_TOPIC}"; then
        warn "未发现 ${JOINT_TOPIC} 话题。"
        exit 1
    fi

    local topic_info publisher_count
    if ! topic_info="$(ros2 topic info "${JOINT_TOPIC}" 2>&1)"; then
        warn "获取 ${JOINT_TOPIC} 信息失败。"
        printf '%s\n' "${topic_info}"
        exit 1
    fi

    if ! publisher_count="$(
        TOPIC_INFO_DATA="${topic_info}" python3 <<'PY'
import os
text = os.environ['TOPIC_INFO_DATA']
for line in text.splitlines():
    if 'Publisher count' in line:
        try:
            print(int(line.split(':', 1)[1].strip()))
        except (ValueError, IndexError):
            pass
        break
PY
    )"; then
        warn "解析 ${JOINT_TOPIC} 发布者数量失败。"
        exit 1
    fi

    if [[ -z "${publisher_count}" ]]; then
        warn "无法解析 ${JOINT_TOPIC} 发布者数量。"
        printf '%s\n' "${topic_info}"
        exit 1
    fi

    if (( publisher_count <= 0 )); then
        warn "${JOINT_TOPIC} 没有发布者，无法继续。"
        printf '%s\n' "${topic_info}"
        exit 1
    fi

    info "[Step 2/5] 完成：检测到 ${publisher_count} 个发布者。"
}

enable_zero_gravity_mode() {
    info "[Step 3/5] 调用 ${ZERO_G_SERVICE} 启动零重力模式..."
    set +e
    local service_output
    service_output="$(timeout 5 ros2 service call "${ZERO_G_SERVICE}" std_srvs/srv/SetBool "{data: true}" 2>&1)"
    local status=$?
    set -e

    if [[ ${status} -eq 124 ]]; then
        warn "零重力 service 调用超时。"
        exit 1
    fi
    if [[ ${status} -ne 0 ]]; then
        warn "零重力 service 调用失败 (exit ${status})。"
        printf '%s\n' "${service_output}"
        exit 1
    fi
    if ! printf '%s\n' "${service_output}" | grep -qiE "success[:=][[:space:]]*true"; then
        warn "零重力 service 返回失败："
        printf '%s\n' "${service_output}"
        exit 1
    fi

    info "[Step 3/5] 完成：零重力模式已启用。"
}

prompt_user_pose() {
    info "[Step 4/5] 请手动调整机械臂至安全姿态，完成后按回车继续..."
    read -r _
    info "[Step 4/5] 完成：用户已确认姿态。"
}

capture_and_write_pose() {
    info "[Step 5/5] 正在捕获 ${JOINT_TOPIC} 并写入 ${OUTPUT_FILE} ..."
    local capture_json
    set +e
    capture_json="$(
        SAFE_POSE_TOPIC="${JOINT_TOPIC}" \
        SAFE_POSE_TIMEOUT="${CAPTURE_TIMEOUT}" \
        SAFE_POSE_REQUIRED="${REQUIRED_JOINTS_STR}" \
        python3 <<'PY'
import json
import os
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.task import Future
from sensor_msgs.msg import JointState

topic = os.environ.get("SAFE_POSE_TOPIC", "/joint_states")
timeout = float(os.environ.get("SAFE_POSE_TIMEOUT", "10"))
required = [item.strip() for item in os.environ.get("SAFE_POSE_REQUIRED", "").split(',') if item.strip()]

if not required:
    sys.stderr.write("未指定需要解析的关节名称。\n")
    sys.exit(3)

class JointStateCapture(Node):
    def __init__(self):
        super().__init__('safe_pose_capture_cli')
        self._future = Future()
        self.create_subscription(JointState, topic, self._callback, 10)

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
        if time.time() - start_time > timeout:
            node.get_logger().error(f"Timeout waiting for {topic}")
            rclpy.shutdown()
            sys.exit(2)
finally:
    executor.remove_node(node)

if msg is None:
    rclpy.shutdown()
    sys.exit(1)

positions = dict(zip(msg.name, msg.position))
missing = [name for name in required if name not in positions]
if missing:
    sys.stderr.write("缺少以下关节数据: " + ", ".join(missing) + "\n")
    rclpy.shutdown()
    sys.exit(3)

joint_entries = [{"name": name, "position": float(positions[name])} for name in required]

payload = {
    "joint_entries": joint_entries,
    "frame_id": msg.header.frame_id,
    "stamp_sec": int(msg.header.stamp.sec),
    "stamp_nanosec": int(msg.header.stamp.nanosec),
}

print(json.dumps(payload))
rclpy.shutdown()
PY
    )"
    local capture_status=$?
    set -e

    if [[ ${capture_status} -ne 0 ]]; then
        warn "捕获 ${JOINT_TOPIC} 失败 (exit ${capture_status})。"
        exit ${capture_status}
    fi
    if [[ -z "${capture_json}" ]]; then
        warn "未收到任何 JointState 数据。"
        exit 1
    fi

    mkdir -p "$(dirname -- "${OUTPUT_FILE}")"

    SAFE_POSE_CAPTURE="${capture_json}" SAFE_POSE_OUTPUT="${OUTPUT_FILE}" SAFE_POSE_TOPIC="${JOINT_TOPIC}" \
        python3 <<'PY'
import json
import os
from datetime import datetime, timezone

raw = json.loads(os.environ['SAFE_POSE_CAPTURE'])
output_path = os.environ['SAFE_POSE_OUTPUT']
topic = os.environ.get('SAFE_POSE_TOPIC', '/joint_states')

joint_entries = raw.get('joint_entries') or []
if not joint_entries:
    raise SystemExit("joint_entries 为空，无法写入。")

now = datetime.now(timezone.utc).isoformat()
frame_id = raw.get('frame_id') or 'base_link'

lines = [
    "# SAFE_POSE auto-generated via safe_pose_build.sh",
    f"# Captured at {now}",
    "safe_pose:",
    "  joints:",
]

for entry in joint_entries:
    name = entry.get('name', '').strip() or 'unknown_joint'
    value = float(entry.get('position', 0.0))
    lines.append(f"    {name}: {value:.6f}")

lines.extend([
    "  metadata:",
    "    ready: true",
    f"    captured_at: {now}",
    f"    source_topic: {topic}",
    f"    frame_id: {frame_id}",
    "    zero_gravity_requested: true",
])

with open(output_path, 'w', encoding='utf-8') as fp:
    fp.write("\n".join(lines) + "\n")

print(f"写入 {output_path}")
PY

    info "[Step 5/5] 完成：SAFE_POSE 已写入 ${OUTPUT_FILE}。"
}

main() {
    source_ros_env
    ensure_command ros2
    ensure_command python3

    check_robot_driver_node
    check_joint_states_topic
    enable_zero_gravity_mode
    prompt_user_pose
    capture_and_write_pose
}

main "$@"
