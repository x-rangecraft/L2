#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_ROOT="${SCRIPT_DIR}"
WORKSPACE_ROOT="$(cd -- "${PACKAGE_ROOT}/../.." && pwd)"
DEFAULT_PARAM_FILE="${PACKAGE_ROOT}/config/robot_driver_config.yaml"
if [[ -n "${ZSH_VERSION:-}" && -f "/opt/ros/humble/setup.zsh" ]]; then
    ROS_SETUP="/opt/ros/humble/setup.zsh"
else
    ROS_SETUP="/opt/ros/humble/setup.bash"
fi
INSTALL_SETUP="${WORKSPACE_ROOT}/install/setup.bash"
LOG_DIR="${WORKSPACE_ROOT}/log/robot_driver"
mkdir -p "${LOG_DIR}"
if [[ -n "${ROBOT_DRIVER_LOG_FILE_OVERRIDE:-}" ]]; then
    LOG_FILE="${ROBOT_DRIVER_LOG_FILE_OVERRIDE}"
else
    LOG_FILE="${LOG_DIR}/robot_driver_$(date +'%Y%m%d_%H%M%S').log"
fi
touch "${LOG_FILE}"

ORIGINAL_ARGS=("$@")
DAEMON_CHILD=0
STOP_ONLY=0
FORCE_STOP=0
FOLLOW_LOG=0
LOG_STDERR_MIRROR=1
VERBOSE_CONSOLE=0
declare -a CHILD_ARGS=()

DEFAULT_CAN_CHANNEL="can0"
CAN_CHANNEL="${DEFAULT_CAN_CHANNEL}"
CAN_CHANNEL_USER_SET=0
SUPERVISOR_PGREP_PATTERN='start_robot_driver\.sh.*--daemon-child'
XYZ_ONLY_MODE="false"
ZERO_GRAVITY_DEFAULT="true"
PARAM_FILE=""

if [[ -f "${DEFAULT_PARAM_FILE}" ]]; then
    PARAM_FILE="${DEFAULT_PARAM_FILE}"
fi

usage() {
    cat <<'USAGE'
Usage: start_robot_driver.sh [options]

Options:
  --can <canX>           指定 CAN 通道，默认 can0
  --xyz-only             仅在 XYZ 空间规划姿态（true）
  --no-xyz-only          恢复全姿态控制（默认）
  --params <file>        指定参数 YAML（默认 driver/config/robot_driver_config.yaml，若存在）
  --stop                 停止后台守护进程
  --force                配合 --stop 使用：SafetyPose 失败时仍然强制关闭进程
  --follow-log           后台模式下实时查看日志
  --no-follow-log        后台模式下不跟随日志（默认）
  --verbose              控制台打印更多启动进度 / 状态
  -h, --help             显示本帮助
USAGE
}

log() {
    local msg="$1"
    local timestamp
    timestamp="$(date --iso-8601=seconds)"
    local line="[${timestamp}] ${msg}"
    printf '%s\n' "${line}" >> "${LOG_FILE}"
    if [[ ${LOG_STDERR_MIRROR} -ne 0 ]]; then
        printf '%s\n' "${line}" >&2
    fi
}

status_cn() {
    local msg="$1"
    local prefix="${2:-INFO}"
    local time_str
    time_str="$(date +'%H:%M:%S')"
    local console_line="[robot_driver][${time_str}][${prefix}] ${msg}"
    printf '%s\n' "${console_line}"
    log "提示(${prefix}): ${msg}"
}

cleanup_processes() {
    log "Killing existing robot_driver processes (if any)..."
    # 关闭所有 ros2 launch robot_driver 进程
    pkill -f "ros2 launch robot_driver robot_driver.launch.py" 2>/dev/null || true
    # 关闭所有 robot_driver 节点进程（两种匹配都保留，尽量覆盖）
    pkill -f "robot_driver/lib/robot_driver/robot_driver_node.py" 2>/dev/null || true
    pkill -f robot_driver_node 2>/dev/null || true
}

get_cmdline_for_pid() {
    local pid="$1"
    if [[ -r "/proc/${pid}/cmdline" ]]; then
        tr '\0' ' ' < "/proc/${pid}/cmdline"
        return 0
    fi
    if command -v ps >/dev/null 2>&1; then
        ps -o args= -p "${pid}" 2>/dev/null || return 1
    fi
    return 1
}

extract_can_from_cmdline() {
    local cmdline="$1"
    local default_can="${2:-${DEFAULT_CAN_CHANNEL}}"
    local token next
    local -a tokens=()
    default_can="${default_can:-can0}"
    if [[ -z "${cmdline}" ]]; then
        printf '%s\n' "${default_can}"
        return 0
    fi
    # shellcheck disable=SC2206
    read -r -a tokens <<< "${cmdline}"
    local i=0
    local total=${#tokens[@]}
    while (( i < total )); do
        token="${tokens[i]}"
        if [[ "${token}" == --can ]]; then
            ((i+=1))
            if (( i < total )); then
                printf '%s\n' "${tokens[i]}"
                return 0
            fi
        elif [[ "${token}" == --can=* ]]; then
            printf '%s\n' "${token#*=}"
            return 0
        fi
        ((i+=1))
    done
    printf '%s\n' "${default_can}"
}

list_supervisors_for_can() {
    local target_can="$1"
    local include_all=0
    local found=0
    local default_can="${DEFAULT_CAN_CHANNEL:-can0}"
    if [[ -z "${target_can}" ]]; then
        include_all=1
    fi
    while IFS= read -r pid; do
        [[ -z "${pid}" ]] && continue
        if [[ ! "${pid}" =~ ^[0-9]+$ ]]; then
            continue
        fi
        local cmdline
        if ! cmdline="$(get_cmdline_for_pid "${pid}")"; then
            continue
        fi
        if [[ "${cmdline}" != *"--daemon-child"* ]]; then
            continue
        fi
        local cmd_can
        cmd_can="$(extract_can_from_cmdline "${cmdline}" "${default_can}")"
        if [[ ${include_all} -eq 1 || "${cmd_can}" == "${target_can}" ]]; then
            printf '%s\t%s\t%s\n' "${pid}" "${cmd_can}" "${cmdline}"
            found=1
        fi
    done < <(pgrep -f "${SUPERVISOR_PGREP_PATTERN}" 2>/dev/null || true)

    if [[ ${found} -eq 0 ]]; then
        return 1
    fi
    return 0
}

wait_for_pid_exit() {
    local pid="$1"
    local timeout="$2"
    local waited=0
    timeout="${timeout:-5}"
    while kill -0 "${pid}" 2>/dev/null; do
        if (( waited >= timeout )); then
            return 1
        fi
        sleep 1
        ((waited+=1))
    done
    return 0
}

terminate_supervisor_pid() {
    local pid="$1"
    local timeout="$2"
    timeout="${timeout:-10}"
    if ! kill "${pid}" 2>/dev/null; then
        return 1
    fi
    if wait_for_pid_exit "${pid}" "${timeout}"; then
        return 0
    fi
    kill -9 "${pid}" 2>/dev/null || true
    if wait_for_pid_exit "${pid}" 2; then
        return 0
    fi
    return 1
}

stop_daemon() {
    status_cn "正在查找需要关闭的 robot_driver 相关进程..."

    local supervisors launches nodes
    supervisors="$(pgrep -fa "${SUPERVISOR_PGREP_PATTERN}" 2>/dev/null || true)"
    launches="$(pgrep -fa "ros2 launch robot_driver robot_driver.launch.py" 2>/dev/null || true)"
    nodes="$(pgrep -fa "robot_driver/lib/robot_driver/robot_driver_node.py" 2>/dev/null || true)"

    local supervisor_filter=""
    if [[ ${CAN_CHANNEL_USER_SET} -eq 1 ]]; then
        supervisor_filter="${CAN_CHANNEL}"
    fi

    if [[ -z "${supervisors}${launches}${nodes}" ]]; then
        status_cn "未找到正在运行的 robot_driver 相关进程" "WARN"
        log "No robot_driver supervisors or nodes found to stop."
        return 0
    fi

    status_cn "已找到需要关闭的进程：" "INFO"
    if [[ -n "${supervisors}" ]]; then
        while IFS= read -r line; do
            [[ -z "${line}" ]] && continue
            status_cn "  supervisor: ${line}"
        done <<< "${supervisors}"
    fi
    if [[ -n "${launches}" ]]; then
        while IFS= read -r line; do
            [[ -z "${line}" ]] && continue
            status_cn "  launch:     ${line}"
        done <<< "${launches}"
    fi
    if [[ -n "${nodes}" ]]; then
        while IFS= read -r line; do
            [[ -z "${line}" ]] && continue
            status_cn "  node:       ${line}"
        done <<< "${nodes}"
    fi

    status_cn "正在关闭上述进程..." "INFO"
    log "Stopping robot_driver supervisor and ROS processes..."

    local targeted_output=""
    if targeted_output="$(list_supervisors_for_can "${supervisor_filter}")"; then
        while IFS=$'\t' read -r pid cmd_can cmdline; do
            [[ -z "${pid}" ]] && continue
            status_cn "  supervisor PID=${pid} (CAN=${cmd_can}) 正在关闭..."
            if terminate_supervisor_pid "${pid}" 10; then
                status_cn "  supervisor PID=${pid} 已退出" "INFO"
            else
                status_cn "  supervisor PID=${pid} 未能在超时时间内退出" "WARN"
            fi
        done <<< "${targeted_output}"
    else
        if [[ -n "${supervisor_filter}" ]]; then
            status_cn "未匹配到 CAN=${supervisor_filter} 的守护进程，可能已停止或使用了其他 CAN" "WARN"
        else
            status_cn "未匹配到任何守护进程，可能已提前退出" "WARN"
        fi
    fi

    cleanup_processes || true

    sleep 1

    local still_launch still_nodes remaining_output="" remaining_targeted=0
    still_launch="$(pgrep -fa "ros2 launch robot_driver robot_driver.launch.py" 2>/dev/null || true)"
    still_nodes="$(pgrep -fa "robot_driver/lib/robot_driver/robot_driver_node.py" 2>/dev/null || true)"
    if remaining_output="$(list_supervisors_for_can "${supervisor_filter}")"; then
        remaining_targeted=1
    fi

    if [[ ${remaining_targeted} -eq 0 && -z "${still_launch}${still_nodes}" ]]; then
        status_cn "robot_driver 相关进程已全部关闭" "INFO"
        log "robot_driver stopped."
        return 0
    fi

    status_cn "部分进程仍在运行，请手动检查：" "WARN"
    if [[ ${remaining_targeted} -eq 1 ]]; then
        while IFS=$'\t' read -r pid cmd_can cmdline; do
            [[ -z "${pid}" ]] && continue
            status_cn "  supervisor: PID=${pid} (CAN=${cmd_can}) ${cmdline}" "WARN"
        done <<< "${remaining_output}"
    fi
    if [[ -n "${still_launch}" ]]; then
        while IFS= read -r line; do
            [[ -z "${line}" ]] && continue
            status_cn "  launch:     ${line}" "WARN"
        done <<< "${still_launch}"
    fi
    if [[ -n "${still_nodes}" ]]; then
        while IFS= read -r line; do
            [[ -z "${line}" ]] && continue
            status_cn "  node:       ${line}" "WARN"
        done <<< "${still_nodes}"
    fi

    log "WARNING: some robot_driver processes may still be running; please check with ps."
    return 1
}

build_child_args() {
    CHILD_ARGS=()
    local i=0
    local total=${#ORIGINAL_ARGS[@]}
    while (( i < total )); do
        local arg="${ORIGINAL_ARGS[i]}"
        case "${arg}" in
            --daemon|--stop|--force|--follow-log|--no-follow-log|--daemon-child|--verbose|--no-exit-after-safe)
                ;;
            --safe-timeout)
                ((i+=1))
                ;;
            --safe-timeout=*)
                ;;
            *)
                CHILD_ARGS+=("${arg}")
                ;;
        esac
        ((i+=1))
    done
    # 子进程只通过 --daemon-child 标记自己为守护实例，不再支持前台模式
    CHILD_ARGS+=("--daemon-child")
}

launch_daemon_mode() {
    local existing_supervisors
    if existing_supervisors="$(list_supervisors_for_can "${CAN_CHANNEL}")"; then
        status_cn "检测到 robot_driver 守护进程已在运行 (CAN=${CAN_CHANNEL})" "WARN"
        while IFS=$'\t' read -r pid cmd_can cmdline; do
            [[ -z "${pid}" ]] && continue
            status_cn "  supervisor PID=${pid} (CAN=${cmd_can}): ${cmdline}" "WARN"
        done <<< "${existing_supervisors}"
        log "Existing supervisor detected for CAN ${CAN_CHANNEL}; aborting new launch."
        exit 0
    fi
    build_child_args
    status_cn "正在后台启动 robot_driver，日志输出：${LOG_FILE}"
    log "Starting robot_driver supervisor in daemon mode (log: ${LOG_FILE})"

    # 直接通过 nohup 后台启动子脚本，并使用 $! 精确获取守护进程 PID，
    # 避免依赖模糊的 pgrep 匹配。
    env ROBOT_DRIVER_LOG_FILE_OVERRIDE="${LOG_FILE}" \
        nohup "$0" "${CHILD_ARGS[@]}" </dev/null >/dev/null 2>&1 &
    local daemon_pid=$!
    disown "${daemon_pid}" 2>/dev/null || true

    # 固定每 1s 检查一次，最多等待 10s；10s 内守护进程仍不存在则视为启动失败。
    local waited=0
    local timeout=10
    while (( waited < timeout )); do
        if kill -0 "${daemon_pid}" >/dev/null 2>&1; then
            break
        fi
        sleep 1
        ((waited+=1))
    done
    if ! kill -0 "${daemon_pid}" >/dev/null 2>&1; then
        status_cn "后台守护进程启动失败 (PID=${daemon_pid} 在 ${timeout}s 内未存活)" "ERROR"
        log "ERROR: Daemon-child supervisor PID ${daemon_pid} is not alive ${timeout}s after start."
        exit 1
    fi

    status_cn "后台守护进程 PID=${daemon_pid} 已启动"
    # 在后台守护进程就绪后，尝试通过 SafetyPose 动作将机械臂回到安全位姿并打开零重力。
    # 启动阶段这是“软约束”：失败时仅打印提示，不影响守护进程存活。
    if ! call_safety_pose "startup" "true"; then
        status_cn "启动完成，但通过 SafetyPose 回到安全位姿失败，请检查机械臂状态和日志" "WARN"
    fi

    if [[ ${FOLLOW_LOG} -eq 1 ]]; then
        if command -v tail >/dev/null 2>&1; then
            status_cn "进入实时日志跟随模式，按 Ctrl+C 可退出（驱动持续运行）"
            tail -n +1 -F "${LOG_FILE}" || true
            status_cn "停止跟随日志，驱动仍在后台运行"
        else
            status_cn "系统缺少 tail 命令，无法实时跟随日志" "WARN"
        fi
        exit 0
    fi
    status_cn "robot_driver 已在后台运行，日志：${LOG_FILE}"
    exit 0
}

ensure_command() {
    if ! command -v "$1" >/dev/null 2>&1; then
        log "ERROR: Required command '$1' not found in PATH"
        exit 1
    fi
}

source_if_exists() {
    local file="$1"
    if [[ -f "$file" ]]; then
        set +u
        # shellcheck source=/dev/null
        source "$file"
        set -u
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
        log "ERROR: Failed to source ROS 2 environment. Expected ${ROS_SETUP} or ${INSTALL_SETUP}."
        exit 1
    fi
}

resolve_path() {
    local target="$1"
    if [[ -z "$target" ]]; then
        return 1
    fi
    if [[ "$target" = /* ]]; then
        printf '%s\n' "$target"
    else
        python3 -c 'import os,sys; print(os.path.abspath(sys.argv[1]))' "$target"
    fi
}

ensure_value_present() {
    local option="$1"
    local remaining="$2"
    if (( remaining < 2 )); then
        log "ERROR: Missing value for ${option}"
        exit 1
    fi
}

prepend_pythonpath() {
    local new_entry="$1"
    if [[ -z "${PYTHONPATH:-}" ]]; then
        export PYTHONPATH="${new_entry}"
    else
        export PYTHONPATH="${new_entry}:${PYTHONPATH}"
    fi
}

call_safety_pose() {
    local stage="$1"       # "startup" or "shutdown"
    local enable_zero="$2" # "true" or "false"

    status_cn "通过 SafetyPose 动作让机械臂回到安全位姿（阶段：${stage}，enable_zero_gravity_after=${enable_zero}）"

    local exit_code=0
    set +e
    (
        # 子 shell：加载 ROS 环境后调用 SafetyPose action client。
        # 若加载失败或动作执行失败，仅影响本子 shell 的退出码，方便外层根据
        # --force 决定是否继续强制关闭。
        source_ros_env
        SAFETY_POSE_STAGE="${stage}" SAFETY_POSE_ENABLE_ZERO_GRAVITY_AFTER="${enable_zero}" \
            python3 - <<'PY'
import math
import os
import sys
import traceback

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from robot_driver.action import SafetyPose


SERVER_TIMEOUT = 60.0
RESULT_TIMEOUT = 120.0


def _parse_bool(value, default=False):
    if value is None:
        return default
    value = str(value).strip().lower()
    if value in ('1', 'true', 'yes', 'y', 'on'):
        return True
    if value in ('0', 'false', 'no', 'n', 'off'):
        return False
    return default


class SafetyPoseClient(Node):
    def __init__(self, stage, enable_zero_gravity_after):
        super().__init__('safety_pose_client')
        self._stage = stage
        self._enable_zero_gravity_after = bool(enable_zero_gravity_after)
        self._client = ActionClient(self, SafetyPose, '/robot_driver/action/safety_pose')

    def run(self):
        stage = self._stage
        enable_zero = self._enable_zero_gravity_after
        print(
            f"[SafetyPose] stage={stage} enable_zero_gravity_after={enable_zero}",
            flush=True,
        )

        self.get_logger().info(
            f"SafetyPose client ({stage}): waiting for action server..."
        )
        if not self._client.wait_for_server(timeout_sec=SERVER_TIMEOUT):
            msg = (
                f"SafetyPose server '/robot_driver/action/safety_pose' not available "
                f"within {SERVER_TIMEOUT:.1f}s"
            )
            print(f"[SafetyPose] ERROR: {msg}", file=sys.stderr, flush=True)
            return 1

        goal_msg = SafetyPose.Goal()
        goal_msg.enable_zero_gravity_after = enable_zero

        self.get_logger().info(
            f"SafetyPose client ({stage}): sending goal enable_zero_gravity_after={enable_zero}"
        )
        send_future = self._client.send_goal_async(goal_msg)

        try:
            rclpy.spin_until_future_complete(self, send_future, timeout_sec=RESULT_TIMEOUT)
        except Exception:
            print("[SafetyPose] ERROR: exception while waiting for goal acceptance", file=sys.stderr)
            traceback.print_exc()
            return 1

        if not send_future.done():
            msg = (
                f"Timed out waiting for SafetyPose goal acceptance "
                f"after {RESULT_TIMEOUT:.1f}s"
            )
            print(f"[SafetyPose] ERROR: {msg}", file=sys.stderr, flush=True)
            return 1

        goal_handle = send_future.result()
        if goal_handle is None or not getattr(goal_handle, 'accepted', False):
            print("[SafetyPose] ERROR: goal rejected by server", file=sys.stderr, flush=True)
            return 1

        result_future = goal_handle.get_result_async()
        try:
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=RESULT_TIMEOUT)
        except Exception:
            print("[SafetyPose] ERROR: exception while waiting for result", file=sys.stderr)
            traceback.print_exc()
            return 1

        if not result_future.done():
            msg = (
                f"Timed out waiting for SafetyPose result "
                f"after {RESULT_TIMEOUT:.1f}s"
            )
            print(f"[SafetyPose] ERROR: {msg}", file=sys.stderr, flush=True)
            return 1

        action_result = result_future.result()
        result_msg = getattr(action_result, 'result', None)
        if result_msg is None:
            print("[SafetyPose] ERROR: result message missing", file=sys.stderr, flush=True)
            return 1

        max_error = getattr(result_msg, 'max_error', float('nan'))
        try:
            max_error_val = float(max_error)
            if math.isfinite(max_error_val):
                max_error_str = f"{max_error_val:.6f}"
            else:
                max_error_str = "nan"
        except Exception:
            max_error_str = "nan"

        result_code = getattr(result_msg, 'result_code', '')
        last_error = getattr(result_msg, 'last_error', '')

        print(
            "[SafetyPose] DONE: success=%s result_code=%s max_error=%s last_error=%s"
            % (
                bool(getattr(result_msg, 'success', False)),
                result_code or "''",
                max_error_str,
                last_error or "''",
            ),
            flush=True,
        )

        return 0 if bool(getattr(result_msg, 'success', False)) else 1


def main():
    stage = os.environ.get('SAFETY_POSE_STAGE', 'startup')
    enable_str = os.environ.get('SAFETY_POSE_ENABLE_ZERO_GRAVITY_AFTER', 'true')
    enable_zero = _parse_bool(enable_str, default=True)

    try:
        rclpy.init(args=None)
    except Exception:
        print("[SafetyPose] ERROR: failed to initialize rclpy", file=sys.stderr)
        traceback.print_exc()
        return 1

    node = SafetyPoseClient(stage, enable_zero)
    try:
        return node.run()
    except KeyboardInterrupt:
        print("[SafetyPose] interrupted by user", file=sys.stderr)
        return 1
    except Exception:
        print("[SafetyPose] ERROR: unexpected exception in client", file=sys.stderr)
        traceback.print_exc()
        return 1
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
PY
    )
    exit_code=$?
    set -e

    if [[ ${exit_code} -ne 0 ]]; then
        status_cn "通过 SafetyPose 回到安全位姿失败（阶段：${stage}）" "ERROR"
        log "ERROR: SafetyPose action failed during ${stage} stage (exit code ${exit_code})"
        return ${exit_code}
    fi

    status_cn "SafetyPose 执行完成，机械臂已在安全位姿（阶段：${stage}）"
    log "SafetyPose action succeeded during ${stage} stage"
    return 0
}

CAN_CHANNEL_PATTERN='^can[0-9]+$'

while [[ $# -gt 0 ]]; do
    case "$1" in
        --can)
            ensure_value_present "--can" "$#"
            CAN_CHANNEL="$2"
            CAN_CHANNEL_USER_SET=1
            shift 2
            ;;
        --can=*)
            CAN_CHANNEL="${1#*=}"
            CAN_CHANNEL_USER_SET=1
            shift
            ;;
        --xyz-only)
            XYZ_ONLY_MODE="true"
            shift
            ;;
        --no-xyz-only)
            XYZ_ONLY_MODE="false"
            shift
            ;;
        --params)
            ensure_value_present "--params" "$#"
            PARAM_FILE="$(resolve_path "$2")"
            shift 2
            ;;
        --params=*)
            value="${1#*=}"
            PARAM_FILE="$(resolve_path "$value")"
            shift
            ;;
        --daemon)
            # 兼容旧参数：当前版本始终以后台守护模式运行，此参数保持为 no-op
            shift
            ;;
        --follow-log)
            FOLLOW_LOG=1
            shift
            ;;
        --no-follow-log)
            FOLLOW_LOG=0
            shift
            ;;
        --verbose)
            VERBOSE_CONSOLE=1
            LOG_STDERR_MIRROR=1
            shift
            ;;
        --stop)
            STOP_ONLY=1
            shift
            ;;
        --force)
            FORCE_STOP=1
            shift
            ;;
        --daemon-child)
            DAEMON_CHILD=1
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            log "ERROR: Unknown option $1"
            usage
            exit 1
            ;;
    esac
done

if [[ ${FORCE_STOP} -eq 1 && ${STOP_ONLY} -eq 0 ]]; then
    log "ERROR: --force must be used together with --stop"
    usage
    exit 1
fi

if [[ ${STOP_ONLY} -eq 1 ]]; then
    # 停机流程：优先通过 SafetyPose 动作让机械臂回到安全位姿并关闭零重力。
    # 只有 SafetyPose 成功（或使用 --force 时允许失败）才会真正停止进程。

    # 检查是否有正在运行的相关进程；若没有，则认为已经关闭，直接返回。
    ensure_command pgrep
    supervisors="$(pgrep -fa "${SUPERVISOR_PGREP_PATTERN}" 2>/dev/null || true)"
    launches="$(pgrep -fa "ros2 launch robot_driver robot_driver.launch.py" 2>/dev/null || true)"
    nodes="$(pgrep -fa "robot_driver/lib/robot_driver/robot_driver_node.py" 2>/dev/null || true)"

    if [[ -z "${supervisors}${launches}${nodes}" ]]; then
        status_cn "未找到正在运行的 robot_driver 相关进程，认为已关闭" "INFO"
        log "No running robot_driver processes detected; skipping SafetyPose and stop."
        exit 0
    fi

    ensure_command python3

    if ! call_safety_pose "shutdown" "false"; then
        if [[ ${FORCE_STOP} -eq 0 ]]; then
            status_cn "关闭失败：SafetyPose 动作未能让机械臂安全落位，未执行关闭操作" "ERROR"
            log "ERROR: SafetyPose shutdown sequence failed; not stopping processes (no --force)."
            exit 1
        fi
        status_cn "SafetyPose 失败，但启用了 --force，将强制关闭 robot_driver 相关进程（存在安全风险，请确认机械臂状态）" "WARN"
        log "WARNING: SafetyPose shutdown failed; proceeding to force-stop due to --force."
    else
        status_cn "已通过 SafetyPose 确认机械臂回到安全位姿并关闭零重力，开始关闭进程..." "INFO"
        log "SafetyPose shutdown sequence succeeded; stopping robot_driver processes."
    fi

    if stop_daemon; then
        exit 0
    else
        exit 1
    fi
fi

if [[ ! "${CAN_CHANNEL}" =~ ${CAN_CHANNEL_PATTERN} ]]; then
    log "ERROR: Invalid CAN channel '${CAN_CHANNEL}'. Expected format can<num>."
    exit 1
fi

if [[ -n "${PARAM_FILE}" && ! -f "${PARAM_FILE}" ]]; then
    log "ERROR: Parameter file not found: ${PARAM_FILE}"
    exit 1
fi

ensure_command ros2
ensure_command python3
ensure_command pkill
ensure_command pgrep
ensure_command nohup

if [[ ${DAEMON_CHILD} -eq 0 ]]; then
    # 主入口始终以后台守护模式启动，当前进程只负责拉起守护子进程然后退出
    launch_daemon_mode
fi

status_cn "开始启动 robot_driver（CAN=${CAN_CHANNEL}，零重力默认=${ZERO_GRAVITY_DEFAULT}）"

I2RT_REPO_ROOT="$(cd -- "${WORKSPACE_ROOT}/../I2RT" 2>/dev/null && pwd || true)"
I2RT_PYTHON_ROOT=""
RESET_SCRIPT=""
if [[ -n "${I2RT_REPO_ROOT}" && -d "${I2RT_REPO_ROOT}/i2rt" ]]; then
    I2RT_PYTHON_ROOT="${I2RT_REPO_ROOT}/i2rt"
    if [[ -x "${I2RT_PYTHON_ROOT}/scripts/reset_all_can.sh" ]]; then
        RESET_SCRIPT="${I2RT_PYTHON_ROOT}/scripts/reset_all_can.sh"
    fi
else
    log "WARNING: I2RT repository not found at ${WORKSPACE_ROOT}/../I2RT"
fi

source_ros_env

if [[ -n "${I2RT_PYTHON_ROOT}" ]]; then
    prepend_pythonpath "${I2RT_PYTHON_ROOT}"
fi
if [[ -d "${PACKAGE_ROOT}/src" ]]; then
    prepend_pythonpath "${PACKAGE_ROOT}/src"
fi

log "Using workspace: ${WORKSPACE_ROOT}"
log "Log file: ${LOG_FILE}"
log "CAN channel: ${CAN_CHANNEL}"
log "XYZ-only mode: ${XYZ_ONLY_MODE}"
log "Zero-gravity default: ${ZERO_GRAVITY_DEFAULT}"
if [[ -n "${PARAM_FILE}" ]]; then
    log "Parameter file: ${PARAM_FILE}"
else
    log "Parameter file: <none>"
fi

run_can_maintenance() {
    local first_run="$1"
    local cleanup_script="${SCRIPT_DIR}/scripts/cleanup_can_interface.sh"
    if [[ "${first_run}" -eq 1 ]]; then
        if [[ -x "${cleanup_script}" ]]; then
            log "Running CAN interface cleanup for ${CAN_CHANNEL}..."
            if ! bash "${cleanup_script}" "${CAN_CHANNEL}" --force |& tee -a "${LOG_FILE}"; then
                CLEANUP_EXIT=$?
                if [[ ${CLEANUP_EXIT} -eq 2 ]]; then
                    log "WARNING: User cancelled CAN cleanup, continuing anyway"
                else
                    log "ERROR: CAN cleanup failed with exit code ${CLEANUP_EXIT}"
                    return ${CLEANUP_EXIT}
                fi
            fi
        else
            log "WARNING: CAN cleanup script not found at ${cleanup_script}, skipping cleanup"
        fi

        if [[ -n "${RESET_SCRIPT}" ]]; then
            log "Resetting CAN bus via ${RESET_SCRIPT}"
            if ! bash "${RESET_SCRIPT}" |& tee -a "${LOG_FILE}"; then
                log "WARNING: reset_all_can.sh returned non-zero exit status"
            fi
        else
            log "WARNING: reset_all_can.sh not found; skipping CAN reset"
        fi
    fi

    if python3 -c 'import importlib.util, sys; sys.exit(0 if importlib.util.find_spec("driver.hardware.can_interface_manager") else 1)' >/dev/null 2>&1; then
        log "Running CAN interface self-check"
        CAN_CHECK_CMD=(python3 -m driver.hardware.can_interface_manager --ensure --interface "${CAN_CHANNEL}")
        if [[ -n "${RESET_SCRIPT}" ]]; then
            CAN_CHECK_CMD+=(--reset-script "${RESET_SCRIPT}")
        fi
        if ! "${CAN_CHECK_CMD[@]}" |& tee -a "${LOG_FILE}"; then
            log "ERROR: CAN interface self-check failed"
            return 1
        fi
    elif [[ "${first_run}" -eq 1 ]]; then
        log "WARNING: driver.hardware.can_interface_manager module not found; skipping CAN self-check"
    fi
    return 0
}

export ROBOT_DRIVER_LOG_FILE="${LOG_FILE}"

LAUNCH_ARGS=("can_channel:=${CAN_CHANNEL}" "xyz_only_mode:=${XYZ_ONLY_MODE}" "zero_gravity_default:=${ZERO_GRAVITY_DEFAULT}")
if [[ -n "${PARAM_FILE}" ]]; then
    LAUNCH_ARGS+=("params:=${PARAM_FILE}")
fi

SHUTDOWN_REQUESTED=0
trap 'SHUTDOWN_REQUESTED=1; log "Shutdown signal received, waiting for current launch to exit..."' SIGINT SIGTERM

ATTEMPT=0
RESTART_DELAY_SUCCESS=2
RESTART_DELAY_FAILURE=5
FIRST_RUN=1

while true; do
    if [[ ${SHUTDOWN_REQUESTED} -ne 0 ]]; then
        break
    fi

    ((++ATTEMPT))
    log "=== Launch attempt #${ATTEMPT} ==="
    cleanup_processes

    if ! run_can_maintenance "${FIRST_RUN}"; then
        log "ERROR: CAN maintenance failed; retrying in ${RESTART_DELAY_FAILURE}s"
        sleep ${RESTART_DELAY_FAILURE} || true
        continue
    fi
    FIRST_RUN=0

    log "Starting ros2 launch robot_driver robot_driver.launch.py ${LAUNCH_ARGS[*]}"
    set +e
    if [[ ${VERBOSE_CONSOLE} -eq 1 ]]; then
        ros2 launch robot_driver robot_driver.launch.py "${LAUNCH_ARGS[@]}" |& tee -a "${LOG_FILE}"
    else
        ros2 launch robot_driver robot_driver.launch.py "${LAUNCH_ARGS[@]}" >> "${LOG_FILE}" 2>&1
    fi
    EXIT_CODE=${PIPESTATUS[0]}
    set -e

    if [[ ${SHUTDOWN_REQUESTED} -ne 0 ]]; then
        log "Shutdown requested; supervisor exiting after ros2 launch finished (code ${EXIT_CODE})."
        break
    fi

    if [[ ${EXIT_CODE} -eq 0 ]]; then
        log "ros2 launch exited cleanly (code 0); restarting in ${RESTART_DELAY_SUCCESS}s."
        sleep ${RESTART_DELAY_SUCCESS} || true
    else
        log "WARNING: ros2 launch exited with code ${EXIT_CODE}; restarting in ${RESTART_DELAY_FAILURE}s."
        sleep ${RESTART_DELAY_FAILURE} || true
    fi
done

log "robot_driver supervisor stopped"
