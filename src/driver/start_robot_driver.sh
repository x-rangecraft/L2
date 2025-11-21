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
FOLLOW_LOG=0
LOG_STDERR_MIRROR=1
VERBOSE_CONSOLE=0
declare -a CHILD_ARGS=()

CAN_CHANNEL="can0"
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

cleanup_supervisors() {
    # Ensure there is at most one start_robot_driver.sh supervisor.
    # 只匹配真正的守护进程，不会杀掉当前这次 CLI 调用（没有 --daemon-child）。
    log "Killing existing robot_driver supervisor processes (if any)..."
    pkill -f "start_robot_driver.sh --daemon-child" 2>/dev/null || true
}

stop_daemon() {
    status_cn "正在查找需要关闭的 robot_driver 相关进程..."

    # 收集当前所有相关进程（仅用于打印，不靠这些 PID 进行精确杀死）
    local supervisors launches nodes
    supervisors="$(pgrep -fa "start_robot_driver.sh --daemon-child" 2>/dev/null || true)"
    launches="$(pgrep -fa "ros2 launch robot_driver robot_driver.launch.py" 2>/dev/null || true)"
    nodes="$(pgrep -fa "robot_driver/lib/robot_driver/robot_driver_node.py" 2>/dev/null || true)"

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

    # Best-effort kill of all daemon-child supervisors and ROS nodes.
    # 1) 先按惯用启动方式杀掉所有 start_robot_driver.sh --can ... 守护/残留进程
    pkill -f "start_robot_driver.sh --can" 2>/dev/null || true
    # 2) 再杀掉带 --daemon-child 标记的守护进程
    cleanup_supervisors || true
    # 3) 最后杀掉所有 ros2 launch / robot_driver_node 进程
    cleanup_processes || true

    # 等待一小段时间让进程退出
    sleep 1

    # 再次检查是否还有相关进程存活
    local still_sup still_launch still_nodes
    still_sup="$(pgrep -fa "start_robot_driver.sh --daemon-child" 2>/dev/null || true)"
    still_launch="$(pgrep -fa "ros2 launch robot_driver robot_driver.launch.py" 2>/dev/null || true)"
    still_nodes="$(pgrep -fa "robot_driver/lib/robot_driver/robot_driver_node.py" 2>/dev/null || true)"

    if [[ -z "${still_sup}${still_launch}${still_nodes}" ]]; then
        status_cn "robot_driver 相关进程已全部关闭" "INFO"
        log "robot_driver stopped."
        return 0
    fi

    status_cn "部分进程仍在运行，请手动检查：" "WARN"
    if [[ -n "${still_sup}" ]]; then
        while IFS= read -r line; do
            [[ -z "${line}" ]] && continue
            status_cn "  supervisor: ${line}" "WARN"
        done <<< "${still_sup}"
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
            --daemon|--stop|--follow-log|--no-follow-log|--daemon-child|--verbose|--no-exit-after-safe)
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
    # Before starting a new supervisor, proactively clean up any stray
    # daemon-child supervisors from previous runs to guarantee a single
    # supervisor and a single /robot_driver instance.
    cleanup_supervisors
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

CAN_CHANNEL_PATTERN='^can[0-9]+$'

while [[ $# -gt 0 ]]; do
    case "$1" in
        --can)
            ensure_value_present "--can" "$#"
            CAN_CHANNEL="$2"
            shift 2
            ;;
        --can=*)
            CAN_CHANNEL="${1#*=}"
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

if [[ ${STOP_ONLY} -eq 1 ]]; then
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
        sleep ${RESTART_DELAY_FAILURE}
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
        log "Shutdown requested; exiting supervisor loop (ros2 launch code ${EXIT_CODE})"
        break
    fi

    # 关闭自动重启逻辑：仅记录一次退出原因后直接退出监督循环。
    if [[ ${EXIT_CODE} -eq 0 ]]; then
        log "ros2 launch exited cleanly (code 0). Auto-restart disabled; supervisor exiting."
    else
        log "WARNING: ros2 launch exited with code ${EXIT_CODE}; auto-restart disabled; supervisor exiting."
    fi
    break
done

log "robot_driver supervisor stopped"
