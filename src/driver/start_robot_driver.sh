#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_ROOT="${SCRIPT_DIR}"
WORKSPACE_ROOT="$(cd -- "${PACKAGE_ROOT}/../.." && pwd)"
DEFAULT_PARAM_FILE="${PACKAGE_ROOT}/config/robot_driver_config.yaml"
# Prefer shell-matching setup script if available (avoids zsh sourcing bash issues),
# fallback to standard bash setup.
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
PID_FILE="${LOG_DIR}/robot_driver.pid"

ORIGINAL_ARGS=("$@")
DAEMON_MODE="daemon"
DAEMON_CHILD=0
STOP_ONLY=0
FOLLOW_LOG=0
# 镜像日志到标准错误，方便直接看到提示。后台子进程会自动关闭该开关。
LOG_STDERR_MIRROR=1
VERBOSE_CONSOLE=0
declare -a CHILD_ARGS=()
EXIT_AFTER_SAFE_POSE=1
SAFE_POSE_WAIT_TIMEOUT=120

CAN_CHANNEL="can0"
XYZ_ONLY_MODE="false"
# 默认在落位 SAFE_POSE 后自动进入零重力模式
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
  --zero-gravity         启动时默认进入零重力
  --no-zero-gravity      启动时默认关闭零重力（默认）
  --params <file>        指定参数 YAML（默认 driver/config/robot_driver_config.yaml，若存在）
  --daemon               后台守护运行（默认），命令返回后守护进程继续运行
  --foreground           前台运行，行为与旧版本相同
  --stop                 停止后台守护进程
  --follow-log           后台模式下跟随日志输出
  --no-follow-log        后台模式下不跟随日志（默认）
  --verbose              控制台打印更多启动进度 / 状态
  --no-exit-after-safe   前台模式下，不在安全位姿落位后退出脚本
  --safe-timeout <sec>   等待安全位姿落位的超时时间（默认 120s，前台模式有效）
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

read_pid_file() {
    if [[ -f "${PID_FILE}" ]]; then
        local pid
        pid="$(tr -d '[:space:]' < "${PID_FILE}")"
        if [[ "${pid}" =~ ^[0-9]+$ ]]; then
            printf '%s\n' "${pid}"
            return 0
        fi
    fi
    return 1
}

write_pid_file() {
    printf '%s\n' "$$" > "${PID_FILE}"
}

cleanup_pid_file_on_exit() {
    if [[ -f "${PID_FILE}" ]]; then
        local pid
        pid="$(tr -d '[:space:]' < "${PID_FILE}")"
        if [[ "${pid}" = "$$" ]]; then
            rm -f "${PID_FILE}"
        fi
    fi
}

ensure_no_active_daemon() {
    local existing_pid
    if existing_pid="$(read_pid_file)"; then
        if kill -0 "${existing_pid}" >/dev/null 2>&1; then
            log "robot_driver supervisor already running as PID ${existing_pid}; use --stop before starting a new instance."
            exit 0
        fi
        log "Removing stale PID file ${PID_FILE} (PID ${existing_pid} not running)"
        rm -f "${PID_FILE}"
    fi
}

wait_for_pid_file() {
    local timeout=${1:-15}
    local end=$((SECONDS + timeout))
    while (( SECONDS < end )); do
        local pid
        if pid="$(read_pid_file)"; then
            printf '%s\n' "${pid}"
            return 0
        fi
        sleep 0.2
    done
    return 1
}

cleanup_processes() {
    log "Killing existing robot_driver processes (if any)..."
    pkill -f robot_driver_node 2>/dev/null || true
    pkill -f "ros2 launch robot_driver robot_driver.launch.py" 2>/dev/null || true
}

safe_shutdown_before_kill() {
    # Best-effort: request SAFE_POSE via /robot_driver/safety_stop before stopping supervisor.
    # 如果任一步失败（无 PID、ROS 环境不可用、缺少 ros2 等），仅记录日志并继续原有关机流程。
    local pid
    if ! pid="$(read_pid_file)"; then
        log "safe_shutdown: PID file ${PID_FILE} not found, skip SAFE_POSE request."
        return 0
    fi
    if ! kill -0 "${pid}" >/dev/null 2>&1; then
        log "safe_shutdown: PID ${pid} not running, skip SAFE_POSE request."
        return 0
    fi

    # 尝试加载 ROS 环境；失败则不影响后续 stop 行为。
    if ! try_source_ros_env >/dev/null 2>&1; then
        log "safe_shutdown: Failed to source ROS 2 env, skipping SAFE_POSE request."
        return 0
    fi

    if ! command -v ros2 >/dev/null 2>&1; then
        log "safe_shutdown: 'ros2' CLI not found, skipping SAFE_POSE request."
        return 0
    fi

    status_cn "停止 robot_driver 前：先通过 /robot_driver/safety_stop 请求回到 SAFE_POSE" "INFO"
    if ! ros2 topic pub /robot_driver/safety_stop std_msgs/msg/Empty "{}" --once >/dev/null 2>&1; then
        log "safe_shutdown: Failed to publish /robot_driver/safety_stop, continuing with shutdown."
        return 0
    fi

    # 等待一小段时间，让 SAFE_POSE 序列有机会完成（内部已带 wait_time 与校验）。
    local wait_sec=10
    log "safe_shutdown: Waiting ${wait_sec}s for SAFE_POSE sequence before stopping supervisor PID ${pid}"
    sleep "${wait_sec}"
    return 0
}

stop_daemon() {
    local pid
    if ! pid="$(read_pid_file)"; then
        log "robot_driver supervisor is not running (PID file ${PID_FILE} not found)."
        return 0
    fi
    if ! kill -0 "${pid}" >/dev/null 2>&1; then
        log "Stale PID file for PID ${pid}; removing."
        rm -f "${PID_FILE}"
        return 0
    fi
    local pgid
    pgid="$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d '[:space:]')" || pgid=""
    local kill_target="${pid}"
    if [[ -n "${pgid}" && "${pgid}" =~ ^[0-9]+$ ]]; then
        kill_target="-${pgid}"
        log "Stopping robot_driver supervisor PID ${pid} (process group ${pgid})..."
    else
        log "Stopping robot_driver supervisor PID ${pid}..."
    fi
    kill "${kill_target}" >/dev/null 2>&1 || true
    local waited=0
    local timeout=15
    local hard_kill_sent=0
    while kill -0 "${pid}" >/dev/null 2>&1; do
        if (( waited >= timeout )); then
            if (( hard_kill_sent == 0 )); then
                log "PID ${pid} still running after ${timeout}s; sending SIGKILL."
                kill -9 "${kill_target}" >/dev/null 2>&1 || true
                hard_kill_sent=1
                waited=0
                timeout=5
                continue
            fi
            log "ERROR: Failed to stop PID ${pid}; please kill it manually."
            return 1
        fi
        sleep 1
        ((waited+=1))
    done
    cleanup_processes || true
    rm -f "${PID_FILE}"
    log "robot_driver supervisor stopped."
    return 0
}

build_child_args() {
    CHILD_ARGS=()
    local i=0
    local total=${#ORIGINAL_ARGS[@]}
    while (( i < total )); do
        local arg="${ORIGINAL_ARGS[i]}"
        case "${arg}" in
            --daemon|--no-daemon|--foreground|--stop|--follow-log|--no-follow-log|--daemon-child|--verbose|--no-exit-after-safe)
                ;;
            --safe-timeout)
                ((i+=1))  # skip value
                ;;
            --safe-timeout=*)
                ;;
            *)
                CHILD_ARGS+=("${arg}")
                ;;
        esac
        ((i+=1))
    done
    CHILD_ARGS+=("--daemon-child" "--foreground")
}

monitor_startup_events() {
    local log_file="$1"
    local timeout="$2"
    local fifo
    fifo="$(mktemp -u)"
    mkfifo "${fifo}"
    python3 - "${log_file}" "${timeout}" <<'PY' > "${fifo}" &
import os
import sys
import time

log_path = sys.argv[1]
timeout = float(sys.argv[2])
deadline = time.time() + timeout
os.makedirs(os.path.dirname(log_path), exist_ok=True)
open(log_path, 'a').close()

ros_started = False
hardware_ready = False

with open(log_path, 'r', encoding='utf-8', errors='ignore') as f:
    f.seek(0, os.SEEK_SET)
    while True:
        if time.time() > deadline:
            print('EVENT:TIMEOUT', flush=True)
            sys.exit(1)
        pos = f.tell()
        line = f.readline()
        if not line:
            time.sleep(0.2)
            f.seek(pos)
            continue
        if 'process started with pid' in line and 'robot_driver_node' in line and not ros_started:
            ros_started = True
            print('EVENT:ROS_PROCESS', flush=True)
        if 'Hardware connected via' in line and not hardware_ready:
            hardware_ready = True
            print('EVENT:HARDWARE', flush=True)
        if 'SAFE_POSE not marked ready' in line or 'SAFE_POSE unavailable' in line:
            print('EVENT:SAFE_POSE_SKIP', flush=True)
            sys.exit(3)
        if 'Moved to SAFE_POSE' in line:
            print('EVENT:SAFE_POSE_DONE', flush=True)
            sys.exit(0)
        if 'Failed to connect hardware' in line:
            print('EVENT:CONNECT_FAIL', flush=True)
            sys.exit(2)
        if 'Traceback' in line or 'ERROR' in line:
            print('EVENT:ERROR:' + line.strip(), flush=True)
PY
    local monitor_pid=$!
    local line
    while IFS= read -r line; do
        case "${line}" in
            EVENT:ROS_PROCESS)
                status_cn "ROS 节点进程已启动"
                ;;
            EVENT:HARDWARE)
                status_cn "硬件通信已建立，准备落位安全姿态"
                ;;
            EVENT:SAFE_POSE_DONE)
                status_cn "安全位姿已到位，默认切换至零重力模式并保持后台运行"
                ;;
            EVENT:SAFE_POSE_SKIP)
                status_cn "SAFE_POSE 未标记为 ready，驱动跳过落位，请检查配置" "WARN"
                ;;
            EVENT:CONNECT_FAIL)
                status_cn "硬件连接失败，查看日志 ${LOG_FILE}" "WARN"
                ;;
            EVENT:TIMEOUT)
                status_cn "等待安全位姿超时（${SAFE_POSE_WAIT_TIMEOUT}s），请检查日志 ${LOG_FILE}" "WARN"
                ;;
            EVENT:ERROR:*)
                status_cn "${line#EVENT:ERROR:}" "WARN"
                ;;
        esac
    done < "${fifo}"
    wait "${monitor_pid}" || true
    local rc=$?
    rm -f "${fifo}"
    return "${rc}"
}

launch_daemon_mode() {
    ensure_no_active_daemon
    build_child_args
    status_cn "正在后台启动 robot_driver，日志输出：${LOG_FILE}"
    log "Starting robot_driver supervisor in daemon mode (log: ${LOG_FILE})"
    if command -v setsid >/dev/null 2>&1; then
        env ROBOT_DRIVER_LOG_FILE_OVERRIDE="${LOG_FILE}" \
            setsid nohup "$0" "${CHILD_ARGS[@]}" </dev/null >/dev/null 2>&1 &
    else
        log "WARNING: 'setsid' command not found; daemon will remain in current session."
        env ROBOT_DRIVER_LOG_FILE_OVERRIDE="${LOG_FILE}" \
            nohup "$0" "${CHILD_ARGS[@]}" </dev/null >/dev/null 2>&1 &
    fi
    local launcher_pid=$!
    disown "${launcher_pid}" 2>/dev/null || true
    local daemon_pid
    if daemon_pid="$(wait_for_pid_file 10)"; then
        status_cn "后台守护进程 PID=${daemon_pid} 已启动"
    else
        status_cn "后台守护进程 PID 暂不可见，请手动检查" "WARN"
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

    status_cn "等待硬件连接与安全位姿落位（超时 ${SAFE_POSE_WAIT_TIMEOUT}s）"
    if monitor_startup_events "${LOG_FILE}" "${SAFE_POSE_WAIT_TIMEOUT}"; then
        status_cn "启动流程完成，脚本退出，后台驱动持续运行"
        if [[ ${EXIT_AFTER_SAFE_POSE} -eq 1 ]]; then
            exit 0
        fi
    else
        local monitor_rc=$?
        case "${monitor_rc}" in
            1)
                status_cn "未在超时时间内检测到安全位姿，请检查日志 ${LOG_FILE}" "WARN"
                ;;
            2)
                status_cn "硬件连接失败，请检查电源/CAN 接口，详见 ${LOG_FILE}" "WARN"
                ;;
            3)
                status_cn "SAFE_POSE 未就绪，驱动跳过落位，请确认 YAML 已标记 ready" "WARN"
                ;;
            *)
                status_cn "启动监控异常 (code=${monitor_rc})，请查看 ${LOG_FILE}" "WARN"
                ;;
        esac
        if [[ ${EXIT_AFTER_SAFE_POSE} -eq 1 ]]; then
            exit "${monitor_rc}"
        fi
    fi

    status_cn "继续输出日志，按 Ctrl+C 结束查看（驱动仍在后台运行）"
    tail -n +1 -F "${LOG_FILE}" || true
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
        # shellcheck source=/dev/null
        # Temporarily disable -u to allow ROS setup scripts to reference undefined variables
        set +u
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

try_source_ros_env() {
    # 与 source_ros_env 类似，但失败时只打 WARNING，不退出脚本；用于停机安全路径。
    local sourced=0
    if source_if_exists "${ROS_SETUP}"; then
        sourced=1
    fi
    if source_if_exists "${INSTALL_SETUP}"; then
        sourced=1
    fi
    if [[ ${sourced} -eq 0 ]]; then
        log "WARNING: Failed to source ROS 2 environment; skipping ROS-based SAFE_POSE request."
        return 1
    fi
    return 0
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
        --zero-gravity)
            ZERO_GRAVITY_DEFAULT="true"
            shift
            ;;
        --no-zero-gravity)
            ZERO_GRAVITY_DEFAULT="false"
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
            DAEMON_MODE="daemon"
            shift
            ;;
        --no-daemon|--foreground)
            DAEMON_MODE="foreground"
            shift
            ;;
        --no-exit-after-safe)
            EXIT_AFTER_SAFE_POSE=0
            shift
            ;;
        --safe-timeout)
            ensure_value_present "--safe-timeout" "$#"
            SAFE_POSE_WAIT_TIMEOUT="$2"
            shift 2
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

# 只有父进程需要镜像到终端；子进程专注写日志文件，避免重复输出
if [[ ${DAEMON_CHILD} -eq 1 ]]; then
    LOG_STDERR_MIRROR=0
fi

if [[ ${STOP_ONLY} -eq 1 ]]; then
    # 在真正停止 supervisor 前，先向运行中的 robot_driver 节点发送一次 /robot_driver/safety_stop，
    # 尝试回到 SAFE_POSE，避免悬停姿态下直接断控导致“掉下去”。
    safe_shutdown_before_kill || true
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

if [[ ${DAEMON_CHILD} -eq 0 && "${DAEMON_MODE}" = "daemon" ]]; then
    launch_daemon_mode
fi

ensure_no_active_daemon
trap cleanup_pid_file_on_exit EXIT
write_pid_file

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
log "Zero-gravity default: ${ZERO_GRAVITY_DEFAULT} (SAFE_POSE 完成后自动启用)"
log "Verbose console: ${VERBOSE_CONSOLE}"
if [[ -n "${PARAM_FILE}" ]]; then
    log "Parameter file: ${PARAM_FILE}"
else
    log "Parameter file: <none>"
fi
log "启动入口：./src/driver/start_robot_driver.sh --can can0（按需替换 can 通道）；若需要实时输出可追加 --follow-log，默认改写日志至 log/robot_driver/robot_driver_<timestamp>.log"

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
# 默认使用内置配置文件，除非显式传入 --params 覆盖
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

    if [[ ${EXIT_CODE} -eq 0 ]]; then
        log "ros2 launch exited cleanly (code 0). Restarting in ${RESTART_DELAY_SUCCESS}s."
        sleep ${RESTART_DELAY_SUCCESS}
    else
        log "WARNING: ros2 launch exited with code ${EXIT_CODE}; restarting in ${RESTART_DELAY_FAILURE}s"
        sleep ${RESTART_DELAY_FAILURE}
    fi
done

log "robot_driver supervisor stopped"
