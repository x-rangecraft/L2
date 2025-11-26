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

STOP_ONLY=0
LOG_STDERR_MIRROR=1
VERBOSE_CONSOLE=0

DEFAULT_CAN_CHANNEL="can0"
CAN_CHANNEL="${DEFAULT_CAN_CHANNEL}"
CAN_CHANNEL_USER_SET=0
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

    sleep 1

    # 如仍未退出，则使用 SIGKILL 强制结束
    pkill -9 -f "ros2 launch robot_driver robot_driver.launch.py" 2>/dev/null || true
    pkill -9 -f "robot_driver/lib/robot_driver/robot_driver_node.py" 2>/dev/null || true
    pkill -9 -f robot_driver_node 2>/dev/null || true
}

ros2_node_exists() {
    local nodes
    nodes="$(ros2 node list 2>/dev/null || true)"
    if [[ -z "${nodes}" ]]; then
        return 1
    fi
    if grep -q '^/robot_driver$' <<< "${nodes}"; then
        return 0
    fi
    return 1
}

ensure_no_running_node() {
    if ! command -v ros2 >/dev/null 2>&1; then
        return
    fi
    status_cn "环境检查：确认 /robot_driver 未在运行"
    if ros2_node_exists; then
        status_cn "检测到 /robot_driver 已在运行，跳过本次启动" "WARN"
        exit 0
    fi
}

wait_for_ros_node() {
    local timeout="$1"
    local waited=0
    while (( waited < timeout )); do
        if ros2_node_exists; then
            return 0
        fi
        sleep 1
        ((waited++))
    done
    return 1
}

wait_for_node_absence() {
    local timeout="$1"
    local waited=0
    while (( waited < timeout )); do
        if ! ros2_node_exists; then
            return 0
        fi
        sleep 1
        ((waited++))
    done
    return 1
}

wait_for_log_marker() {
    local pattern="$1"
    local success_msg="$2"
    local timeout="${3:-60}"
    local warn_msg="${4:-${success_msg} 超时}"
    local waited=0
    while (( waited < timeout )); do
        if grep -Fq "${pattern}" "${LOG_FILE}" 2>/dev/null; then
            status_cn "${success_msg}"
            return 0
        fi
        sleep 1
        waited=$((waited + 1))
    done
    status_cn "${warn_msg}" "WARN"
    return 1
}

report_startup_sequence() {
    status_cn "节点 /robot_driver 启动检测中..."
    if wait_for_ros_node 30; then
        status_cn "节点 /robot_driver 启动成功"
    else
        status_cn "节点 /robot_driver 未在 30s 内出现" "WARN"
    fi

    # 第一阶段（Base）：硬件连接
    wait_for_log_marker "[robot_start] 基础启动完成" "第一阶段完成：基础启动" 120 "硬件连接超时，请检查 CAN 连接"

    # 第二阶段（Ext）：安全位姿
    wait_for_log_marker "[robot_start] 安全位姿复位成功" "已回到安全位姿" 180 "安全位姿复位超时，请查看日志"

    # 第二阶段（Ext）：零重力 + 启动结束
    wait_for_log_marker "[robot_start] 启动结束" "启动流程全部完成" 30 "启动未完成，请查看日志"
}

stop_daemon() {
    status_cn "正在查找需要关闭的 robot_driver 相关进程..."

    local service_attempted=0
    local service_success=0
    if command -v ros2 >/dev/null 2>&1 && ros2_node_exists; then
        service_attempted=1
        status_cn "尝试通过 /robot_driver/service/shutdown 服务关闭节点..."
        status_cn "等待安全位姿复位完成（最长 180 秒）..."
        local service_output=""
        # 使用 timeout 命令包装 ros2 service call，ROS2 Humble 不支持 --timeout 参数
        if service_output=$(timeout 180 ros2 service call /robot_driver/service/shutdown std_srvs/srv/Trigger '{}' 2>&1); then
            status_cn "shutdown 服务调用成功，等待节点退出..."
            if wait_for_node_absence 120; then
                service_success=1
            else
                status_cn "服务调用已完成，但节点仍在运行，准备改用进程级别停止" "WARN"
            fi
        else
            local exit_code=$?
            if [[ ${exit_code} -eq 124 ]]; then
                status_cn "shutdown 服务调用超时（180秒），改用进程级别停止" "WARN"
            else
                status_cn "shutdown 服务调用失败，改用进程级别停止" "WARN"
            fi
            log "Shutdown service call failed (exit=${exit_code}): ${service_output}"
        fi
    fi

    if [[ ${service_attempted} -eq 1 && ${service_success} -eq 1 ]]; then
        status_cn "通过服务接口关闭 robot_driver 成功" "INFO"
        return 0
    fi

    local launches nodes
    launches="$(pgrep -fa "ros2 launch robot_driver robot_driver.launch.py" 2>/dev/null || true)"
    nodes="$(pgrep -fa "robot_driver/lib/robot_driver/robot_driver_node.py" 2>/dev/null || true)"

    if [[ -z "${launches}${nodes}" ]]; then
        status_cn "未找到正在运行的 robot_driver 相关进程" "WARN"
        log "No robot_driver nodes found to stop."
        return 0
    fi

    status_cn "服务关闭失败，进入进程级别清理：" "WARN"
    if [[ -n "${launches}" ]]; then
        while IFS= read -r line; do
            [[ -z "${line}" ]] && continue
            status_cn "  仍在运行的 launch: ${line}"
        done <<< "${launches}"
    fi
    if [[ -n "${nodes}" ]]; then
        while IFS= read -r line; do
            [[ -z "${line}" ]] && continue
            status_cn "  仍在运行的节点: ${line}"
        done <<< "${nodes}"
    fi

    status_cn "正在关闭上述进程..." "INFO"
    log "Stopping robot_driver ROS processes..."

    cleanup_processes || true

    sleep 1

    local still_launch still_nodes
    still_launch="$(pgrep -fa "ros2 launch robot_driver robot_driver.launch.py" 2>/dev/null || true)"
    still_nodes="$(pgrep -fa "robot_driver/lib/robot_driver/robot_driver_node.py" 2>/dev/null || true)"

    if [[ -z "${still_launch}${still_nodes}" ]]; then
        status_cn "robot_driver 相关进程已全部关闭" "INFO"
        log "robot_driver stopped."
        return 0
    fi

    status_cn "部分进程仍在运行，请手动检查：" "WARN"
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
        --verbose)
            VERBOSE_CONSOLE=1
            LOG_STDERR_MIRROR=1
            shift
            ;;
        --stop)
            STOP_ONLY=1
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
    ensure_command ros2
    source_ros_env
    if stop_daemon; then
        exit 0
    fi
    exit 1
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

source_ros_env
ensure_no_running_node

status_cn "开始启动 robot_driver（CAN=${CAN_CHANNEL}，零重力默认=${ZERO_GRAVITY_DEFAULT}）"

I2RT_REPO_ROOT="$(cd -- "${WORKSPACE_ROOT}/../I2RT" 2>/dev/null && pwd || true)"
I2RT_PYTHON_ROOT=""
if [[ -n "${I2RT_REPO_ROOT}" && -d "${I2RT_REPO_ROOT}/i2rt" ]]; then
    I2RT_PYTHON_ROOT="${I2RT_REPO_ROOT}/i2rt"
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

export ROBOT_DRIVER_LOG_FILE="${LOG_FILE}"

LAUNCH_ARGS=("can_channel:=${CAN_CHANNEL}" "xyz_only_mode:=${XYZ_ONLY_MODE}" "zero_gravity_default:=${ZERO_GRAVITY_DEFAULT}")
if [[ -n "${PARAM_FILE}" ]]; then
    LAUNCH_ARGS+=("params:=${PARAM_FILE}")
fi

cleanup_processes

# 后台启动 ros2 launch，主脚本等待启动完成
status_cn "启动 /robot_driver 节点，日志：${LOG_FILE}"
log "Starting ros2 launch robot_driver robot_driver.launch.py ${LAUNCH_ARGS[*]}"

if [[ ${VERBOSE_CONSOLE} -eq 1 ]]; then
    ros2 launch robot_driver robot_driver.launch.py "${LAUNCH_ARGS[@]}" 2>&1 | tee -a "${LOG_FILE}" &
else
    ros2 launch robot_driver robot_driver.launch.py "${LAUNCH_ARGS[@]}" >> "${LOG_FILE}" 2>&1 &
fi
LAUNCH_PID=$!

# 等待启动序列完成
report_startup_sequence

# 检查 launch 进程是否还在运行
if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    status_cn "ros2 launch 进程已退出，请检查日志" "ERROR"
    exit 1
fi

status_cn "robot_driver 启动完成"
status_cn "日志文件：${LOG_FILE}"

exit 0
