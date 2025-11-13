#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_ROOT="${SCRIPT_DIR}"
WORKSPACE_ROOT="$(cd -- "${PACKAGE_ROOT}/../.." && pwd)"
DEFAULT_PARAM_FILE="${PACKAGE_ROOT}/config/robot_driver_config.yaml"
ROS_SETUP="/opt/ros/humble/setup.bash"
INSTALL_SETUP="${WORKSPACE_ROOT}/install/setup.bash"
LOG_DIR="${WORKSPACE_ROOT}/log/robot_driver"
mkdir -p "${LOG_DIR}"
LOG_FILE="${LOG_DIR}/robot_driver_$(date +'%Y%m%d_%H%M%S').log"
touch "${LOG_FILE}"

CAN_CHANNEL="can0"
XYZ_ONLY_MODE="false"
ZERO_GRAVITY_DEFAULT="false"
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
  -h, --help             显示本帮助
USAGE
}

log() {
    local msg="$1"
    printf '[%s] %s\n' "$(date --iso-8601=seconds)" "${msg}" | tee -a "${LOG_FILE}" >&2
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
        source "$file"
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

CAN_CHANNEL_PATTERN='^can[0-9]+$'

while [[ $# -gt 0 ]]; do
    case "$1" in
        --can)
            if [[ $# -lt 2 ]]; then
                log "ERROR: Missing value for --can"
                exit 1
            fi
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
            if [[ $# -lt 2 ]]; then
                log "ERROR: Missing value for --params"
                exit 1
            fi
            PARAM_FILE="$(resolve_path "$2")"
            shift 2
            ;;
        --params=*)
            value="${1#*=}"
            PARAM_FILE="$(resolve_path "$value")"
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
    if [[ -z "${PYTHONPATH:-}" ]]; then
        export PYTHONPATH="${I2RT_PYTHON_ROOT}"
    else
        export PYTHONPATH="${I2RT_PYTHON_ROOT}:${PYTHONPATH}"
    fi
fi
if [[ -d "${PACKAGE_ROOT}/src" ]]; then
    if [[ -z "${PYTHONPATH:-}" ]]; then
        export PYTHONPATH="${PACKAGE_ROOT}/src"
    else
        export PYTHONPATH="${PACKAGE_ROOT}/src:${PYTHONPATH}"
    fi
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

log "Killing existing robot_driver processes (if any)..."
pkill -f robot_driver_node 2>/dev/null || true
pkill -f "ros2 launch driver robot_driver.launch.py" 2>/dev/null || true

if [[ -n "${RESET_SCRIPT}" ]]; then
    log "Resetting CAN bus via ${RESET_SCRIPT}"
    if ! bash "${RESET_SCRIPT}"; then
        log "WARNING: reset_all_can.sh returned non-zero exit status"
    fi
else
    log "WARNING: reset_all_can.sh not found; skipping CAN reset"
fi

if python3 -c 'import importlib.util, sys; sys.exit(0 if importlib.util.find_spec("driver.can_interface_manager") else 1)' >/dev/null 2>&1; then
    log "Running CAN interface self-check"
    CAN_CHECK_CMD=(python3 -m driver.can_interface_manager --ensure --interface "${CAN_CHANNEL}")
    if [[ -n "${RESET_SCRIPT}" ]]; then
        CAN_CHECK_CMD+=(--reset-script "${RESET_SCRIPT}")
    fi
    if ! "${CAN_CHECK_CMD[@]}" |& tee -a "${LOG_FILE}"; then
        log "ERROR: CAN interface self-check failed"
        exit 1
    fi
else
    log "WARNING: driver.can_interface_manager module not found; skipping CAN self-check"
fi

export ROBOT_DRIVER_LOG_FILE="${LOG_FILE}"

LAUNCH_ARGS=("can_channel:=${CAN_CHANNEL}" "xyz_only_mode:=${XYZ_ONLY_MODE}" "zero_gravity_default:=${ZERO_GRAVITY_DEFAULT}")
if [[ -n "${PARAM_FILE}" ]]; then
    LAUNCH_ARGS+=("params:=${PARAM_FILE}")
fi

log "Starting ros2 launch driver robot_driver.launch.py ${LAUNCH_ARGS[*]}"
set +e
ros2 launch driver robot_driver.launch.py "${LAUNCH_ARGS[@]}" |& tee -a "${LOG_FILE}"
EXIT_CODE=${PIPESTATUS[0]}
set -e

if [[ ${EXIT_CODE} -ne 0 ]]; then
    log "ERROR: ros2 launch exited with code ${EXIT_CODE}"
    exit ${EXIT_CODE}
fi

log "robot_driver launch finished successfully"
