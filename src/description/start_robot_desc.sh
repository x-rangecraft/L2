#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
LOG_DIR="${REPO_ROOT}/log"
LOG_FILE="${LOG_DIR}/robot_desc_node.log"
ASSET_LOG_FILE="${LOG_DIR}/robot_desc_check.log"

mkdir -p "${LOG_DIR}"

print_usage() {
    cat <<'USAGE'
Usage: start_robot_desc.sh [options] [-- extra ros2 args]

Options:
  --urdf-path PATH       URDF/xacro path (supports package://). Default: package://description/urdf/yam.urdf
  --frame-prefix PREFIX  Frame prefix injected into the published URDF
  --publish-rate HZ      Publish rate in Hz (default 1.0)
  --xacro-arg KEY:=VAL   Forward xacro mappings (repeatable)
  --skip-check           Skip the asset validation step
  -h, --help             Show this help message

Anything after '--' is passed verbatim to the underlying ros2 run command.
USAGE
}

source_ros_setup() {
    local script_path="$1"
    if [[ ! -f "${script_path}" ]]; then
        return 1
    fi
    set +u
    # shellcheck disable=SC1090
    source "${script_path}"
    set -u
    return 0
}

URDF_PATH="package://description/urdf/yam.urdf"
FRAME_PREFIX=""
PUBLISH_RATE="1.0"
SKIP_CHECK=0
XACRO_ARGS=()
PASS_THROUGH=()
PYTHON_BIN="${PYTHON_BIN:-python3}"
export PYTHONPATH="${SCRIPT_DIR}/src:${PYTHONPATH:-}"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --urdf-path)
            URDF_PATH="$2"
            shift 2
            ;;
        --frame-prefix)
            FRAME_PREFIX="$2"
            shift 2
            ;;
        --publish-rate)
            PUBLISH_RATE="$2"
            shift 2
            ;;
        --xacro-arg)
            XACRO_ARGS+=("$2")
            shift 2
            ;;
        --skip-check)
            SKIP_CHECK=1
            shift
            ;;
        -h|--help)
            print_usage
            exit 0
            ;;
        --)
            shift
            PASS_THROUGH+=("$@")
            break
            ;;
        *)
            PASS_THROUGH+=("$1")
            shift
            ;;
    esac
done

printf '\n[%s] Starting robot_desc_node via start_robot_desc.sh\n' "$(date --iso-8601=seconds)" | tee -a "${LOG_FILE}"

if source_ros_setup "${REPO_ROOT}/install/setup.bash"; then
    :
elif source_ros_setup "/opt/ros/humble/setup.bash"; then
    :
else
    echo "ROS 2 environment not found (install/setup.bash or /opt/ros/humble/setup.bash)" | tee -a "${LOG_FILE}"
    exit 1
fi

if [[ ${SKIP_CHECK} -eq 0 ]]; then
    CHECK_CMD=("${PYTHON_BIN}" -m description.asset_check --urdf-path "${URDF_PATH}" --frame-prefix "${FRAME_PREFIX}" --log-file "${ASSET_LOG_FILE}")
    for arg in "${XACRO_ARGS[@]}"; do
        CHECK_CMD+=('--xacro-arg' "${arg}")
    done
    printf '[%s] Running asset validation...\n' "$(date --iso-8601=seconds)" | tee -a "${LOG_FILE}"
    if ! "${CHECK_CMD[@]}" |& tee -a "${LOG_FILE}"; then
        echo "Asset validation failed. Aborting node launch." | tee -a "${LOG_FILE}"
        exit 1
    fi
else
    printf '[%s] Asset validation skipped via --skip-check\n' "$(date --iso-8601=seconds)" | tee -a "${LOG_FILE}"
fi

ROS_PARAMS=(-p "urdf_path:=${URDF_PATH}" -p "publish_rate:=${PUBLISH_RATE}" -p "frame_prefix:=${FRAME_PREFIX}")
if [[ ${#XACRO_ARGS[@]} -gt 0 ]]; then
    XACRO_PARAM_VALUE="$(IFS=','; printf '%s' "${XACRO_ARGS[*]}")"
    ROS_PARAMS+=(-p "xacro_args:=${XACRO_PARAM_VALUE}")
fi

CMD=(ros2 run description robot_desc_node)
if [[ ${#PASS_THROUGH[@]} -gt 0 ]]; then
    CMD+=("${PASS_THROUGH[@]}")
fi
if [[ ${#ROS_PARAMS[@]} -gt 0 ]]; then
    CMD+=(--ros-args "${ROS_PARAMS[@]}")
fi

printf '[%s] Executing: %s\n' "$(date --iso-8601=seconds)" "${CMD[*]}" | tee -a "${LOG_FILE}"
"${CMD[@]}" |& tee -a "${LOG_FILE}"
