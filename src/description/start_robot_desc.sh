#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
LOG_DIR="${REPO_ROOT}/log"
LOG_FILE="${LOG_DIR}/robot_desc_node.log"
ASSET_LOG_FILE="${LOG_DIR}/robot_desc_check.log"

# Mesh HTTP server configuration
HTTP_MESH_BIND_HOST="${HTTP_MESH_BIND_HOST:-0.0.0.0}"
HTTP_MESH_ADVERTISE_HOST="${HTTP_MESH_ADVERTISE_HOST:-192.168.1.184}"
HTTP_MESH_CHECK_HOST="${HTTP_MESH_CHECK_HOST:-127.0.0.1}"
HTTP_MESH_PORT="${HTTP_MESH_PORT:-9091}"
HTTP_MESH_SOURCE_DIR="${SCRIPT_DIR}/meshes"
HTTP_MESH_STAGING_ROOT="${HTTP_MESH_STAGING_ROOT:-/tmp/robot_desc_mesh_http}"
HTTP_MESH_SERVE_DIR="${HTTP_MESH_STAGING_ROOT}/meshes"
HTTP_MESH_LOG_FILE="${LOG_DIR}/mesh_http_server.log"
HTTP_MESH_PID_FILE="${LOG_DIR}/mesh_http_server.pid"
HTTP_MESH_PROBE_FILE="${HTTP_MESH_PROBE_FILE:-base_link_visual.stl}"

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

mesh_http_probe() {
    local url="$1"
    if command -v curl >/dev/null 2>&1; then
        if curl -fsS --max-time 3 -o /dev/null "${url}"; then
            return 0
        fi
    elif command -v wget >/dev/null 2>&1; then
        if wget -q --timeout=3 -O - "${url}" >/dev/null; then
            return 0
        fi
    else
        if python3 - "${url}" <<'PY'
import sys
import urllib.request
import urllib.error

url = sys.argv[1]
try:
    with urllib.request.urlopen(url, timeout=3):
        sys.exit(0)
except Exception:
    sys.exit(1)
PY
        then
            return 0
        fi
    fi
    return 1
}

check_mesh_http_server() {
    local probe_url="http://${HTTP_MESH_CHECK_HOST}:${HTTP_MESH_PORT}/meshes/${HTTP_MESH_PROBE_FILE}"
    mesh_http_probe "${probe_url}"
}

prepare_mesh_http_payload() {
    if [[ ! -d "${HTTP_MESH_SOURCE_DIR}" ]]; then
        printf '[%s] Mesh directory not found at %s\n' "$(date --iso-8601=seconds)" "${HTTP_MESH_SOURCE_DIR}" | tee -a "${LOG_FILE}"
        return 1
    fi
    rm -rf "${HTTP_MESH_STAGING_ROOT}"
    mkdir -p "${HTTP_MESH_STAGING_ROOT}"
    if ! ln -s "${HTTP_MESH_SOURCE_DIR}" "${HTTP_MESH_SERVE_DIR}"; then
        printf '[%s] Failed to link mesh assets into %s\n' "$(date --iso-8601=seconds)" "${HTTP_MESH_SERVE_DIR}" | tee -a "${LOG_FILE}"
        return 1
    fi
    return 0
}

start_mesh_http_server() {
    if ! prepare_mesh_http_payload; then
        return 1
    fi
    printf '[%s] Starting mesh HTTP server on %s:%s (serving %s)\n' \
        "$(date --iso-8601=seconds)" "${HTTP_MESH_BIND_HOST}" "${HTTP_MESH_PORT}" "${HTTP_MESH_SERVE_DIR}" | tee -a "${LOG_FILE}"
    (
        cd "${HTTP_MESH_STAGING_ROOT}" &&
        nohup python3 -m http.server "${HTTP_MESH_PORT}" --bind "${HTTP_MESH_BIND_HOST}" \
            > "${HTTP_MESH_LOG_FILE}" 2>&1 & echo $! > "${HTTP_MESH_PID_FILE}"
    )
    local server_pid
    server_pid=$(cat "${HTTP_MESH_PID_FILE}" 2>/dev/null || true)
    sleep 2
    if check_mesh_http_server; then
        printf '[%s] Mesh HTTP server ready at http://%s:%s/meshes/\n' \
            "$(date --iso-8601=seconds)" "${HTTP_MESH_ADVERTISE_HOST}" "${HTTP_MESH_PORT}" | tee -a "${LOG_FILE}"
        return 0
    fi
    if [[ -n "${server_pid}" ]]; then
        kill "${server_pid}" 2>/dev/null || true
    fi
    printf '[%s] Mesh HTTP server failed to start (see %s)\n' "$(date --iso-8601=seconds)" "${HTTP_MESH_LOG_FILE}" | tee -a "${LOG_FILE}"
    return 1
}

ensure_mesh_http_server() {
    local public_url="http://${HTTP_MESH_ADVERTISE_HOST}:${HTTP_MESH_PORT}/meshes/"
    printf '[%s] Checking mesh HTTP resources at %s\n' "$(date --iso-8601=seconds)" "${public_url}" | tee -a "${LOG_FILE}"
    if check_mesh_http_server; then
        printf '[%s] Mesh HTTP server already available; skipping restart\n' "$(date --iso-8601=seconds)" | tee -a "${LOG_FILE}"
        return 0
    fi
    start_mesh_http_server
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

if ! ensure_mesh_http_server; then
    echo "Mesh HTTP server is not ready; aborting launch." | tee -a "${LOG_FILE}"
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

ROS_PARAMS=(-p "urdf_path:=${URDF_PATH}" -p "publish_rate:=${PUBLISH_RATE}")
if [[ -n "${FRAME_PREFIX}" ]]; then
    ROS_PARAMS+=(-p "frame_prefix:=${FRAME_PREFIX}")
fi
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
