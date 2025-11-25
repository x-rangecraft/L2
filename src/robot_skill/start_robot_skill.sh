#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
if [[ -n "${ZSH_VERSION:-}" && -f "/opt/ros/humble/setup.zsh" ]]; then
    ROS_SETUP="/opt/ros/humble/setup.zsh"
else
    ROS_SETUP="/opt/ros/humble/setup.bash"
fi
if [[ -n "${ZSH_VERSION:-}" && -f "${WORKSPACE_ROOT}/install/setup.zsh" ]]; then
    INSTALL_SETUP="${WORKSPACE_ROOT}/install/setup.zsh"
else
if [[ -n "${ZSH_VERSION:-}" && -f "${WORKSPACE_ROOT}/install/setup.zsh" ]]; then
    INSTALL_SETUP="${WORKSPACE_ROOT}/install/setup.zsh"
else
    INSTALL_SETUP="${WORKSPACE_ROOT}/install/setup.bash"
fi
fi
LOG_DIR="${WORKSPACE_ROOT}/log/robot_skill"
mkdir -p "${LOG_DIR}"
LOG_FILE="${LOG_DIR}/robot_skill_$(date +'%Y%m%d_%H%M%S').log"

usage() {
    cat <<'USAGE'
Usage: start_robot_skill.sh [--start|--stop]

Options:
  --start    启动 robot_skill 节点（默认）
  --stop     停止所有 robot_skill 相关进程
  -h --help  显示本帮助
USAGE
}

ACTION="start"
if [[ $# -gt 0 ]]; then
    case "$1" in
        --start)
            ACTION="start"
            ;;
        --stop)
            ACTION="stop"
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            usage
            exit 1
            ;;
    esac
fi

source_env() {
    local reset_nounset=0
    if [[ -o nounset ]]; then
        set +u
        reset_nounset=1
    fi
    if [[ -f "${ROS_SETUP}" ]]; then
        # shellcheck disable=SC1090
        source "${ROS_SETUP}"
    else
        echo "[robot_skill] Warning: ${ROS_SETUP} not found" >&2
    fi
    if [[ -f "${INSTALL_SETUP}" ]]; then
        # shellcheck disable=SC1090
        source "${INSTALL_SETUP}"
    else
        echo "[robot_skill] Warning: ${INSTALL_SETUP} not found" >&2
    fi
    if [[ ${reset_nounset} -eq 1 ]]; then
        set -u
    fi
}

start_node() {
    if pgrep -f "ros2 launch robot_skill" >/dev/null 2>&1; then
        echo "[robot_skill] 已检测到正在运行的 launch，跳过启动。" >&2
        exit 0
    fi
    echo "[robot_skill] 启动中，日志: ${LOG_FILE}"
    source_env
    nohup ros2 launch robot_skill robot_skill.launch.py >>"${LOG_FILE}" 2>&1 &
    local launch_pid=$!
    disown

    sleep 2
    if ! kill -0 "${launch_pid}" >/dev/null 2>&1; then
        echo "[robot_skill] 启动失败，最近日志：" >&2
        tail -n 40 "${LOG_FILE}" >&2 || true
        exit 1
    fi
    echo "[robot_skill] 已启动 (PID=${launch_pid})"
}

stop_node() {
    echo "[robot_skill] 关闭所有相关进程..."
    pkill -f "ros2 launch robot_skill" 2>/dev/null || true
    pkill -f robot_skill_node 2>/dev/null || true
    sleep 1
    if pgrep -f "ros2 launch robot_skill" >/dev/null 2>&1; then
        echo "[robot_skill] 仍检测到 launch 进程，请手动检查。" >&2
        return 1
    fi
    if pgrep -f robot_skill_node >/dev/null 2>&1; then
        echo "[robot_skill] 仍检测到节点进程，请手动检查。" >&2
        return 1
    fi
    echo "[robot_skill] 已尝试关闭所有相关进程。"
}

case "${ACTION}" in
    start)
        start_node
        ;;
    stop)
        stop_node
        ;;
esac
