#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
INSTALL_SETUP="${WORKSPACE_DIR}/install/setup.bash"
LOG_DIR="${WORKSPACE_DIR}/log"
LOG_FILE="${LOG_DIR}/web_interactive_gui.log"
PID_FILE="${LOG_DIR}/web_interactive_gui.pid"

usage() {
    cat <<'USAGE'
用法: webInteractiveGUI.sh {start|stop}

start  启动 web_interactive_gui ROS 节点 (后台运行, 日志写入 log/web_interactive_gui.log)
stop   停止已启动的 web_interactive_gui 节点
USAGE
}

ensure_env() {
    if [[ ! -f "${ROS_SETUP}" ]]; then
        echo "未找到 ROS 环境: ${ROS_SETUP}" >&2
        exit 1
    fi
    if [[ ! -f "${INSTALL_SETUP}" ]]; then
        cat >&2 <<EOF
未找到工作区安装环境: ${INSTALL_SETUP}
请先在 ${WORKSPACE_DIR} 运行 colcon build (例如 ./start_robot.sh)
EOF
        exit 1
    fi
    mkdir -p "${LOG_DIR}"
}

is_running() {
    if [[ -f "${PID_FILE}" ]]; then
        local pid
        pid="$(<"${PID_FILE}")"
        if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
            return 0
        fi
    fi
    return 1
}

start_node() {
    if is_running; then
        echo "web_interactive_gui 已在运行 (PID $(<"${PID_FILE}"))"
        exit 0
    fi

    ensure_env

    echo "[$(date --iso-8601=seconds)] 启动 web_interactive_gui, 日志: ${LOG_FILE}"
    nohup bash -lc "source '${ROS_SETUP}' && source '${INSTALL_SETUP}' && exec ros2 run web_interactive_gui web_interactive_gui" \
        >>"${LOG_FILE}" 2>&1 &
    echo $! > "${PID_FILE}"
    disown
    echo "web_interactive_gui 已启动 (PID $(<"${PID_FILE}"))"
}

stop_node() {
    if ! is_running; then
        echo "未检测到运行中的 web_interactive_gui"
        [[ -f "${PID_FILE}" ]] && rm -f "${PID_FILE}"
        exit 0
    fi

    local pid
    pid="$(<"${PID_FILE}")"
    echo "正在停止 web_interactive_gui (PID ${pid})"
    if ! kill "${pid}" 2>/dev/null; then
        echo "发送 SIGTERM 失败，尝试清理 PID 文件" >&2
        rm -f "${PID_FILE}"
        exit 1
    fi

    for _ in {1..20}; do
        if ! kill -0 "${pid}" 2>/dev/null; then
            rm -f "${PID_FILE}"
            echo "web_interactive_gui 已停止"
            return
        fi
        sleep 0.5
    done

    echo "进程仍在运行，发送 SIGKILL"
    kill -9 "${pid}" 2>/dev/null || true
    rm -f "${PID_FILE}"
    echo "web_interactive_gui 已强制停止"
}

case "${1:-}" in
    start)
        start_node
        ;;
    stop)
        stop_node
        ;;
    -h|--help|help|"")
        usage
        exit 0
        ;;
    *)
        echo "未知参数: $1" >&2
        usage
        exit 1
        ;;
esac
