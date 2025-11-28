#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
INSTALL_SETUP="${WORKSPACE_DIR}/install/setup.bash"
LOG_DIR="${WORKSPACE_DIR}/log"
LOG_FILE="${LOG_DIR}/web_interactive_gui.log"

NODE_PATTERN="ros2 run web_interactive_gui"
STARTUP_MARKER="[web_gui] 启动完成"

usage() {
    cat <<'USAGE'
用法: webInteractiveGUI.sh {start|stop}

start  启动 web_interactive_gui ROS 节点 (后台运行, 日志写入 log/web_interactive_gui.log)
stop   停止已启动的 web_interactive_gui 节点
USAGE
}

log() {
    local msg="$1"
    local timestamp
    timestamp="$(date --iso-8601=seconds)"
    printf '[%s] %s\n' "${timestamp}" "${msg}"
}

ensure_env() {
    if [[ ! -f "${ROS_SETUP}" ]]; then
        log "错误: 未找到 ROS 环境: ${ROS_SETUP}"
        exit 1
    fi
    if [[ ! -f "${INSTALL_SETUP}" ]]; then
        log "错误: 未找到工作区安装环境: ${INSTALL_SETUP}"
        log "请先在 ${WORKSPACE_DIR} 运行 colcon build"
        exit 1
    fi
    mkdir -p "${LOG_DIR}"
}

is_running() {
    pgrep -f "${NODE_PATTERN}" > /dev/null 2>&1
}

wait_for_log_marker() {
    local pattern="$1"
    local timeout="${2:-30}"
    local waited=0
    
    while (( waited < timeout )); do
        if grep -Fq "${pattern}" "${LOG_FILE}" 2>/dev/null; then
            return 0
        fi
        sleep 1
        waited=$((waited + 1))
    done
    return 1
}

start_node() {
    if is_running; then
        log "web_interactive_gui 已在运行"
        exit 0
    fi

    ensure_env

    # 清空旧日志（便于检测新的启动标记）
    : > "${LOG_FILE}"

    log "启动 web_interactive_gui, 日志: ${LOG_FILE}"
    nohup bash -lc "source '${ROS_SETUP}' && source '${INSTALL_SETUP}' && exec ros2 run web_interactive_gui web_interactive_gui" \
        >>"${LOG_FILE}" 2>&1 &
    disown

    # 等待启动成功标记
    log "等待节点初始化..."
    if wait_for_log_marker "${STARTUP_MARKER}" 60; then
        log "web_interactive_gui 启动成功"
    else
        log "错误: 启动超时（60秒），请查看日志: ${LOG_FILE}"
        exit 1
    fi
}

stop_node() {
    if ! is_running; then
        log "未检测到运行中的 web_interactive_gui"
        exit 0
    fi

    log "正在停止 web_interactive_gui..."
    pkill -f "${NODE_PATTERN}" || true

    # 等待退出
    for _ in {1..20}; do
        if ! is_running; then
            log "web_interactive_gui 已停止"
            return
        fi
        sleep 0.5
    done

    # 强制停止
    log "进程仍在运行，发送 SIGKILL"
    pkill -9 -f "${NODE_PATTERN}" || true
    log "web_interactive_gui 已强制停止"
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
        log "未知参数: $1"
        usage
        exit 1
        ;;
esac
