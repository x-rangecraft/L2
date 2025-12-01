#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
INSTALL_SETUP="${WORKSPACE_DIR}/install/setup.bash"
LOG_DIR="${WORKSPACE_DIR}/log"
LOG_FILE="${LOG_DIR}/web_interactive_gui.log"

ROS_RUN_PATTERN="ros2 run web_interactive_gui"
NODE_EXEC_PATTERN="web_interactive_gui/lib/web_interactive_gui/web_interactive_gui"
STARTUP_MARKER="[web_gui] 启动完成"

usage() {
    cat <<'USAGE'
用法: webInteractiveGUI.sh [--start|--stop]

--start  启动 web_interactive_gui 节点 (后台运行, 日志写入 log/web_interactive_gui.log)
--stop   停止所有 web_interactive_gui 相关进程
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

collect_node_pids() {
    if ! command -v pgrep >/dev/null 2>&1; then
        ps -eo pid=,command= \
            | awk -v pattern="${NODE_EXEC_PATTERN}" 'index($0, pattern) {print $1}' \
            | sort -u
        return
    fi

    pgrep -f "${NODE_EXEC_PATTERN}" 2>/dev/null | sort -u
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

detect_host_ip() {
    local ip

    if command -v hostname >/dev/null 2>&1; then
        for ip in $(hostname -I 2>/dev/null); do
            if [[ -n "${ip}" && ${ip} != 127.* && ${ip} != 0.0.0.0 ]]; then
                echo "${ip}"
                return
            fi
        done
    fi

    if command -v ip >/dev/null 2>&1; then
        ip route get 1.1.1.1 2>/dev/null \
            | awk 'NR==1 {for (i = 1; i <= NF; ++i) if ($i == "src") {print $(i+1); exit}}'
        return
    fi

    echo "127.0.0.1"
}

print_access_hint() {
    local port="5000"

    if [[ -f "${LOG_FILE}" ]]; then
        local url
        url=$(grep -Eo 'http://[^ ]+' "${LOG_FILE}" | tail -n 1 || true)
        if [[ -n "${url:-}" && ${url} =~ :([0-9]+)$ ]]; then
            port="${BASH_REMATCH[1]}"
        fi
    fi

    local host
    host=$(detect_host_ip)
    log "浏览器访问: http://${host}:${port}"
}

stop_processes() {
    local action_msg="$1"
    local quiet_if_absent="${2:-0}"
    local -a running_pids=()
    mapfile -t running_pids < <(collect_node_pids)

    if (( ${#running_pids[@]} == 0 )); then
        if (( quiet_if_absent == 0 )); then
            log "未检测到运行中的 web_interactive_gui"
        fi
        return 0
    fi

    log "${action_msg}: 检测到 ${#running_pids[@]} 个进程 (PID: ${running_pids[*]})"
    kill "${running_pids[@]}" 2>/dev/null || true
    pkill -f "${ROS_RUN_PATTERN}" 2>/dev/null || true

    for _ in {1..20}; do
        sleep 0.5
        mapfile -t running_pids < <(collect_node_pids)
        if (( ${#running_pids[@]} == 0 )); then
            log "web_interactive_gui 已停止"
            return 0
        fi
    done

    log "进程仍在运行，发送 SIGKILL"
    kill -9 "${running_pids[@]}" 2>/dev/null || true
    pkill -9 -f "${ROS_RUN_PATTERN}" 2>/dev/null || true
    sleep 0.5
    mapfile -t running_pids < <(collect_node_pids)
    if (( ${#running_pids[@]} == 0 )); then
        log "web_interactive_gui 已强制停止"
        return 0
    fi

    log "警告: 仍检测到 web_interactive_gui 进程 (PID: ${running_pids[*]})"
    return 1
}

ensure_single_instance() {
    local -a running_pids=()
    mapfile -t running_pids < <(collect_node_pids)
    local count="${#running_pids[@]}"

    if (( count > 1 )); then
        log "检测到 ${count} 个 /web_interactive_gui 节点进程 (PID: ${running_pids[*]})，执行清理后再启动"
        if stop_processes "清理残留的 web_interactive_gui 进程" 1; then
            return 0
        fi
        return 3
    elif (( count == 1 )); then
        log "web_interactive_gui 已在运行 (PID: ${running_pids[0]})，不重复启动"
        return 2
    fi

    return 0
}

start_node() {
    local status=0
    ensure_single_instance || status=$?
    if (( status == 2 )); then
        return 0
    elif (( status != 0 )); then
        exit 1
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
        local -a running_pids=()
        mapfile -t running_pids < <(collect_node_pids)
        if (( ${#running_pids[@]} == 1 )); then
            log "web_interactive_gui 启动成功 (PID: ${running_pids[0]})"
        elif (( ${#running_pids[@]} > 1 )); then
            log "web_interactive_gui 启动成功，但仍检测到 ${#running_pids[@]} 个进程 (PID: ${running_pids[*]})"
        else
            log "web_interactive_gui 启动完成，但未立即检测到 PID，请查看日志确认"
        fi
        print_access_hint
    else
        log "错误: 启动超时（60秒），请查看日志: ${LOG_FILE}"
        exit 1
    fi
}

stop_node() {
    stop_processes "正在停止 web_interactive_gui" 0
}

ACTION=""
while [[ $# -gt 0 ]]; do
    case "$1" in
        --start|start)
            ACTION="start"
            shift
            ;;
        --stop|stop)
            ACTION="stop"
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            log "未知参数: $1"
            usage
            exit 1
            ;;
    esac
done

if [[ -z "${ACTION}" ]]; then
    usage
    exit 1
fi

case "${ACTION}" in
    start)
        start_node
        ;;
    stop)
        stop_node
        ;;
esac
