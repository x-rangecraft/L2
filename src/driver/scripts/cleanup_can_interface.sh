#!/usr/bin/env bash
# cleanup_can_interface.sh
# 检查并清理占用 CAN 接口的进程
# 用法: cleanup_can_interface.sh <can_channel> [--force]
#
# 返回值:
#   0 - 清理成功或无需清理
#   1 - 参数错误
#   2 - 用户取消清理
#   3 - 清理失败

set -euo pipefail

CAN_CHANNEL="${1:-}"
FORCE_MODE=0

if [[ "$#" -ge 2 && "$2" == "--force" ]]; then
    FORCE_MODE=1
fi

if [[ -z "${CAN_CHANNEL}" ]]; then
    echo "ERROR: CAN channel not specified" >&2
    echo "Usage: $0 <can_channel> [--force]" >&2
    exit 1
fi

log() {
    printf '[cleanup_can] %s\n' "$1" >&2
}

warn() {
    printf '[cleanup_can][WARN] %s\n' "$1" >&2
}

# 检查占用 CAN 接口的进程
log "检查占用 ${CAN_CHANNEL} 的进程..."
CAN_USERS=$(sudo lsof 2>/dev/null | grep -i "${CAN_CHANNEL}" | awk '{print $2}' | sort -u || true)

if [[ -z "${CAN_USERS}" ]]; then
    log "${CAN_CHANNEL} 未被占用"
else
    warn "发现进程正在使用 ${CAN_CHANNEL}: ${CAN_USERS}"
    for pid in ${CAN_USERS}; do
        PROC_NAME=$(ps -p "${pid}" -o comm= 2>/dev/null || echo "unknown")
        PROC_CMD=$(ps -p "${pid}" -o args= 2>/dev/null | head -c 60 || echo "unknown")
        log "  - PID ${pid} (${PROC_NAME}): ${PROC_CMD}"
    done

    SHOULD_KILL=0
    if [[ ${FORCE_MODE} -eq 1 ]]; then
        SHOULD_KILL=1
        log "强制模式：将自动停止所有占用进程"
    else
        read -rp "检测到进程占用 ${CAN_CHANNEL}，是否停止这些进程? [y/N]: " confirm_kill
        if [[ "${confirm_kill}" =~ ^[Yy]$ ]]; then
            SHOULD_KILL=1
        fi
    fi

    if [[ ${SHOULD_KILL} -eq 1 ]]; then
        log "正在停止占用 ${CAN_CHANNEL} 的进程..."
        for pid in ${CAN_USERS}; do
            if kill -0 "${pid}" 2>/dev/null; then
                log "  终止 PID ${pid}..."
                kill -TERM "${pid}" 2>/dev/null || true
                sleep 0.5
                if kill -0 "${pid}" 2>/dev/null; then
                    log "  强制终止 PID ${pid}..."
                    kill -KILL "${pid}" 2>/dev/null || true
                fi
            fi
        done
        sleep 1
        log "${CAN_CHANNEL} 占用进程已清理"
    else
        warn "用户选择不停止占用进程"
        exit 2
    fi
fi

# 检查 I2RT SDK 相关进程
log "检查 I2RT SDK 相关进程..."
I2RT_PIDS=$(ps aux | grep -E "python.*(i2rt|stage1_device)" | grep -v grep | awk '{print $2}' || true)

if [[ -z "${I2RT_PIDS}" ]]; then
    log "未发现 I2RT 相关进程"
else
    warn "发现 I2RT Python 进程: ${I2RT_PIDS}"
    for pid in ${I2RT_PIDS}; do
        PROC_CMD=$(ps -p "${pid}" -o args= 2>/dev/null | head -c 60 || echo "unknown")
        log "  - PID ${pid}: ${PROC_CMD}..."
    done

    SHOULD_KILL_I2RT=0
    if [[ ${FORCE_MODE} -eq 1 ]]; then
        SHOULD_KILL_I2RT=1
        log "强制模式：将自动停止所有 I2RT 进程"
    else
        read -rp "检测到 I2RT 相关进程，是否停止? [y/N]: " confirm_kill_i2rt
        if [[ "${confirm_kill_i2rt}" =~ ^[Yy]$ ]]; then
            SHOULD_KILL_I2RT=1
        fi
    fi

    if [[ ${SHOULD_KILL_I2RT} -eq 1 ]]; then
        log "正在停止 I2RT 进程..."
        for pid in ${I2RT_PIDS}; do
            if kill -0 "${pid}" 2>/dev/null; then
                kill -TERM "${pid}" 2>/dev/null || true
            fi
        done
        sleep 1
        for pid in ${I2RT_PIDS}; do
            if kill -0 "${pid}" 2>/dev/null; then
                log "  强制终止 PID ${pid}..."
                kill -KILL "${pid}" 2>/dev/null || true
            fi
        done
        log "I2RT 进程已清理"
    else
        warn "用户选择不停止 I2RT 进程"
    fi
fi

log "CAN 接口清理完成"
exit 0
