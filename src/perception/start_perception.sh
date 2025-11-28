#!/bin/bash
#
# start_perception.sh - Perception 节点启动/停止脚本
#
# 用法:
#   ./start_perception.sh --start   # 启动节点
#   ./start_perception.sh --stop    # 停止节点
#

set -e

# 配置
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
L2_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
LOG_DIR="$L2_DIR/log/perception"
NODE_NAME="perception_node.py"
PACKAGE_NAME="perception"

# 颜色
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# 创建日志目录
mkdir -p "$LOG_DIR"

# 日志文件
DATE_STR=$(date +%Y%m%d)
STARTUP_LOG="$LOG_DIR/startup_${DATE_STR}.log"
RUNTIME_LOG="$LOG_DIR/runtime_${DATE_STR}.log"

log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] [INFO] $1" >> "$STARTUP_LOG"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] [WARN] $1" >> "$STARTUP_LOG"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] [ERROR] $1" >> "$STARTUP_LOG"
}

# 检查节点是否运行
is_running() {
    # 使用 pgrep -x 精确匹配进程名，或检查 ros2 run 进程
    pgrep -f "ros2 run.*$NODE_NAME" > /dev/null 2>&1 || \
    pgrep -f "lib/perception/$NODE_NAME" > /dev/null 2>&1
}

# 获取 PID
get_pid() {
    # 优先返回实际 Python 节点进程的 PID
    pgrep -f "lib/perception/$NODE_NAME" 2>/dev/null | head -1 || \
    pgrep -f "ros2 run.*$NODE_NAME" 2>/dev/null | head -1
}

# 启动节点
start_node() {
    log_info "启动 Perception 节点..."
    
    # 检查是否已运行
    if is_running; then
        log_warn "节点已在运行 (PID: $(get_pid))"
        return 0
    fi
    
    # Source ROS 2 环境
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
    fi
    
    # Source 工作空间
    if [ -f "$L2_DIR/install/setup.bash" ]; then
        source "$L2_DIR/install/setup.bash"
    else
        log_error "工作空间未编译: $L2_DIR/install/setup.bash 不存在"
        exit 1
    fi
    
    # 启动节点（后台运行）
    log_info "启动 ros2 run $PACKAGE_NAME $NODE_NAME"
    nohup ros2 run "$PACKAGE_NAME" "$NODE_NAME" >> "$RUNTIME_LOG" 2>&1 &
    
    # 等待节点启动
    log_info "等待节点就绪..."
    local max_wait=60
    local wait_count=0
    
    while [ $wait_count -lt $max_wait ]; do
        sleep 1
        wait_count=$((wait_count + 1))
        
        # 检查进程是否存在
        if ! is_running; then
            log_error "节点启动失败，请查看日志: $RUNTIME_LOG"
            exit 1
        fi
        
        # 尝试调用 get_status 服务
        if ros2 service call /perception/service/get_status perception/srv/GetStatus "{}" --timeout 1 2>/dev/null | grep -q "ready: true"; then
            log_info "✅ Perception 节点启动成功 (PID: $(get_pid))"
            return 0
        fi
        
        if [ $((wait_count % 10)) -eq 0 ]; then
            log_info "等待中... (${wait_count}s)"
        fi
    done
    
    log_warn "节点启动超时，但进程仍在运行 (PID: $(get_pid))"
    log_warn "可能仍在加载模型，请稍后检查"
}

# 停止节点
stop_node() {
    log_info "停止 Perception 节点..."
    
    if ! is_running; then
        log_warn "节点未运行"
        return 0
    fi
    
    # 获取所有匹配的进程 PID（ros2 run 父进程 + 实际 Python 节点）
    local pids=""
    pids="$(pgrep -f "ros2 run.*$NODE_NAME" 2>/dev/null | tr '\n' ' ')"
    pids="$pids$(pgrep -f "lib/perception/$NODE_NAME" 2>/dev/null | tr '\n' ' ')"
    log_info "发送 SIGTERM 到进程: $pids"
    
    # 向所有匹配进程发送 SIGTERM
    pkill -TERM -f "ros2 run.*$NODE_NAME" 2>/dev/null || true
    pkill -TERM -f "lib/perception/$NODE_NAME" 2>/dev/null || true
    
    # 等待进程退出
    local max_wait=10
    local wait_count=0
    
    while [ $wait_count -lt $max_wait ]; do
        sleep 1
        wait_count=$((wait_count + 1))
        
        if ! is_running; then
            log_info "✅ 节点已停止"
            return 0
        fi
    done
    
    # 强制终止所有匹配进程
    log_warn "发送 SIGKILL"
    pkill -9 -f "ros2 run.*$NODE_NAME" 2>/dev/null || true
    pkill -9 -f "lib/perception/$NODE_NAME" 2>/dev/null || true
    sleep 1
    
    if is_running; then
        log_error "无法停止节点"
        exit 1
    fi
    
    log_info "✅ 节点已强制停止"
}

# 显示状态
show_status() {
    if is_running; then
        echo -e "${GREEN}● 运行中${NC} (PID: $(get_pid))"
    else
        echo -e "${RED}○ 未运行${NC}"
    fi
}

# 显示帮助
show_help() {
    echo "用法: $0 [选项]"
    echo ""
    echo "选项:"
    echo "  --start    启动 Perception 节点"
    echo "  --stop     停止 Perception 节点"
    echo "  --status   显示节点状态"
    echo "  --help     显示此帮助"
}

# 主函数
main() {
    case "${1:-}" in
        --start)
            start_node
            ;;
        --stop)
            stop_node
            ;;
        --status)
            show_status
            ;;
        --help|-h)
            show_help
            ;;
        *)
            log_error "未知选项: $1"
            show_help
            exit 1
            ;;
    esac
}

main "$@"

