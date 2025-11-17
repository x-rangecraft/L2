#!/usr/bin/env bash
# TF Publisher - 统一的 TF 发布管理工具
# 启动 robot_tf_publisher 节点，统一管理所有 TF（静态+动态）
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DYNAMIC_TF_SCRIPT="${SCRIPT_DIR}/src/dynamic_tf_publish.sh"

usage() {
  cat <<'EOF'
用法：tf_publisher.sh [模式]

模式：
  (默认) 无参数：后台启动 TF 发布节点
  --daemon / --start：后台启动 TF 发布节点（同默认）
  --foreground：前台运行，按 Ctrl+C 停止
  --stop：停止后台 TF 进程
  --status：查看 TF 进程状态
  --help：显示此帮助

功能说明：
  本工具启动 robot_tf_publisher 节点，统一管理所有 TF：
  - 静态 TF：发布固定的坐标变换（从 static_tf_config.yaml 读取）
    包括 world→base_link, world→camera_link 等
  - 动态 TF：根据 joint_states 实时计算并发布机器人各链接的变换
    包括 base_link→link_1→link_2→...→link_6

  优势：
  - 只有一个节点（robot_tf_publisher）
  - 避免 ros2 node list 出现大量 static_transform_publisher_*
  - 统一管理，便于维护

示例：
  ./tf_publisher.sh                    # 后台启动（默认）
  ./tf_publisher.sh --foreground       # 前台运行
  ./tf_publisher.sh --stop             # 停止
  ./tf_publisher.sh --status           # 查看状态

注意：
  本脚本已简化，直接调用 src/dynamic_tf_publish.sh
  如需旧的分离模式，请直接使用独立的静态TF脚本
EOF
}

# Check if dynamic TF script exists
if [[ ! -x "$DYNAMIC_TF_SCRIPT" ]]; then
  echo "[ERROR] 未找到动态TF脚本: $DYNAMIC_TF_SCRIPT" >&2
  exit 1
fi

# Parse arguments
if [[ $# -gt 0 ]]; then
  case "$1" in
    --daemon|--start)
      "$DYNAMIC_TF_SCRIPT" --daemon
      ;;
    --foreground)
      "$DYNAMIC_TF_SCRIPT" --foreground
      ;;
    --stop)
      "$DYNAMIC_TF_SCRIPT" --stop
      ;;
    --status)
      "$DYNAMIC_TF_SCRIPT" --status
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[ERROR] 未知参数：$1" >&2
      usage >&2
      exit 2
      ;;
  esac
else
  # Default: daemon (background)
  "$DYNAMIC_TF_SCRIPT" --daemon
fi
