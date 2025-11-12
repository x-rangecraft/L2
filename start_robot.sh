#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="${SCRIPT_DIR}/log"
LOG_FILE="${LOG_DIR}/start_robot.log"
SETUP_FILE="${SCRIPT_DIR}/install/setup.bash"

mkdir -p "${LOG_DIR}"

print_usage() {
    cat <<'USAGE'
Usage: ./start_robot.sh [options]

Options:
  --skip-build            跳过 colcon build（默认会构建整个工作区）
  --colcon-arg ARG        追加传给 colcon build 的参数，可重复
  -h, --help              显示本帮助

说明:
  该脚本负责构建和加载 ROS 2 工作区环境，不会自动启动节点。
  构建完成后，工作区环境已准备就绪。

示例:
  ./start_robot.sh                                              # 构建整个工作区
  ./start_robot.sh --skip-build                                 # 跳过构建，直接检查环境
  ./start_robot.sh --colcon-arg "--packages-select description" # 只构建指定包
USAGE
}

RUN_BUILD=1
COLCON_ARGS=("--symlink-install")

while [[ $# -gt 0 ]]; do
    case "$1" in
        --skip-build)
            RUN_BUILD=0
            shift
            ;;
        --colcon-arg)
            if [[ $# -lt 2 ]]; then
                echo "Missing value for --colcon-arg" >&2
                exit 1
            fi
            COLCON_ARGS+=("$2")
            shift 2
            ;;
        -h|--help)
            print_usage
            exit 0
            ;;
        *)
            echo "Unknown option: $1" >&2
            print_usage
            exit 1
            ;;
    esac
done

if [[ ${RUN_BUILD} -ne 0 ]]; then
    printf '\n[%s] Running colcon build\n' "$(date --iso-8601=seconds)" | tee -a "${LOG_FILE}"
    if ! (cd "${SCRIPT_DIR}" && colcon build "${COLCON_ARGS[@]}") |& tee -a "${LOG_FILE}"; then
        echo "colcon build failed" | tee -a "${LOG_FILE}"
        exit 1
    fi
    printf '[%s] Build completed successfully\n' "$(date --iso-8601=seconds)" | tee -a "${LOG_FILE}"
else
    printf '\n[%s] colcon build skipped via --skip-build\n' "$(date --iso-8601=seconds)" | tee -a "${LOG_FILE}"
fi

if [[ ! -f "${SETUP_FILE}" ]]; then
    echo "ERROR: ${SETUP_FILE} not found. Build may have failed." | tee -a "${LOG_FILE}"
    exit 1
fi

printf '[%s] ROS 2 workspace ready. Environment file: %s\n' "$(date --iso-8601=seconds)" "${SETUP_FILE}" | tee -a "${LOG_FILE}"
printf '\n========================================\n' | tee -a "${LOG_FILE}"
printf '工作区构建完成!\n' | tee -a "${LOG_FILE}"
printf '========================================\n' | tee -a "${LOG_FILE}"
printf '启动节点:\n' | tee -a "${LOG_FILE}"
printf '  ./src/description/start_robot_desc.sh   # 发布 robot_description 话题\n\n' | tee -a "${LOG_FILE}"
