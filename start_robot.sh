#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="${SCRIPT_DIR}/log"
LOG_FILE="${LOG_DIR}/start_robot.log"
DESC_SCRIPT="${SCRIPT_DIR}/src/description/start_robot_desc.sh"

mkdir -p "${LOG_DIR}"

print_usage() {
    cat <<'USAGE'
Usage: ./start_robot.sh [options] [-- start_robot_desc args]

Options:
  --skip-build            跳过 colcon build（默认会构建整个工作区）
  --colcon-arg ARG        追加传给 colcon build 的参数，可重复
  -h, --help              显示本帮助

示例：
  ./start_robot.sh --                         # 构建后使用默认参数启动 robot_desc_node
  ./start_robot.sh --colcon-arg "--packages-select description" -- --frame-prefix robot1_
USAGE
}

RUN_BUILD=1
COLCON_ARGS=("--symlink-install")
DESC_ARGS=()

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
        --)
            shift
            DESC_ARGS+=("$@")
            break
            ;;
        *)
            echo "Unknown option: $1" >&2
            print_usage
            exit 1
            ;;
    esac
done

if [[ ! -x "${DESC_SCRIPT}" ]]; then
    echo "Missing executable ${DESC_SCRIPT}" | tee -a "${LOG_FILE}"
    exit 1
fi

if [[ ${RUN_BUILD} -ne 0 ]]; then
    printf '\n[%s] Running colcon build\n' "$(date --iso-8601=seconds)" | tee -a "${LOG_FILE}"
    if ! (cd "${SCRIPT_DIR}" && colcon build "${COLCON_ARGS[@]}") |& tee -a "${LOG_FILE}"; then
        echo "colcon build failed" | tee -a "${LOG_FILE}"
        exit 1
    fi
else
    printf '\n[%s] colcon build skipped via --skip-build\n' "$(date --iso-8601=seconds)" | tee -a "${LOG_FILE}"
fi

printf '[%s] Launching robot description pipeline\n' "$(date --iso-8601=seconds)" | tee -a "${LOG_FILE}"
"${DESC_SCRIPT}" "${DESC_ARGS[@]}" |& tee -a "${LOG_FILE}"
