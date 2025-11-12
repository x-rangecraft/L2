#!/usr/bin/env bash
set -euo pipefail

CONFIG_FILE="static_tf_config.yaml"

if [[ ! -f "$CONFIG_FILE" ]]; then
  echo "[ERROR] 未找到 $CONFIG_FILE，请先运行 static_tf_config_build.sh" >&2
  exit 1
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "[ERROR] 未检测到 ros2 命令，请先 source 对应的 ROS 2 环境" >&2
  exit 1
fi

readarray -t TRANSFORM_LINES < <(python3 - "$CONFIG_FILE" <<'PY')
import json, sys
from pathlib import Path
path = Path(sys.argv[1])
with path.open() as f:
    data = json.load(f)
transforms = data.get('transforms', {})
order = []
seen = set()
for edge in data.get('tf_tree', []):
    key = f"{edge.get('parent')}_to_{edge.get('child')}"
    if key in transforms and key not in seen:
        order.append((key, transforms[key]))
        seen.add(key)
for key, value in transforms.items():
    if key not in seen:
        order.append((key, value))
        seen.add(key)
for name, t in order:
    try:
        parent = t['parent_frame']
        child = t['child_frame']
        tr = t['translation_m']
        quat = t['quaternion']['forward_parent_to_child']
    except KeyError as exc:
        raise SystemExit(f"配置缺失字段: {exc}")
    print(
        f"{parent} {child} {tr['x']} {tr['y']} {tr['z']} "
        f"{quat['x']} {quat['y']} {quat['z']} {quat['w']} {name}"
    )
PY
)

if [[ ${#TRANSFORM_LINES[@]} -eq 0 ]]; then
  echo "[ERROR] 配置中未找到任何 transforms" >&2
  exit 1
fi

pids=()
cleanup() {
  if [[ ${#pids[@]} -gt 0 ]]; then
    printf "\n正在停止静态 TF 发布进程...\n"
    for pid in "${pids[@]}"; do
      kill "$pid" 2>/dev/null || true
    done
    if [[ ${#pids[@]} -gt 0 ]]; then
      wait "${pids[@]}" 2>/dev/null || true
    fi
  fi
}
trap cleanup INT TERM

echo "启动静态 TF 发布..."
for line in "${TRANSFORM_LINES[@]}"; do
  read -r parent child tx ty tz qx qy qz qw name <<<"$line"
  echo "  - ${name}: ${parent} -> ${child}"
  ros2 run tf2_ros static_transform_publisher \
    "$tx" "$ty" "$tz" "$qx" "$qy" "$qz" "$qw" \
    "$parent" "$child" &
  pids+=("$!")
done

echo "所有静态 TF 节点已启动，按 Ctrl+C 停止。"
wait
