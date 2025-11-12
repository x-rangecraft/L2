#!/usr/bin/env bash
set -euo pipefail

CONFIG_FILE="static_tf_config.yaml"

# Initialize defaults
BASE_TX_DEFAULT="0.0"; BASE_TY_DEFAULT="0.0"; BASE_TZ_DEFAULT="0.0"
BASE_WRX_DEFAULT="0.0"; BASE_WRY_DEFAULT="0.0"; BASE_WRZ_DEFAULT="0.0"
BASE_BRX_DEFAULT="0.0"; BASE_BRY_DEFAULT="0.0"; BASE_BRZ_DEFAULT="0.0"
CAM_TX_DEFAULT="0.0"; CAM_TY_DEFAULT="0.0"; CAM_TZ_DEFAULT="0.0"
CAM_WRX_DEFAULT="0.0"; CAM_WRY_DEFAULT="0.0"; CAM_WRZ_DEFAULT="0.0"
CAM_BRX_DEFAULT="0.0"; CAM_BRY_DEFAULT="0.0"; CAM_BRZ_DEFAULT="0.0"

load_defaults() {
  if [[ ! -f "${CONFIG_FILE}" ]]; then
    return
  fi
  local python_output
  if ! python_output=$(python3 - "$CONFIG_FILE" <<'PY'
import json, sys
from pathlib import Path
path = Path(sys.argv[1])
if not path.exists():
    sys.exit(0)
with path.open() as f:
    data = json.load(f)
transforms = data.get('transforms', {})
def emit(prefix, key):
    t = transforms.get(key, {})
    tr = t.get('translation_m', {})
    rw = t.get('rotation_deg', {}).get('about_world_axes', {})
    rb = t.get('rotation_deg', {}).get('about_body_axes', {})
    print(f"{prefix}_TX_DEFAULT={tr.get('x', 0.0)}")
    print(f"{prefix}_TY_DEFAULT={tr.get('y', 0.0)}")
    print(f"{prefix}_TZ_DEFAULT={tr.get('z', 0.0)}")
    print(f"{prefix}_WRX_DEFAULT={rw.get('x', 0.0)}")
    print(f"{prefix}_WRY_DEFAULT={rw.get('y', 0.0)}")
    print(f"{prefix}_WRZ_DEFAULT={rw.get('z', 0.0)}")
    print(f"{prefix}_BRX_DEFAULT={rb.get('x', 0.0)}")
    print(f"{prefix}_BRY_DEFAULT={rb.get('y', 0.0)}")
    print(f"{prefix}_BRZ_DEFAULT={rb.get('z', 0.0)}")
emit('BASE', 'world_to_base_link')
emit('CAM', 'world_to_camera_link')
PY
); then
    echo "[WARN] 读取现有配置失败，继续使用默认 0" >&2
    return
  fi
  while IFS='=' read -r key value; do
    [[ -z "$key" ]] && continue
    printf -v "$key" '%s' "$value"
  done <<<"${python_output}"
}

is_number() {
  [[ $1 =~ ^-?[0-9]*\.?[0-9]+([eE]-?[0-9]+)?$ ]]
}

prompt_numeric() {
  local __var="$1" __prompt="$2" __default="$3" __input
  while true; do
    read -r -p "$__prompt [$__default]: " __input || true
    __input=${__input:-$__default}
    if is_number "$__input"; then
      printf -v "$__var" '%s' "$__input"
      break
    fi
    echo "请输入合法数字 (允许小数/科学计数法)" >&2
  done
}

collect_transform() {
  local prefix="$1" label="$2"
  local tx_default="${prefix}_TX_DEFAULT" ty_default="${prefix}_TY_DEFAULT" tz_default="${prefix}_TZ_DEFAULT"
  local wrx_default="${prefix}_WRX_DEFAULT" wry_default="${prefix}_WRY_DEFAULT" wrz_default="${prefix}_WRZ_DEFAULT"
  local brx_default="${prefix}_BRX_DEFAULT" bry_default="${prefix}_BRY_DEFAULT" brz_default="${prefix}_BRZ_DEFAULT"

  prompt_numeric "${prefix}_TX" "${label}：X (m)" "${!tx_default}"
  prompt_numeric "${prefix}_TY" "${label}：Y (m)" "${!ty_default}"
  prompt_numeric "${prefix}_TZ" "${label}：Z (m)" "${!tz_default}"

  prompt_numeric "${prefix}_WRX" "${label}：绕世界 X 轴旋转 (deg)" "${!wrx_default}"
  prompt_numeric "${prefix}_WRY" "${label}：绕世界 Y 轴旋转 (deg)" "${!wry_default}"
  prompt_numeric "${prefix}_WRZ" "${label}：绕世界 Z 轴旋转 (deg)" "${!wrz_default}"

  prompt_numeric "${prefix}_BRX" "${label}：绕自身 X 轴旋转 (deg)" "${!brx_default}"
  prompt_numeric "${prefix}_BRY" "${label}：绕自身 Y 轴旋转 (deg)" "${!bry_default}"
  prompt_numeric "${prefix}_BRZ" "${label}：绕自身 Z 轴旋转 (deg)" "${!brz_default}"
}

print_summary() {
  cat <<'TABLE'
================ 参数确认 ================
TABLE
  printf "%-20s %-18s %-18s %-18s\n" "节点" "平移 (m)" "世界轴旋转 (deg)" "自身轴旋转 (deg)"
  printf "%-20s %-18s %-18s %-18s\n" "--------------------" "------------------" "----------------------" "----------------------"
  printf "%-20s %-18s %-18s %-18s\n" \
    "base_link" \
    "(${BASE_TX}, ${BASE_TY}, ${BASE_TZ})" \
    "(${BASE_WRX}, ${BASE_WRY}, ${BASE_WRZ})" \
    "(${BASE_BRX}, ${BASE_BRY}, ${BASE_BRZ})"
  printf "%-20s %-18s %-18s %-18s\n" \
    "camera_link" \
    "(${CAM_TX}, ${CAM_TY}, ${CAM_TZ})" \
    "(${CAM_WRX}, ${CAM_WRY}, ${CAM_WRZ})" \
    "(${CAM_BRX}, ${CAM_BRY}, ${CAM_BRZ})"
  echo "==========================================="
}

generate_config() {
  export BASE_TX BASE_TY BASE_TZ BASE_WRX BASE_WRY BASE_WRZ BASE_BRX BASE_BRY BASE_BRZ
  export CAM_TX CAM_TY CAM_TZ CAM_WRX CAM_WRY CAM_WRZ CAM_BRX CAM_BRY CAM_BRZ
  python3 - "$CONFIG_FILE" <<'PY'
import json, math, os, sys
from datetime import datetime, timezone

config_path = sys.argv[1]

AXIS = {
    'X': (1.0, 0.0, 0.0),
    'Y': (0.0, 1.0, 0.0),
    'Z': (0.0, 0.0, 1.0),
}

order = ['X', 'Y', 'Z']

def quat_mul(q2, q1):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w2 * x1 + x2 * w1 + y2 * z1 - z2 * y1,
        w2 * y1 - x2 * z1 + y2 * w1 + z2 * x1,
        w2 * z1 + x2 * y1 - y2 * x1 + z2 * w1,
        w2 * w1 - x2 * x1 - y2 * y1 - z2 * z1,
    )

def quat_conj(q):
    x, y, z, w = q
    return (-x, -y, -z, w)

def quat_normalize(q):
    x, y, z, w = q
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if norm == 0:
        return (0.0, 0.0, 0.0, 1.0)
    return (x / norm, y / norm, z / norm, w / norm)

def axis_angle_quat(axis, angle):
    axis_x, axis_y, axis_z = axis
    norm = math.sqrt(axis_x**2 + axis_y**2 + axis_z**2)
    if norm == 0.0 or abs(angle) < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    axis_x /= norm
    axis_y /= norm
    axis_z /= norm
    half = angle / 2.0
    sin_half = math.sin(half)
    return (axis_x * sin_half, axis_y * sin_half, axis_z * sin_half, math.cos(half))

def rotate_vector(q, v):
    vx, vy, vz = v
    qv = (vx, vy, vz, 0.0)
    out = quat_mul(quat_mul(q, qv), quat_conj(q))
    return out[0], out[1], out[2]

def build_transform(name, parent, child, translation, rot_world, rot_body):
    q = (0.0, 0.0, 0.0, 1.0)
    sequence = []
    # world rotations (extrinsic)
    for axis in order:
        angle_deg = rot_world[axis]
        sequence.append({"type": "world_axis", "axis": axis, "angle_deg": angle_deg})
        q_inc = axis_angle_quat(AXIS[axis], math.radians(angle_deg))
        q = quat_normalize(quat_mul(q_inc, q))
    # body rotations (intrinsic)
    for axis in order:
        angle_deg = rot_body[axis]
        sequence.append({"type": "body_axis", "axis": axis, "angle_deg": angle_deg})
        local_axis_world = rotate_vector(q, AXIS[axis])
        q_inc = axis_angle_quat(local_axis_world, math.radians(angle_deg))
        q = quat_normalize(quat_mul(q_inc, q))
    forward = quat_normalize(q)
    inverse = quat_conj(forward)
    tx, ty, tz = translation
    inv_tx, inv_ty, inv_tz = rotate_vector(inverse, (-tx, -ty, -tz))
    return {
        "name": name,
        "parent_frame": parent,
        "child_frame": child,
        "translation_m": {"x": tx, "y": ty, "z": tz},
        "rotation_deg": {
            "about_world_axes": {axis.lower(): rot_world[axis] for axis in order},
            "about_body_axes": {axis.lower(): rot_body[axis] for axis in order},
        },
        "transform_sequence": sequence,
        "quaternion": {
            "forward_parent_to_child": {"x": forward[0], "y": forward[1], "z": forward[2], "w": forward[3]},
            "inverse_child_to_parent": {"x": inverse[0], "y": inverse[1], "z": inverse[2], "w": inverse[3]},
        },
        "inverse_translation_m": {"x": inv_tx, "y": inv_ty, "z": inv_tz},
    }

base_transform = build_transform(
    'world_to_base_link',
    'world',
    'base_link',
    (float(os.environ['BASE_TX']), float(os.environ['BASE_TY']), float(os.environ['BASE_TZ'])),
    {'X': float(os.environ['BASE_WRX']), 'Y': float(os.environ['BASE_WRY']), 'Z': float(os.environ['BASE_WRZ'])},
    {'X': float(os.environ['BASE_BRX']), 'Y': float(os.environ['BASE_BRY']), 'Z': float(os.environ['BASE_BRZ'])}
)

camera_transform = build_transform(
    'world_to_camera_link',
    'world',
    'camera_link',
    (float(os.environ['CAM_TX']), float(os.environ['CAM_TY']), float(os.environ['CAM_TZ'])),
    {'X': float(os.environ['CAM_WRX']), 'Y': float(os.environ['CAM_WRY']), 'Z': float(os.environ['CAM_WRZ'])},
    {'X': float(os.environ['CAM_BRX']), 'Y': float(os.environ['CAM_BRY']), 'Z': float(os.environ['CAM_BRZ'])}
)

data = {
    "metadata": {
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "generator": "static_tf_config_build.sh",
        "translation_units": "m",
        "rotation_units": "deg",
        "rotation_order": {
            "world_axes": order,
            "body_axes": order,
        },
    },
    "tf_tree": [
        {"parent": "world", "child": "base_link"},
        {"parent": "world", "child": "camera_link"},
    ],
    "transforms": {
        "world_to_base_link": base_transform,
        "world_to_camera_link": camera_transform,
    },
}

with open(config_path, 'w', encoding='utf-8') as f:
    json.dump(data, f, indent=2)
    f.write('\n')
PY
}

main() {
  load_defaults
  echo "== 配置 world → base_link 参数 =="
  collect_transform "BASE" "base_link"
  printf "\\n== 配置 world → camera_link 参数 ==\\n"
  collect_transform "CAM" "camera_link"
  echo
  print_summary
  read -r -p "确认生成 static_tf_config.yaml ? (Y/n): " confirm
  confirm=${confirm:-Y}
  if [[ ! "$confirm" =~ ^[Yy]$ ]]; then
    echo "已取消生成。"
    exit 1
  fi
  generate_config
  echo "已生成 ${CONFIG_FILE}"
}

main "$@"
