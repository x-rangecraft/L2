#!/usr/bin/env bash
# Quick verification script for TF tools

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
URDF_PATH="${SCRIPT_DIR}/../description/urdf/yam.urdf"

echo "======================================"
echo "TF Tools 环境检查"
echo "======================================"
echo ""

# Check ROS environment
echo "1. 检查 ROS 2 环境..."
if command -v ros2 >/dev/null 2>&1; then
  echo "   ✅ ROS 2 已加载: $(which ros2)"
else
  echo "   ❌ ROS 2 未加载，请先 source ROS 2 环境"
  exit 1
fi

# Check URDF
echo ""
echo "2. 检查 URDF 文件..."
if [[ -f "$URDF_PATH" ]]; then
  echo "   ✅ URDF 存在: $URDF_PATH"
  joint_count=$(grep -c '<joint name=' "$URDF_PATH" || true)
  echo "   📊 关节数量: $joint_count"
else
  echo "   ❌ URDF 文件不存在: $URDF_PATH"
  exit 1
fi

# Check Python dependencies
echo ""
echo "3. 检查 Python 依赖..."
python3 -c "import rclpy; import sensor_msgs; import geometry_msgs; import tf2_ros" 2>/dev/null
if [[ $? -eq 0 ]]; then
  echo "   ✅ Python ROS 2 库已安装"
else
  echo "   ❌ Python ROS 2 库缺失，请安装: pip3 install rclpy"
  exit 1
fi

# Check scripts
echo ""
echo "4. 检查脚本文件..."
scripts=(
  "start_robot_tf.sh"
  "src/robot_tf.py"
  "static_tf_config_build.sh"
)

all_ok=true
for script in "${scripts[@]}"; do
  if [[ -x "${SCRIPT_DIR}/${script}" ]]; then
    echo "   ✅ ${script}"
  else
    echo "   ❌ ${script} (不存在或无执行权限)"
    all_ok=false
  fi
done

if [[ "$all_ok" != true ]]; then
  echo ""
  echo "请修复权限: chmod +x src/tf_tools/*.sh src/tf_tools/src/*.{sh,py}"
  exit 1
fi

# Check running processes
echo ""
echo "5. 检查运行状态..."
"${SCRIPT_DIR}/start_robot_tf.sh" --status 2>/dev/null || echo "   ℹ️  当前未运行 robot_tf 节点"

# Check topics
echo ""
echo "6. 检查相关话题..."
topics=(
  "/joint_states"
  "/robot_description"
  "/tf"
  "/tf_static"
)

for topic in "${topics[@]}"; do
  if ros2 topic list 2>/dev/null | grep -q "^${topic}$"; then
    pub_count=$(ros2 topic info "$topic" 2>/dev/null | grep "Publisher count:" | awk '{print $3}')
    sub_count=$(ros2 topic info "$topic" 2>/dev/null | grep "Subscription count:" | awk '{print $3}')
    echo "   ✅ ${topic} (发布: ${pub_count}, 订阅: ${sub_count})"
  else
    echo "   ⚠️  ${topic} (不存在)"
  fi
done

echo ""
echo "======================================"
echo "✅ 环境检查完成"
echo "======================================"
echo ""
echo "快速启动命令:"
echo "  后台启动: ./src/tf_tools/start_robot_tf.sh --start"
echo "  前台运行: ./src/tf_tools/start_robot_tf.sh --foreground"
echo "  查看状态: ./src/tf_tools/start_robot_tf.sh --status"
echo "  停止服务: ./src/tf_tools/start_robot_tf.sh --stop"
echo ""
echo "详细文档: ./src/tf_tools/README.md"
echo ""
