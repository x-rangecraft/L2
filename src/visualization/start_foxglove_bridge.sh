#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
WORKSPACE_ROOT=$(cd -- "${SCRIPT_DIR}/../.." && pwd)
ROS_SETUP="/opt/ros/humble/setup.bash"
WORKSPACE_SETUP="${WORKSPACE_ROOT}/install/setup.bash"
ADDRESS=${FOXGLOVE_ADDRESS:-0.0.0.0}
PORT=${FOXGLOVE_PORT:-9090}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "Missing ROS setup script: ${ROS_SETUP}" >&2
  exit 1
fi
# Temporarily disable nounset because upstream setup files access unset vars.
set +u
source "${ROS_SETUP}"
set -u

if [[ -f "${WORKSPACE_SETUP}" ]]; then
  set +u
  source "${WORKSPACE_SETUP}"
  set -u
else
  echo "Workspace setup not found at ${WORKSPACE_SETUP}, continuing with system environment." >&2
fi

cd -- "${WORKSPACE_ROOT}"
exec ros2 run foxglove_bridge foxglove_bridge --ros-args -p address:=${ADDRESS} -p port:=${PORT}
