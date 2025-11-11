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
  echo "Loading workspace setup from ${WORKSPACE_SETUP}..." >&2
  set +u
  source "${WORKSPACE_SETUP}"
  set -u
else
  echo "Workspace setup not found at ${WORKSPACE_SETUP}" >&2
  echo "Using system foxglove_bridge package instead." >&2
fi

# Check if port is already in use
if netstat -tuln 2>/dev/null | grep -q ":${PORT} " || ss -tuln 2>/dev/null | grep -q ":${PORT} "; then
  echo "Port ${PORT} is already in use." >&2
  echo "Checking for existing foxglove_bridge processes..." >&2

  # Find existing foxglove_bridge processes
  EXISTING_PIDS=$(pgrep -f "foxglove_bridge.*port:=${PORT}" || true)

  if [[ -n "${EXISTING_PIDS}" ]]; then
    echo "Found existing foxglove_bridge process(es) on port ${PORT}:" >&2
    ps -p ${EXISTING_PIDS} -o pid,cmd 2>/dev/null || true
    echo "" >&2
    echo "Stopping existing process(es)..." >&2
    kill ${EXISTING_PIDS} 2>/dev/null || true
    sleep 2

    # Force kill if still running
    if pgrep -f "foxglove_bridge.*port:=${PORT}" > /dev/null; then
      echo "Force stopping..." >&2
      kill -9 ${EXISTING_PIDS} 2>/dev/null || true
      sleep 1
    fi
    echo "Existing process(es) stopped." >&2
  else
    echo "Port is in use by another process. Please free port ${PORT} or set FOXGLOVE_PORT to a different value." >&2
    exit 1
  fi
fi

cd -- "${WORKSPACE_ROOT}"

# Log file path
LOG_DIR="${WORKSPACE_ROOT}/log"
mkdir -p "${LOG_DIR}"
LOG_FILE="${LOG_DIR}/foxglove_bridge.log"

echo "Starting foxglove_bridge on ${ADDRESS}:${PORT}..." >&2
echo "" >&2
echo "========================================" >&2
echo "Foxglove WebSocket Connection Info:" >&2
echo "----------------------------------------" >&2

# Get local IP addresses for display
LOCAL_IPS=$(hostname -I 2>/dev/null | tr ' ' '\n' | grep -v '^$' || echo "")

if [[ "${ADDRESS}" == "0.0.0.0" ]]; then
  echo "Server listening on all interfaces (0.0.0.0:${PORT})" >&2
  echo "" >&2
  echo "Connect using one of these URLs:" >&2
  if [[ -n "${LOCAL_IPS}" ]]; then
    while IFS= read -r ip; do
      echo "  ws://${ip}:${PORT}" >&2
    done <<< "${LOCAL_IPS}"
  fi
  echo "  ws://localhost:${PORT}" >&2
else
  echo "  ws://${ADDRESS}:${PORT}" >&2
fi

echo "========================================" >&2
echo "" >&2

# Start foxglove_bridge in background with nohup
nohup ros2 run foxglove_bridge foxglove_bridge --ros-args -p address:=${ADDRESS} -p port:=${PORT} >> "${LOG_FILE}" 2>&1 &
BRIDGE_PID=$!

# Wait a moment to check if process started successfully
sleep 2

if ps -p ${BRIDGE_PID} > /dev/null 2>&1; then
  echo "✓ Foxglove Bridge started successfully (PID: ${BRIDGE_PID})" >&2
  echo "  Log file: ${LOG_FILE}" >&2
  echo "" >&2
  echo "To stop the service, run:" >&2
  echo "  kill ${BRIDGE_PID}" >&2
  echo "  or: pkill -f 'foxglove_bridge.*port:=${PORT}'" >&2
else
  echo "✗ Failed to start Foxglove Bridge" >&2
  echo "  Check log file: ${LOG_FILE}" >&2
  exit 1
fi
