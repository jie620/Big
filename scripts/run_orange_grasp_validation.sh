#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/edgepick_ros_logs}"
CAMERA_LAUNCH_FILE="${CAMERA_LAUNCH_FILE:-dabai_dcw2.launch.py}"
GRIPPER_OPEN_POSITION="${GRIPPER_OPEN_POSITION:--0.0796}"
GRIPPER_CLOSE_POSITION="${GRIPPER_CLOSE_POSITION:--0.9}"
SHOW_VIEWER="${SHOW_VIEWER:-true}"
WAIT_TIMEOUT_SEC="${WAIT_TIMEOUT_SEC:-60}"

mkdir -p "$ROS_LOG_DIR"

set +u
source /opt/ros/humble/setup.bash
source /home/jetson/dofbot_pro_ws/install/setup.bash
source "$ROOT_DIR/install/setup.bash"
set -u

cleanup() {
  if [[ -n "${CAMERA_PID:-}" ]] && kill -0 "$CAMERA_PID" 2>/dev/null; then
    kill -- "-$CAMERA_PID" 2>/dev/null || kill "$CAMERA_PID" 2>/dev/null || true
    wait "$CAMERA_PID" 2>/dev/null || true
  fi
}

wait_for_topic() {
  local topic="$1"
  local deadline=$((SECONDS + WAIT_TIMEOUT_SEC))

  while ! ros2 topic list | grep -Fxq "$topic"; do
    if (( SECONDS >= deadline )); then
      echo "Timed out waiting for topic: $topic" >&2
      return 1
    fi
    sleep 1
  done
}

trap cleanup EXIT INT TERM

echo "[1/3] starting Orbbec camera: orbbec_camera ${CAMERA_LAUNCH_FILE}"
setsid ros2 launch orbbec_camera "$CAMERA_LAUNCH_FILE" \
  >"$ROS_LOG_DIR/orbbec_camera_validation.log" 2>&1 &
CAMERA_PID=$!

echo "[2/3] waiting for camera topics"
wait_for_topic /camera/color/image_raw
wait_for_topic /camera/depth/image_raw
wait_for_topic /camera/depth/camera_info

echo "[3/3] launching orange grasp execution"
echo "  gripper_open_position=${GRIPPER_OPEN_POSITION}"
echo "  gripper_close_position=${GRIPPER_CLOSE_POSITION}"
echo "  show_viewer=${SHOW_VIEWER}"

ros2 launch edgepick_bringup edgepick_orange_grasp_execution.launch.py \
  show_viewer:="$SHOW_VIEWER" \
  gripper_open_position:="$GRIPPER_OPEN_POSITION" \
  gripper_close_position:="$GRIPPER_CLOSE_POSITION"

status=$?
cleanup
exit "$status"
