#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source /home/jetson/dofbot_pro_ws/install/local_setup.bash
source /home/jetson/Codex_Projects/Big/install/local_setup.bash
export ROS_DOMAIN_ID=186 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR=/tmp/edgepick_safe_ros_logs
exec /usr/bin/python3 "$ROOT/scripts/integration_safe_mock.py"
