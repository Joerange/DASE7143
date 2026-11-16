#!/usr/bin/env bash
# Works from SSH; no GUI, sudo, CAN manipulation or velocity publisher.
set -eo pipefail
robot_workspace=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
if [ "${1:-}" = --help ] || [ "${1:-}" = -h ]; then
  exec /usr/bin/python3 "$robot_workspace/scripts/stop_robot.py" --help
fi
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"
source "$robot_workspace/install/setup.bash"
export ROS_DOMAIN_ID=180 ROS_LOCALHOST_ONLY=1
exec /usr/bin/python3 "$robot_workspace/scripts/stop_robot.py" "$@"
