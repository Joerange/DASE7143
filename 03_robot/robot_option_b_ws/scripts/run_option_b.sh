#!/usr/bin/env bash
# Usage: run_option_b.sh [target label] [extra ros2 launch arguments...]
set -eo pipefail
option_b_workspace=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
option_b_target=${1:-trash can}
if [ "$#" -gt 0 ]; then shift; fi
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"
source "$option_b_workspace/install/setup.bash"
export ROS_DOMAIN_ID=180 ROS_LOCALHOST_ONLY=1
exec ros2 launch robot_bringup task.launch.py start_sensors:=true enable_navigation:=true \
  allow_runtime_control:=true rviz:=true "target_label:=$option_b_target" "$@"
