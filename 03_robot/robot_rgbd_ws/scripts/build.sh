#!/usr/bin/env bash
set -eo pipefail
workspace=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"
if [[ "${AMENT_PREFIX_PATH:-}" == *robot_nav_ws* ]]; then
  echo 'Open a fresh terminal without robot_nav_ws sourced.' >&2; exit 1
fi
cd "$workspace"
colcon build --symlink-install --packages-select bunker_rgbd
