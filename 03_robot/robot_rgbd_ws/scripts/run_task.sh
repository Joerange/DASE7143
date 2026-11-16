#!/usr/bin/env bash
# Run from a Jetson graphical-desktop terminal. No sudo and no CAN reconfiguration.
set -eo pipefail
workspace=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
mode="${1:-task}"
case "$mode" in
  task|--monitor|--demo) ;;
  *) echo 'Usage: run_task.sh [--monitor|--demo]' >&2; exit 2 ;;
esac
if [[ -z "${DISPLAY:-}" ]]; then
  echo 'Run this command in a Jetson graphical-desktop terminal (DISPLAY is missing).' >&2
  exit 1
fi
if [[ "${AMENT_PREFIX_PATH:-}" == *robot_nav_ws* ]]; then
  echo 'Open a fresh terminal without robot_nav_ws sourced.' >&2; exit 1
fi
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"
if [[ ! -f "$workspace/install/local_setup.bash" ]]; then
  echo 'Build once with ~/robot_rgbd_ws/scripts/build.sh' >&2; exit 1
fi
source "$workspace/install/local_setup.bash"
export ROS_DOMAIN_ID=182 ROS_LOCALHOST_ONLY=1
# Prevent two copies of this session. The lock is released when launch exits.
exec 9>"${XDG_RUNTIME_DIR:-/tmp}/bunker-rgbd-${UID}.lock"
if ! flock -n 9; then echo 'A robot_rgbd_ws session is already running.' >&2; exit 1; fi
camera=true;chassis=false;control=false;demo=false
if [[ "$mode" == task ]]; then
  # Read interface flags only; can0 was already configured/verified by the operator.
  if [[ ! -r /sys/class/net/can0/flags ]] || (( ($(cat /sys/class/net/can0/flags) & 1) == 0 )); then
    echo 'can0 is not UP. Use the existing CAN setup before starting; this script does not change it.' >&2; exit 1
  fi
  # Physical CAN is shared across ROS domains: do not start a second chassis driver.
  if pgrep -f '(^|/)bunker_base_node( |$)' >/dev/null; then
    echo 'A chassis driver is already running. Stop its existing session before using this launcher.' >&2; exit 1
  fi
  chassis=true;control=true
elif [[ "$mode" == --demo ]]; then
  camera=false;demo=true
fi
echo "RGB-D session: $mode | S/Enter start | SPACE/X stop | Q/Esc/close stops and exits"
echo 'Motion is disabled until S is pressed and the independent guard accepts it.'
exec ros2 launch bunker_rgbd task.launch.py start_camera:="$camera" start_chassis:="$chassis" \
  allow_runtime_control:="$control" demo:="$demo" show_window:=true
