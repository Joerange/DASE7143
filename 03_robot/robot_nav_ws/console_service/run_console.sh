#!/usr/bin/env bash
# systemd MainPID must become the trusted ordinary Python control service.
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/dase-orinnx/ros2_ws/install/setup.bash
source /home/dase-orinnx/robot_nav_ws/install/setup.bash
exec /usr/bin/python3 -m robot_console.backend --control-enabled --socket /run/user/1000/robot-console-control.sock
