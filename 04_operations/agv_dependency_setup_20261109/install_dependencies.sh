#!/usr/bin/env bash
# Install ROS dependencies only. Does not start ROS nodes or change CAN state.
set -Eeuo pipefail
if [[ $(id -u) -ne 0 ]]; then
  echo "Run with sudo: sudo bash $0" >&2
  exit 1
fi
task_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
log_path="$task_dir/install.log"
umask 022
exec > >(tee -a "$log_path") 2>&1
trap 'result=$?; if (( result != 0 )); then echo "INSTALL_FAILED exit=$result"; fi' EXIT
echo "INSTALL_STARTED $(date -Is)"
source /etc/os-release
if [[ "$ID" != ubuntu || "$VERSION_ID" != 22.04 || ! -f /opt/ros/humble/setup.bash ]]; then
  echo "Expected Ubuntu 22.04 with ROS 2 Humble; stopping."
  exit 2
fi
export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get install -y --no-remove \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-pointcloud-to-laserscan \
  ros-humble-nav2-velocity-smoother \
  ros-humble-nav2-collision-monitor \
  ros-humble-nav2-simple-commander
apt-get check
dpkg --audit
echo "INSTALL_COMPLETE $(date -Is)"
