#!/usr/bin/env bash
# Desktop session: automatic preparation; mission start remains manual.
set -eo pipefail
umask 077
robot_workspace=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
if [ "${1:-}" = --help ] || [ "${1:-}" = -h ]; then
  exec /usr/bin/python3 "$robot_workspace/scripts/start_robot.py" --help
fi
if [ "$(uname -s)" != Linux ] || [ "$(id -u)" -eq 0 ]; then
  echo '请在 Jetson 图形桌面的普通用户终端运行，勿 sudo 整个脚本。' >&2
  exit 1
fi
if [ -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]; then
  echo '未找到图形桌面 DISPLAY/WAYLAND_DISPLAY；请在 Jetson 桌面终端执行。' >&2
  exit 1
fi
for robot_setup in /opt/ros/humble/setup.bash "$HOME/ros2_ws/install/setup.bash" "$robot_workspace/install/setup.bash"; do
  if [ ! -f "$robot_setup" ]; then
    echo "缺少环境文件：$robot_setup；请先按 README.md 构建工作区。" >&2
    exit 1
  fi
  source "$robot_setup"
done
# Must match the reviewed robot-console.service; do not inherit a test domain.
export ROS_DOMAIN_ID=180 ROS_LOCALHOST_ONLY=1
exec /usr/bin/python3 "$robot_workspace/scripts/start_robot.py" "$@"
