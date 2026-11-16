#!/usr/bin/env bash
# Local desktop viewer. Starting it never starts CAN, chassis or mission.
set -eo pipefail
workspace=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
if [ "${1:-}" = monitor ]; then shift; fi
if [ "${1:-}" = --demo ]; then
  export PYTHONPATH="$workspace/src/robot_console${PYTHONPATH:+:$PYTHONPATH}"
  exec /usr/bin/python3 -m robot_console.ui "$@"
fi
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"
source "$workspace/install/setup.bash"
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-180}"
socket_path="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}/robot-console.sock"
control_socket="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}/robot-console-control.sock"
if /usr/bin/python3 - "$control_socket" <<'PY'
import socket,sys
try:
    with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as client:
        client.settimeout(.5);client.connect(sys.argv[1])
except OSError:sys.exit(1)
PY
then
  exec /usr/bin/python3 -m robot_console.ui --socket "$control_socket" "$@"
fi
mkdir -p "$workspace/console_exports"
if ! /usr/bin/python3 - "$socket_path" <<'PY'
import socket,sys
try:
    with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as client:
        client.settimeout(.5);client.connect(sys.argv[1])
except OSError:sys.exit(1)
PY
then
  # A monitor backend remains independent of the window. Control mode is only
  # started through the reviewed local service, never by this desktop script.
  nohup /usr/bin/python3 -m robot_console.backend --socket "$socket_path" \
    --export-dir "$workspace/console_exports" >"$workspace/console_exports/monitor.log" 2>&1 </dev/null &
  backend_pid=$!
  /usr/bin/python3 - "$socket_path" "$backend_pid" <<'PY'
import os,socket,sys,time
for _ in range(100):
    try:
        with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as client:
            client.settimeout(.1);client.connect(sys.argv[1]);break
    except OSError:
        try:os.kill(int(sys.argv[2]),0)
        except OSError:raise SystemExit('Monitor backend failed; see console_exports/monitor.log')
        time.sleep(.1)
else:raise SystemExit('Monitor socket not ready; see console_exports/monitor.log')
PY
fi
exec /usr/bin/python3 -m robot_console.ui --socket "$socket_path" "$@"
