#!/usr/bin/env bash
# Prepare/install only; never enables CAN, starts the driver, or starts services.
set -euo pipefail
source_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
case "${1:---check}" in
  --check) ;;
  --install) [ "$(id -u)" -eq 0 ] || { echo 'Installation requires an explicit root invocation.' >&2; exit 1; } ;;
  *) echo 'Usage: install_services.sh [--check|--install]' >&2; exit 2 ;;
esac
for path in can_helper.py can_client.py run_console.sh systemd/robot-console-can.service systemd/robot-console.service config/can.json config/session.sudoers; do
  [ -f "$source_dir/$path" ] || { echo "Missing reviewed console file: $path" >&2; exit 1; }
done
[ "$(id -u dase-orinnx)" -eq 1000 ] || { echo 'Service user UID differs from reviewed config; edit and review both first.' >&2; exit 1; }
[ -x /usr/sbin/ip ] && [ -x /usr/bin/systemctl ] && [ -x /usr/bin/python3 ]
if [ "${1:---check}" != '--install' ]; then
  echo 'Checks passed. --install copies reviewed files and reloads unit definitions only.'
  exit 0
fi
install -d -o root -g root -m 0755 /usr/local/lib/robot-console /etc/robot-console
# The privileged helper and launch wrapper are immutable to the desktop account.
find "$source_dir" -maxdepth 1 -type f \( -name '*.py' -o -name 'run_console.sh' \) -exec install -o root -g root -m 0755 '{}' /usr/local/lib/robot-console/ \;
if [ -d "$source_dir/static" ]; then
  cp -R "$source_dir/static" /usr/local/lib/robot-console/
  chown -R root:root /usr/local/lib/robot-console/static
  chmod -R go-w /usr/local/lib/robot-console/static
fi
install -o root -g root -m 0644 "$source_dir/config/can.json" /etc/robot-console/can.json
install -o root -g root -m 0644 "$source_dir/systemd/robot-console-can.service" /etc/systemd/system/robot-console-can.service
install -o root -g root -m 0644 "$source_dir/systemd/robot-console.service" /etc/systemd/system/robot-console.service
/usr/sbin/visudo -cf "$source_dir/config/session.sudoers"
install -o root -g root -m 0440 "$source_dir/config/session.sudoers" /etc/sudoers.d/robot-console-session
install -d -o dase-orinnx -g dase-orinnx -m 0700 /home/dase-orinnx/.ros /home/dase-orinnx/robot_nav_ws/mission_logs /home/dase-orinnx/robot_nav_ws/console_exports
systemctl daemon-reload
echo 'Installed definitions only. No service, CAN interface, driver or mission was started.'
echo 'Review operating instructions before explicitly starting robot-console-can.service and robot-console.service.'
