#!/usr/bin/env bash
set -eo pipefail
workspace=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
exec "$workspace/scripts/run_task.sh" --monitor
