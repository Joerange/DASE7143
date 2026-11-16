#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
export YOLO_CONFIG_DIR="$PWD/.ultralytics"
exec .venv/bin/python -u detect.py "$@"
