#!/usr/bin/env bash
# Tested on Ubuntu 22.04 aarch64, JetPack 6.2.1, CUDA 12.6, Python 3.10.
set -euo pipefail
cd "$(dirname "$0")"
mkdir -p downloads models results
python3 -m venv --without-pip .venv
if ! .venv/bin/python -m pip --version >/dev/null 2>&1; then
  curl -fL --retry 2 https://bootstrap.pypa.io/get-pip.py -o downloads/get-pip.py
  .venv/bin/python downloads/get-pip.py
fi
# Install CUDA-enabled Jetson builds before resolving PyPI dependencies.
.venv/bin/pip install --no-deps torch==2.8.0 torchvision==0.23.0 \
  --index-url https://pypi.jetson-ai-lab.io/jp6/cu126/
if [[ -f requirements-lock.txt ]]; then
  .venv/bin/pip install -r requirements-lock.txt
else
  .venv/bin/pip install ultralytics==8.3.233 numpy==1.26.4 opencv-python==4.11.0.86 ftfy regex tqdm
  .venv/bin/pip install --no-deps 'git+https://github.com/openai/CLIP.git@d05afc436d78f1c48dc0dbf8e5980a9d471f35f6'
fi
.venv/bin/pip check
if [[ ! -s models/yolov8s-worldv2.pt ]]; then
  curl -fL --retry 2 https://github.com/ultralytics/assets/releases/download/v8.3.0/yolov8s-worldv2.pt -o models/yolov8s-worldv2.pt
fi
export YOLO_CONFIG_DIR="$PWD/.ultralytics"
.venv/bin/python prepare_model.py
