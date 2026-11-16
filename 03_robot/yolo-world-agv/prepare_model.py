#!/usr/bin/env python3
"""Embed the garbage-bin vocabulary once for subsequent offline inference."""
from pathlib import Path
import torch
from ultralytics import YOLOWorld

root = Path(__file__).resolve().parent
torch.set_num_threads(4)
model = YOLOWorld(str(root / "models/yolov8s-worldv2.pt"))
classes = ["trash can", "garbage bin", "waste bin", "dustbin", "recycling bin"]
model.set_classes(classes)
# Retain text embeddings, but omit the large encoder from the inference checkpoint.
model.model.clip_model = None
target = root / "models/yolov8s-worldv2-trash.pt"
model.save(str(target))
print("Saved", target, classes)
