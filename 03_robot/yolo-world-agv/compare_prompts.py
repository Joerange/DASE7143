#!/usr/bin/env python3
"""Diagnostic prompt comparison on captured positive and negative scene images."""
import json
from pathlib import Path
import torch
from ultralytics import YOLOWorld

root = Path(__file__).resolve().parent
out = root / "results/prompt_comparison"
out.mkdir(exist_ok=True)
torch.set_num_threads(4)
model = YOLOWorld(str(root / "models/yolov8s-worldv2.pt"))
model.to("cuda:0")
prompts = ["trash can", "garbage bin", "waste bin", "dustbin", "recycling bin"]
records = []
for labels in [[p] for p in prompts] + [prompts]:
    model.set_classes(labels)
    name = "combined" if len(labels) > 1 else labels[0].replace(" ", "_")
    for scene in ["camera_initial", "camera_bin_placed"]:
        r = model.predict(str(root / "results" / (scene + ".jpg")), device=0,
                          imgsz=640, conf=0.05, iou=0.5, agnostic_nms=True,
                          verbose=False)[0]
        detections = [{"label": r.names[int(b.cls.item())], "confidence": float(b.conf.item()),
                       "xyxy": b.xyxy[0].tolist()} for b in r.boxes]
        row = {"prompts": labels, "scene": scene, "detections": detections}
        records.append(row)
        print(json.dumps(row), flush=True)
        r.save(str(out / f"{scene}_{name}.jpg"))
(out / "comparison.json").write_text(json.dumps(records, indent=2))
