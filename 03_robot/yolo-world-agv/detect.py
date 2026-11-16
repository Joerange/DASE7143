#!/usr/bin/env python3
"""Capture RealSense RGB using V4L2 and run YOLO-World on the AGV GPU."""
import argparse
import json
import statistics
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

import cv2
import torch
from ultralytics import YOLOWorld

ROOT = Path(__file__).resolve().parent


class Camera:
    def __init__(self, source, width, height):
        self.cap = cv2.VideoCapture(source, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(f"Cannot open camera {source}")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.lock = threading.Lock()
        self.frame = None
        self.seq = 0
        self.timestamp = None
        self.error = None
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._read, daemon=True)
        self.thread.start()

    def _read(self):
        while not self.stop.is_set():
            ok, frame = self.cap.read()
            if not ok:
                self.error = RuntimeError("Camera stopped delivering frames")
                return
            with self.lock:
                self.frame = frame
                self.seq += 1
                self.timestamp = datetime.now(timezone.utc).isoformat()

    def get(self, previous=0, timeout=10):
        start = time.monotonic()
        while time.monotonic() - start < timeout:
            if self.error:
                raise self.error
            with self.lock:
                if self.frame is not None and self.seq > previous:
                    return self.frame.copy(), self.seq, self.timestamp
            time.sleep(0.01)
        raise TimeoutError("No fresh camera frame received in 10 seconds")

    def close(self):
        self.stop.set()
        self.thread.join(timeout=3)
        self.cap.release()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--source", default="/dev/video4")
    p.add_argument("--model", default=str(ROOT / "models/yolov8s-worldv2-trash.pt"))
    p.add_argument("--classes", nargs="+", help="Optional vocabulary override (needs cached CLIP)")
    p.add_argument("--conf", type=float, default=0.15)
    p.add_argument("--imgsz", type=int, default=640)
    p.add_argument("--frames", type=int, default=60, help="0 runs until Ctrl-C")
    p.add_argument("--interval", type=float, default=0.25)
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--output", type=Path)
    p.add_argument("--show", action="store_true", help="Preview on an attached graphical desktop")
    args = p.parse_args()
    if args.frames < 0 or args.interval < 0 or not 0 < args.conf <= 1:
        p.error("frames/interval must be nonnegative; conf must be in (0, 1]")
    if not torch.cuda.is_available():
        raise RuntimeError("CUDA unavailable: use the installed Jetson virtual environment")
    torch.set_num_threads(4)
    model = YOLOWorld(args.model)
    if args.classes:
        model.set_classes(args.classes)
        model.model.clip_model = None
    output = args.output or ROOT / "results" / datetime.now().strftime("%Y%m%d_%H%M%S")
    output.mkdir(parents=True, exist_ok=False)
    for name in ("raw", "annotated"):
        (output / name).mkdir()
    predict = dict(device=0, imgsz=args.imgsz, conf=args.conf, iou=0.5,
                   agnostic_nms=True, half=True, verbose=False)
    camera = Camera(args.source, args.width, args.height)
    rows = []
    started = time.monotonic()
    status = "complete"
    try:
        time.sleep(1)
        frame, seq, _ = camera.get()
        for _ in range(3):
            model.predict(frame, **predict)
        with (output / "detections.jsonl").open("w") as log:
            while args.frames == 0 or len(rows) < args.frames:
                tick = time.monotonic()
                frame, seq, timestamp = camera.get(seq)
                result = model.predict(frame, **predict)[0]
                boxes = []
                for box in result.boxes:
                    cls = int(box.cls.item())
                    boxes.append({"class_id": cls, "label": result.names[cls],
                                  "confidence": float(box.conf.item()),
                                  "xyxy": [round(v, 2) for v in box.xyxy[0].tolist()]})
                index = len(rows)
                annotated = result.plot(line_width=1)
                for subdir, img in (("raw", frame), ("annotated", annotated)):
                    if not cv2.imwrite(str(output / subdir / f"{index:04d}.jpg"), img):
                        raise IOError("Failed to save frame")
                row = {"frame": index, "camera_sequence": seq, "captured_utc": timestamp,
                       "detections": boxes, "speed_ms": result.speed}
                log.write(json.dumps(row) + "\n")
                log.flush()
                rows.append(row)
                print(json.dumps(row), flush=True)
                if args.show:
                    cv2.imshow("YOLO-World garbage-bin test", annotated)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        status = "stopped_by_user"
                        break
                time.sleep(max(0, args.interval - (time.monotonic() - tick)))
    except KeyboardInterrupt:
        status = "stopped_by_user"
    except Exception:
        status = "error"
        raise
    finally:
        camera.close()
        if args.show:
            cv2.destroyAllWindows()
        summary = {"status": status, "model": args.model, "classes": model.names,
                   "source": args.source, "conf": args.conf, "imgsz": args.imgsz,
                   "device": torch.cuda.get_device_name(0), "precision": "FP16",
                   "frames": len(rows), "elapsed_seconds_including_warmup": time.monotonic() - started,
                   "frames_with_detection": sum(bool(r["detections"]) for r in rows),
                   "frames_with_detection_ge_025": sum(any(b["confidence"] >= 0.25 for b in r["detections"]) for r in rows),
                   "max_confidence": max((b["confidence"] for r in rows for b in r["detections"]), default=0),
                   "mean_inference_ms": statistics.mean(r["speed_ms"]["inference"] for r in rows) if rows else None,
                   "note": "Detection frequency is not accuracy; frames require visual verification."}
        (output / "summary.json").write_text(json.dumps(summary, indent=2))
        print("SUMMARY", json.dumps(summary), "OUTPUT", output, flush=True)


if __name__ == "__main__":
    main()
