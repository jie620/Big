#!/usr/bin/env python3
"""Measure detector accuracy, latency components and CUDA memory.

The timing loop synchronizes CUDA before and after each prediction. This keeps
the report useful on Jetson, where an asynchronous TensorRT enqueue otherwise
looks much faster than the work that actually blocks the next ROS callback.
"""

import argparse
import json
import statistics
import time
from pathlib import Path


def percentile(values: list[float], fraction: float) -> float:
    """Return a nearest-rank percentile without requiring NumPy."""
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, int(fraction * (len(ordered) - 1))))
    return ordered[index]


def summary(values: list[float]) -> dict[str, float]:
    return {
        "p50": statistics.median(values),
        "p95": percentile(values, 0.95),
        "p99": percentile(values, 0.99),
        "min": min(values),
        "max": max(values),
    }


parser = argparse.ArgumentParser()
parser.add_argument("model")
parser.add_argument("--data", required=True)
parser.add_argument("--image", required=True)
parser.add_argument("--device", default="0")
parser.add_argument("--runs", type=int, default=100)
parser.add_argument("--warmup", type=int, default=10)
parser.add_argument("--output", type=Path, required=True)
args = parser.parse_args()
if args.runs < 5:
    parser.error("--runs must be >= 5")
if args.warmup < 0:
    parser.error("--warmup must be >= 0")

import torch
from ultralytics import YOLO


model = YOLO(args.model, task="detect")
validation = model.val(data=args.data, device=args.device, verbose=False)
for _ in range(args.warmup):
    model.predict(args.image, device=args.device, verbose=False)
if torch.cuda.is_available():
    torch.cuda.synchronize()
    torch.cuda.reset_peak_memory_stats()

latencies: list[float] = []
preprocess: list[float] = []
inference: list[float] = []
postprocess: list[float] = []
for _ in range(args.runs):
    if torch.cuda.is_available():
        torch.cuda.synchronize()
    start = time.perf_counter()
    results = model.predict(args.image, device=args.device, verbose=False)
    if torch.cuda.is_available():
        torch.cuda.synchronize()
    latencies.append((time.perf_counter() - start) * 1000)
    # Ultralytics exposes component timings for both PyTorch and TensorRT
    # backends. Keep missing values out rather than reporting fabricated zeroes.
    if results:
        speed = getattr(results[0], "speed", {}) or {}
        for values, key in (
            (preprocess, "preprocess"),
            (inference, "inference"),
            (postprocess, "postprocess"),
        ):
            value = speed.get(key)
            if value is not None:
                values.append(float(value))

report = {
    "model": args.model,
    "data": args.data,
    "runs": args.runs,
    "warmup": args.warmup,
    "map50": float(validation.box.map50),
    "map50_95": float(validation.box.map),
    "artifact_bytes": Path(args.model).stat().st_size,
    # Keep the original keys for existing result consumers.
    "latency_ms_median": statistics.median(latencies),
    "latency_ms_p95": percentile(latencies, 0.95),
    "latency_ms": summary(latencies),
    "torch_peak_allocated_bytes": torch.cuda.max_memory_allocated()
    if torch.cuda.is_available()
    else None,
    "memory_scope": "PyTorch allocator only; TensorRT/unified-memory total requires tegrastats",
}
if preprocess:
    report["preprocess_ms"] = summary(preprocess)
if inference:
    report["inference_ms"] = summary(inference)
if postprocess:
    report["postprocess_ms"] = summary(postprocess)

args.output.parent.mkdir(parents=True, exist_ok=True)
args.output.write_text(json.dumps(report, indent=2) + "\n")
print(json.dumps(report, indent=2))
