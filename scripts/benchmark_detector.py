#!/usr/bin/env python3
"""Measure accuracy, artifact bytes, latency and CUDA memory; never invent resume metrics."""
import argparse
import json
from pathlib import Path
import statistics
import time

p=argparse.ArgumentParser();p.add_argument("model");p.add_argument("--data",required=True);p.add_argument("--image",required=True)
p.add_argument("--device",default="0");p.add_argument("--runs",type=int,default=100);p.add_argument("--output",type=Path,required=True)
a=p.parse_args()
if a.runs<5:p.error("--runs must be >= 5")
import torch
from ultralytics import YOLO
m=YOLO(a.model,task="detect");validation=m.val(data=a.data,device=a.device,verbose=False)
for _ in range(10):m.predict(a.image,device=a.device,verbose=False)
if torch.cuda.is_available():torch.cuda.synchronize();torch.cuda.reset_peak_memory_stats()
latencies=[]
for _ in range(a.runs):
    start=time.perf_counter();m.predict(a.image,device=a.device,verbose=False)
    if torch.cuda.is_available():torch.cuda.synchronize()
    latencies.append((time.perf_counter()-start)*1000)
report={"model":a.model,"data":a.data,"runs":a.runs,"map50":float(validation.box.map50),"map50_95":float(validation.box.map),
        "artifact_bytes":Path(a.model).stat().st_size,"latency_ms_median":statistics.median(latencies),
        "latency_ms_p95":sorted(latencies)[int(0.95*(len(latencies)-1))],
        "torch_peak_allocated_bytes":torch.cuda.max_memory_allocated() if torch.cuda.is_available() else None,
        "memory_scope":"PyTorch allocator only; TensorRT/unified-memory total requires tegrastats"}
a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(report,indent=2)+"\n");print(json.dumps(report,indent=2))
