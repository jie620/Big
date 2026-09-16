#!/usr/bin/env python3
"""Use Ultralytics' supported ONNX and TensorRT export paths."""
import argparse
p=argparse.ArgumentParser();p.add_argument("weights");p.add_argument("--format",choices=["onnx","engine"],default="onnx")
p.add_argument("--precision",choices=["fp32","fp16","int8"],default="fp16");p.add_argument("--data");p.add_argument("--imgsz",type=int,default=640)
a=p.parse_args()
if a.precision=="int8" and (a.format!="engine" or not a.data):p.error("INT8 requires TensorRT engine and representative --data calibration YAML")
from ultralytics import YOLO
options={"format":a.format,"imgsz":a.imgsz,"half":a.precision=="fp16","device":0 if a.format=="engine" else "cpu"}
if a.precision=="int8":options.update(int8=True,data=a.data)
print(YOLO(a.weights).export(**options))
