#!/usr/bin/env python3
"""Validate an explicitly declared training contract and package local model files."""
import argparse
import json
from pathlib import Path
import shutil
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"src/edgepick_safe"))
from edgepick_safe.contract import JOINTS, checkpoint_contract
parser=argparse.ArgumentParser()
parser.add_argument("checkpoint",type=Path)
parser.add_argument("--absolute-ros-radians",action="store_true",required=True,
                    help="Assert training actions are absolute ROS radians in DOFBOT joint order")
parser.add_argument("--output",type=Path,default=ROOT/"models/vla")
parser.add_argument("--vlm-assets",type=Path,help="Local tokenizer/VLM directory, copied into the model package")
args=parser.parse_args()
source=args.checkpoint.resolve();output=args.output.resolve()
if output.exists():raise SystemExit(f"Output exists; choose a new directory: {output}")
for name in ("config.json","model.safetensors"):
    if not (source/name).is_file():raise SystemExit(f"Missing {source/name}")
config=json.loads((source/"config.json").read_text())
vlm=args.vlm_assets or Path(config.get("vlm_model_name",""))
if not vlm.is_dir() or not (vlm/"config.json").is_file():raise SystemExit("Provide --vlm-assets with the local tokenizer/VLM assets used for training")
output.mkdir(parents=True)
try:
    for item in source.iterdir():
        if item.is_file() and item.suffix in (".json",".safetensors"):shutil.copy2(item,output/item.name)
    shutil.copytree(vlm,output/"vlm")
    config["vlm_model_name"]=str(output/"vlm")
    (output/"config.json").write_text(json.dumps(config,indent=2)+"\n")
    manifest={"joint_names":JOINTS,"action_units":"radian","action_mode":"absolute_joint_position"}
    (output/"edgepick_contract.json").write_text(json.dumps(manifest,indent=2)+"\n")
    checkpoint_contract(output)
except Exception:
    shutil.rmtree(output)
    raise
print(output)
print("Contract validated; local tokenizer/VLM assets included.")
