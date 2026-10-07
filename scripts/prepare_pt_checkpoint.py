#!/usr/bin/env python3
"""Convert a custom SmolVLA ``best.pt`` snapshot for Big's runtime.

The training snapshot is a full PyTorch pickle.  Big loads portable
``model.safetensors`` plus an explicit contract, so this converter preserves
the model tensors and records the input modality observed in the checkpoint.
It never adds a depth branch that was absent during training.
"""

from __future__ import annotations

import argparse
import json
import pathlib
import shutil
from typing import Any

import torch
from safetensors.torch import save_file


ROOT = pathlib.Path(__file__).resolve().parents[1]
DEFAULT_TEMPLATE = ROOT.parent / "Edge_AI" / "smolvla_v3_360000" / "best_pretrained" / "config.json"


def portable(value: Any) -> Any:
    if isinstance(value, pathlib.PurePath):
        return str(value)
    if isinstance(value, dict):
        return {portable(key): portable(item) for key, item in value.items()}
    if isinstance(value, list):
        return [portable(item) for item in value]
    if isinstance(value, tuple):
        return [portable(item) for item in value]
    return value


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=pathlib.Path)
    parser.add_argument("--output", type=pathlib.Path, required=True)
    parser.add_argument("--config-template", type=pathlib.Path, default=DEFAULT_TEMPLATE)
    parser.add_argument("--vlm-assets", type=pathlib.Path,
                        help="Existing local SmolVLM directory used by the checkpoint")
    parser.add_argument("--copy-vlm-assets", action="store_true",
                        help="Copy --vlm-assets into output/vlm; this can use about 2 GB")
    args = parser.parse_args()

    source = args.source.expanduser().resolve()
    output = args.output.expanduser().resolve()
    template = args.config_template.expanduser().resolve()
    if not source.is_file():
        raise SystemExit(f"Missing source checkpoint: {source}")
    if output.exists():
        raise SystemExit(f"Output exists; choose a new directory: {output}")
    if not template.is_file():
        raise SystemExit(f"Missing SmolVLA config template: {template}")
    if args.copy_vlm_assets and not args.vlm_assets:
        raise SystemExit("--copy-vlm-assets requires --vlm-assets")
    if args.vlm_assets and not (args.vlm_assets / "config.json").is_file():
        raise SystemExit(f"VLM assets must contain config.json: {args.vlm_assets}")

    # The pickle contains WindowsPath objects from the training machine.
    pathlib.WindowsPath = pathlib.PosixPath  # type: ignore[attr-defined]
    checkpoint = torch.load(source, map_location="cpu", weights_only=False)
    if not isinstance(checkpoint, dict) or not isinstance(checkpoint.get("model"), dict):
        raise SystemExit("Checkpoint must contain a model state dictionary")
    state = checkpoint["model"]
    if not state or any(not isinstance(key, str) or not torch.is_tensor(value) for key, value in state.items()):
        raise SystemExit("Checkpoint model must contain only named tensors")

    config = json.loads(template.read_text())
    features = config.get("input_features", {})
    expected = {"observation.image", "observation.state"}
    if set(features) != expected or features["observation.image"].get("shape") != [3, 240, 320]:
        raise SystemExit("Config template must describe RGB [3,240,320] and six-state input")
    if features["observation.state"].get("shape") != [6]:
        raise SystemExit("Config template must describe a six-dimensional state")
    config["type"] = "smolvla"
    config["chunk_size"] = 10
    config["n_action_steps"] = 1
    config["device"] = "cuda"
    if args.vlm_assets:
        vlm = args.output / "vlm" if args.copy_vlm_assets else args.vlm_assets.expanduser().resolve()
        config["vlm_model_name"] = str(vlm)

    output.mkdir(parents=True)
    try:
        tensors = {key: value.detach().cpu().contiguous() for key, value in state.items()}
        save_file(tensors, str(output / "model.safetensors"), metadata={
            "format": "edgepick_smolvla_from_pt_v1",
            "source": str(source),
            "step": str(checkpoint.get("step", "unknown")),
        })
        (output / "config.json").write_text(json.dumps(config, indent=2) + "\n")
        task = str(checkpoint.get("task_text", "抓取黄色正方体并放置到红色方形区域，使正方体覆盖区域至少一半"))
        contract = {
            "joint_names": ["Arm1_Joint", "Arm2_Joint", "Arm3_Joint", "Arm4_Joint", "Arm5_Joint", "grip_joint"],
            "action_units": "radian",
            "action_mode": "absolute_joint_position",
            "object": "cube",
            "visual_inputs": ["observation.image"],
            "image_shape": [3, 240, 320],
            "policy_depth_input": False,
            "depth_required_for_safety": True,
            "depth_encoding": "registered depth remains a Safety Gate/perception input; not consumed by this RGB-only policy",
            "observation": "registered RGB-D; RGB and joint positions match the training viewpoint and order",
            "chunk_size": 10,
            "n_action_steps": 1,
            "execution": "execute the first predicted absolute joint position, then reacquire RGB-D and replan",
            "task": task,
        }
        (output / "edgepick_contract.json").write_text(json.dumps(contract, ensure_ascii=False, indent=2) + "\n")
        metadata = {
            "source": str(source),
            "step": checkpoint.get("step"),
            "best_validation_loss": checkpoint.get("best_validation_loss"),
            "task_text": task,
            "training_seconds": checkpoint.get("training_seconds"),
            "args": portable(checkpoint.get("args", {})),
        }
        (output / "training_metadata.json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + "\n")
        if args.copy_vlm_assets:
            shutil.copytree(args.vlm_assets.expanduser().resolve(), output / "vlm")
    except Exception:
        shutil.rmtree(output, ignore_errors=True)
        raise

    print(output)
    print("Converted RGB-only cube SmolVLA checkpoint; depth remains mandatory for Safety Gate perception.")


if __name__ == "__main__":
    main()
