"""The stage-0 model contract for cube RGB-D closed-loop control."""
import json
import math
from pathlib import Path

JOINTS = [f"Arm{i}_Joint" for i in range(1, 6)] + ["grip_joint"]
VISUALS = ["observation.image", "observation.images.depth"]
IMAGE_SHAPE = [3, 240, 320]
CHUNK_SIZE = 10
N_ACTION_STEPS = 1
# Feedback may cover the full URDF gripper range, while the trained policy and
# MuJoCo actuator only use [-1.25, 0] for gripper commands.
STATE_LOWER = [-math.pi / 2] * 5 + [-1.6]
ACTION_LOWER = [-math.pi / 2] * 5 + [-1.25]
UPPER = [math.pi / 2] * 4 + [math.pi, 0.0]


def checkpoint_contract(path):
    path = Path(path).expanduser().resolve()
    for name in ("config.json", "model.safetensors", "edgepick_contract.json"):
        if not (path / name).is_file():
            raise ValueError(f"Missing checkpoint file: {path / name}")
    config = json.loads((path / "config.json").read_text())
    contract = json.loads((path / "edgepick_contract.json").read_text())
    if contract.get("joint_names") != JOINTS or contract.get("action_units") != "radian" or contract.get("action_mode") != "absolute_joint_position":
        raise ValueError("Model joint ordering/units/action semantics do not match DOFBOT")
    accepted_visuals = (VISUALS, ["observation.image"])
    visual_inputs = contract.get("visual_inputs")
    if (contract.get("object") != "cube" or visual_inputs not in accepted_visuals or
            contract.get("image_shape") != IMAGE_SHAPE):
        raise ValueError("Checkpoint must be a cube RGB or RGB-D policy contract")
    if contract.get("chunk_size") != CHUNK_SIZE or contract.get("n_action_steps") != N_ACTION_STEPS:
        raise ValueError("Checkpoint must use chunk_size=10 and n_action_steps=1")
    features = config.get("input_features", {})
    if config.get("type") != "smolvla" or features.get("observation.state", {}).get("shape") != [6] or config.get("output_features", {}).get("action", {}).get("shape") != [6]:
        raise ValueError("Requires SmolVLA six-state/six-action checkpoint")
    visuals = [key for key, val in features.items() if val.get("type") == "VISUAL"]
    if visuals != visual_inputs:
        raise ValueError("Checkpoint manifest visual inputs do not match config")
    for key in visual_inputs:
        shape = features[key].get("shape")
        if shape != IMAGE_SHAPE:
            raise ValueError(f"Invalid {key} shape; expected {IMAGE_SHAPE}")
    return path, config, contract


def ordered_state(names, positions):
    if len(names) != len(positions) or len(set(names)) != len(names):
        raise ValueError("Joint feedback names/positions mismatch")
    values = dict(zip(names, positions))
    q = [float(values[name]) for name in JOINTS]
    if any(not math.isfinite(x) or x < low or x > high for x, low, high in zip(q, STATE_LOWER, UPPER)):
        raise ValueError("Invalid joint feedback")
    return q


def valid_action(action):
    if len(action) != 6 or any(not math.isfinite(float(x)) or x < lo or x > hi for x, lo, hi in zip(action, ACTION_LOWER, UPPER)):
        raise ValueError("Policy produced invalid absolute joint positions")
    return [float(x) for x in action]


def fresh(now, stamp, timeout):
    return math.isfinite(stamp) and stamp > 0 and 0 <= now - stamp <= timeout
