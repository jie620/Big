"""The only supported model contract: absolute ROS joint positions in radians."""
import json
import math
from pathlib import Path

JOINTS = [f"Arm{i}_Joint" for i in range(1, 6)] + ["grip_joint"]
LOWER = [-math.pi / 2] * 5 + [-1.6]
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
    features = config.get("input_features", {})
    if config.get("type") != "smolvla" or features.get("observation.state", {}).get("shape") != [6] or config.get("output_features", {}).get("action", {}).get("shape") != [6]:
        raise ValueError("Requires SmolVLA six-state/six-action checkpoint")
    visuals = [key for key, val in features.items() if val.get("type") == "VISUAL"]
    if visuals != ["observation.image"]:
        raise ValueError("Requires a single observation.image camera")
    shape = features["observation.image"]["shape"]
    if len(shape) != 3 or shape[0] != 3 or any(type(n) is not int or n <= 0 for n in shape):
        raise ValueError("Invalid RGB image shape")
    return path, config, contract


def ordered_state(names, positions):
    if len(names) != len(positions) or len(set(names)) != len(names):
        raise ValueError("Joint feedback names/positions mismatch")
    values = dict(zip(names, positions))
    q = [float(values[name]) for name in JOINTS]
    if any(not math.isfinite(x) or x < low or x > high for x, low, high in zip(q, LOWER, UPPER)):
        raise ValueError("Invalid joint feedback")
    return q


def valid_action(action):
    if len(action) != 6 or any(not math.isfinite(float(x)) or x < lo or x > hi for x, lo, hi in zip(action, LOWER, UPPER)):
        raise ValueError("Policy produced invalid absolute joint positions")
    return [float(x) for x in action]


def fresh(now, stamp, timeout):
    return math.isfinite(stamp) and stamp > 0 and 0 <= now - stamp <= timeout
