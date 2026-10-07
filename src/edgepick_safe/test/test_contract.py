import json
import math
import sys
from pathlib import Path
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from edgepick_safe.contract import ACTION_LOWER, CHUNK_SIZE, IMAGE_SHAPE, JOINTS, N_ACTION_STEPS, VISUALS, checkpoint_contract, fresh, ordered_state, valid_action


def test_feedback_and_action_boundaries():
    assert ordered_state(JOINTS[::-1], [0]*6) == [0]*6
    for q in ([0]*5, [0]*5+[math.nan], [2]*6, [0]*5+[0.1]):
        with pytest.raises(ValueError): valid_action(q)
    assert valid_action([0, 0, 0, 0, 0, -1.25])[-1] == -1.25
    with pytest.raises(ValueError): valid_action([0, 0, 0, 0, 0, ACTION_LOWER[-1] - 1e-3])
    with pytest.raises(ValueError): ordered_state(JOINTS+JOINTS, [0]*12)
    assert not fresh(10,11,1) and not fresh(10,0,20) and fresh(10,9.5,1)


def test_checkpoint_rejects_wrong_semantics(tmp_path):
    config={"type":"smolvla","input_features":{"observation.state":{"shape":[6]},"observation.image":{"type":"VISUAL","shape":IMAGE_SHAPE},"observation.images.depth":{"type":"VISUAL","shape":IMAGE_SHAPE}},"output_features":{"action":{"shape":[6]}}}
    (tmp_path/"config.json").write_text(json.dumps(config))
    (tmp_path/"model.safetensors").touch()
    manifest={"joint_names":JOINTS,"action_units":"radian","action_mode":"absolute_joint_position","object":"cube","visual_inputs":VISUALS,"image_shape":IMAGE_SHAPE,"chunk_size":CHUNK_SIZE,"n_action_steps":N_ACTION_STEPS}
    (tmp_path/"edgepick_contract.json").write_text(json.dumps(manifest))
    path, loaded, contract = checkpoint_contract(tmp_path)
    assert path == tmp_path.resolve()
    assert loaded == config
    assert contract["visual_inputs"] == VISUALS
    assert config["input_features"]["observation.images.depth"]["shape"] == IMAGE_SHAPE
    assert contract["chunk_size"] == 10
    assert contract["n_action_steps"] == 1
    manifest["visual_inputs"] = ["observation.image"]
    (tmp_path/"edgepick_contract.json").write_text(json.dumps(manifest))
    with pytest.raises(ValueError): checkpoint_contract(tmp_path)
    manifest["visual_inputs"] = VISUALS
    manifest["image_shape"] = [3, 224, 224]
    (tmp_path/"edgepick_contract.json").write_text(json.dumps(manifest))
    with pytest.raises(ValueError): checkpoint_contract(tmp_path)
    manifest["action_units"]="degree"
    (tmp_path/"edgepick_contract.json").write_text(json.dumps(manifest))
    with pytest.raises(ValueError): checkpoint_contract(tmp_path)


def test_checkpoint_accepts_explicit_rgb_only_cube_policy(tmp_path):
    config = {
        "type": "smolvla",
        "input_features": {
            "observation.state": {"shape": [6]},
            "observation.image": {"type": "VISUAL", "shape": IMAGE_SHAPE},
        },
        "output_features": {"action": {"shape": [6]}},
    }
    (tmp_path / "config.json").write_text(json.dumps(config))
    (tmp_path / "model.safetensors").touch()
    manifest = {
        "joint_names": JOINTS,
        "action_units": "radian",
        "action_mode": "absolute_joint_position",
        "object": "cube",
        "visual_inputs": ["observation.image"],
        "image_shape": IMAGE_SHAPE,
        "chunk_size": CHUNK_SIZE,
        "n_action_steps": N_ACTION_STEPS,
    }
    (tmp_path / "edgepick_contract.json").write_text(json.dumps(manifest))
    _, _, loaded = checkpoint_contract(tmp_path)
    assert loaded["visual_inputs"] == ["observation.image"]
