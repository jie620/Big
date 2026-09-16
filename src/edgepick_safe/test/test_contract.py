import json
import math
import sys
from pathlib import Path
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from edgepick_safe.contract import JOINTS, checkpoint_contract, fresh, ordered_state, valid_action


def test_feedback_and_action_boundaries():
    assert ordered_state(JOINTS[::-1], [0]*6) == [0]*6
    for q in ([0]*5, [0]*5+[math.nan], [2]*6, [0]*5+[0.1]):
        with pytest.raises(ValueError): valid_action(q)
    with pytest.raises(ValueError): ordered_state(JOINTS+JOINTS, [0]*12)
    assert not fresh(10,11,1) and not fresh(10,0,20) and fresh(10,9.5,1)


def test_checkpoint_rejects_wrong_semantics(tmp_path):
    config={"type":"smolvla","input_features":{"observation.state":{"shape":[6]},"observation.image":{"type":"VISUAL","shape":[3,240,320]}},"output_features":{"action":{"shape":[6]}}}
    (tmp_path/"config.json").write_text(json.dumps(config))
    (tmp_path/"model.safetensors").touch()
    manifest={"joint_names":JOINTS,"action_units":"radian","action_mode":"absolute_joint_position"}
    (tmp_path/"edgepick_contract.json").write_text(json.dumps(manifest))
    assert checkpoint_contract(tmp_path)[1]==config
    manifest["action_units"]="degree"
    (tmp_path/"edgepick_contract.json").write_text(json.dumps(manifest))
    with pytest.raises(ValueError): checkpoint_contract(tmp_path)
