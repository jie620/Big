import sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from edgepick_safe.perception import destination_visible_clear


def test_destination_unknown_and_obstacle_are_not_clear():
    depth=np.ones((100,100),dtype=np.float32)
    k=[100,0,50,0,100,50,0,0,1]
    rotation=np.eye(3);translation=np.array([0.,0.,-1.])
    args=(k,rotation,translation,np.zeros(3),0.1,0.,0.015)
    assert destination_visible_clear(depth,*args)
    depth[48:53,48:53]=0
    assert not destination_visible_clear(depth,*args)
    depth[48:53,48:53]=0.8
    assert not destination_visible_clear(depth,*args)
    depth[:]=np.nan
    assert not destination_visible_clear(depth,*args)


def test_rejected_proposal_allows_new_observation():
    from types import SimpleNamespace
    from edgepick_safe.policy import PolicyNode
    state=SimpleNamespace(sent=True)
    PolicyNode.on_event(state,SimpleNamespace(data="action_jump"))
    assert not state.sent
    state.sent=True
    PolicyNode.on_event(state,SimpleNamespace(data="armed"))
    assert state.sent
