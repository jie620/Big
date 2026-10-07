import sys
import threading
from pathlib import Path
from types import SimpleNamespace
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from edgepick_safe.perception import destination_visible_clear, select_synchronized_pair


def _image_stamp(seconds):
    from types import SimpleNamespace
    whole = int(seconds)
    return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(
        sec=whole, nanosec=int(round((seconds - whole) * 1e9)))))


def test_select_synchronized_pair_prefers_newest_complete_pair():
    rgb = [_image_stamp(1.00), _image_stamp(2.00), _image_stamp(3.00)]
    depth = [_image_stamp(1.01), _image_stamp(2.40), _image_stamp(2.99)]
    selected = select_synchronized_pair(rgb, depth, 0.05)
    assert selected is not None
    assert selected[0] is rgb[2]
    assert selected[1] is depth[2]
    assert selected[2] == 2.99


def test_select_synchronized_pair_rejects_skewed_streams():
    assert select_synchronized_pair([_image_stamp(1.0)], [_image_stamp(1.2)], 0.05) is None


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
    from edgepick_safe.policy import PolicyNode
    state=SimpleNamespace(sent=True)
    PolicyNode.on_event(state,SimpleNamespace(data="action_jump"))
    assert not state.sent
    state.sent=True
    PolicyNode.on_event(state,SimpleNamespace(data="armed"))
    assert state.sent


def test_policy_callbacks_keep_latest_messages_and_invalidate_worker_generation():
    from edgepick_safe.policy import PolicyNode
    state = SimpleNamespace(
        _state_lock=threading.Lock(),
        _pending_condition=None,
        _pending=None,
        _generation=0,
        ready=False,
        sent=False,
        rgb=None,
        depth=None,
        joints=None,
    )
    state._pending_condition = threading.Condition(state._state_lock)
    first = SimpleNamespace(header=SimpleNamespace(stamp="first"))
    second = SimpleNamespace(header=SimpleNamespace(stamp="second"))
    PolicyNode.on_image(state, first)
    PolicyNode.on_image(state, second)
    assert state.rgb is second

    PolicyNode.on_ready(state, SimpleNamespace(data=True))
    ready_generation = state._generation
    assert state.ready and ready_generation == 1
    PolicyNode.on_event(state, SimpleNamespace(data="planning_retry"))
    assert state._generation == ready_generation + 1
    PolicyNode.on_ready(state, SimpleNamespace(data=False))
    assert not state.ready and not state.sent and state._pending is None
    assert state._generation == ready_generation + 2


def test_policy_event_rejection_clears_sent_without_touching_other_state():
    from edgepick_safe.policy import PolicyNode
    state = SimpleNamespace(
        _state_lock=threading.Lock(),
        _pending_condition=None,
        _pending=object(),
        _generation=4,
        ready=True,
        sent=True,
    )
    state._pending_condition = threading.Condition(state._state_lock)
    PolicyNode.on_event(state, SimpleNamespace(data="action_jump"))
    assert state.sent is False
    assert state._pending is None
    assert state._generation == 5
