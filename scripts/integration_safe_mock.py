#!/usr/bin/env python3
"""Run the actual MoveIt/mock-controller route and inject safety failures."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Header, Bool, String
from sensor_msgs.msg import JointState
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PointStamped, Pose
from std_srvs.srv import Trigger
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from edgepick_interfaces.msg import VLAAction

JOINTS=[f"Arm{i}_Joint" for i in range(1,6)]+["grip_joint"]
OUTPUT=Path(os.environ.get("EDGEPICK_TEST_OUTPUT","/tmp/edgepick_safe_acceptance"));OUTPUT.mkdir(parents=True,exist_ok=True)
log=(OUTPUT/"launch.log").open("w")
process=subprocess.Popen(["ros2","launch","edgepick_safe","safe_system.launch.py","mode:=mock","mock_inputs:=false"],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
rclpy.init();node=Node("safe_mock_acceptance")
pubs={"target":node.create_publisher(PointStamped,"/edgepick/safe/target",10),
      "scene":node.create_publisher(Header,"/edgepick/safe/scene_applied",10),
      "policy":node.create_publisher(Header,"/edgepick/safe/policy_alive",10),
      "estop":node.create_publisher(Bool,"/edgepick/safe/estop",QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)),
      "action":node.create_publisher(VLAAction,"/edgepick/vla/proposal",1)}
state={"q":None,"ready":False,"estop":False,"target":True,"scene":False,"policy":True,"seq":0}
events=[];checks=[];executing=[False]
node.create_subscription(GoalStatusArray,"/arm_group_controller/follow_joint_trajectory/_action/status",lambda m:executing.__setitem__(0,any(x.status==2 for x in m.status_list)),10)
node.create_subscription(String,"/edgepick/safe/event",lambda m:events.append(m.data),100)
node.create_subscription(Bool,"/edgepick/safe/ready",lambda m:state.update(ready=m.data),1)
def joints(m):
    values=dict(zip(m.name,m.position))
    if all(n in values for n in JOINTS):state["q"]=[values[n] for n in JOINTS]
node.create_subscription(JointState,"/joint_states",joints,10)
arm=node.create_client(Trigger,"/edgepick/safe/arm");apply=node.create_client(ApplyPlanningScene,"/edgepick/safe/apply_observation")
def heartbeat():
    header=Header(stamp=node.get_clock().now().to_msg(),frame_id="base_link")
    pubs["estop"].publish(Bool(data=state["estop"]))
    if state["policy"]:pubs["policy"].publish(header)
    if state["scene"]:pubs["scene"].publish(header)
    if state["target"]:
        msg=PointStamped(header=header);msg.point.y=0.2;msg.point.z=0.04;pubs["target"].publish(msg)
node.create_timer(0.1,heartbeat)
def wait(predicate,timeout=15):
    until=time.monotonic()+timeout
    while time.monotonic()<until and not predicate():
        if process.poll() is not None:raise RuntimeError("launch exited; inspect launch.log")
        rclpy.spin_once(node,timeout_sec=0.05)
    if not predicate():raise AssertionError(f"timeout; last events={events[-8:]}")
def call(client,request):
    wait(client.service_is_ready,100);future=client.call_async(request);wait(future.done);return future.result()
def arm_gate():
    # Gate may still be initializing its local scene monitor after service creation.
    until=time.monotonic()+90
    while time.monotonic()<until:
        response=call(arm,Trigger.Request())
        if response.success:wait(lambda:state["ready"]);return
        end=time.monotonic()+0.5
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=0.05)
    raise AssertionError(response.message)
def proposal(q=None,seq=None):
    state["seq"]+=1
    msg=VLAAction(header=Header(stamp=node.get_clock().now().to_msg()),joint_names=JOINTS,positions=q or list(state["q"]),sequence_id=seq or state["seq"],valid_for_ms=8000)
    pubs["action"].publish(msg)
def expect(name,operation,timeout=15):
    before=len(events);operation();wait(lambda:name in events[before:],timeout)
    checks.append({"expected":name,"passed":True});print(name,"PASS",flush=True)
try:
    wait(lambda:state["q"] is not None,100)
    until=time.monotonic()+100
    while not call(apply,ApplyPlanningScene.Request(scene=PlanningScene(is_diff=True))).success:
        if time.monotonic()>until:raise AssertionError("scene gate initialization timed out")
        end=time.monotonic()+0.5
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=0.05)
    state["scene"]=True
    arm_gate()
    bad=list(state["q"]);bad[0]=float("nan");expect("joint_limit",lambda:proposal(bad))
    bad=list(state["q"]);bad[0]=1.0;expect("action_jump",lambda:proposal(bad))
    q=list(state["q"]);q[0]+=0.04
    expect("proposal_completed",lambda:proposal(q),25)
    wait(lambda:state["ready"])
    expect("replayed_proposal",lambda:proposal(seq=state["seq"]))
    expect("emergency_stop",lambda:state.update(estop=True))
    state["estop"]=False
    end=time.monotonic()+0.4
    while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=0.05)
    assert not state["ready"],"estop clear must not automatically rearm"
    arm_gate();expect("target_lost",lambda:state.update(target=False),5)
    state["target"]=True
    arm_gate();expect("depth_timeout",lambda:state.update(scene=False),5)
    state["scene"]=True
    arm_gate();expect("policy_timeout",lambda:state.update(policy=False),13)
    state["policy"]=True
    arm_gate()
    # Block the full arm volume: no accepted execution may complete.
    obj=CollisionObject(id="test_block",header=Header(frame_id="base_link"),operation=CollisionObject.ADD)
    pose=Pose();pose.position.z=0.25;pose.orientation.w=1.
    obj.primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=[0.5,0.5,0.5])];obj.primitive_poses=[pose]
    scene=PlanningScene(is_diff=True);scene.robot_state.is_diff=True;scene.world.collision_objects=[obj]
    assert call(apply,ApplyPlanningScene.Request(scene=scene)).success
    expect("planning_or_execution_failed",lambda:proposal(),20)
    obj.operation=CollisionObject.REMOVE
    scene.world.collision_objects=[obj]
    assert call(apply,ApplyPlanningScene.Request(scene=scene)).success
    arm_gate()
    q=list(state["q"]);q[0]+=0.30
    proposal(q);wait(lambda:executing[0],10)
    obj.operation=CollisionObject.ADD
    scene.world.collision_objects=[obj]
    before=len(events)
    print("Injecting collision while controller executing:",executing[0],flush=True)
    assert call(apply,ApplyPlanningScene.Request(scene=scene)).success
    wait(lambda:"execution_cancelled_or_collision" in events[before:],10)
    checks.append({"expected":"execution_cancelled_or_collision","passed":True})
    print("execution_cancelled_or_collision PASS",flush=True)

finally:
    report={"checks":checks,"events":events,"scope":"mock only"}
    (OUTPUT/"results.json").write_text(json.dumps(report,indent=2)+"\n")
    node.destroy_node()
    if rclpy.ok():rclpy.shutdown()
    if process.poll() is None:
        os.killpg(process.pid,signal.SIGINT)
        try:process.wait(timeout=10)
        except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
    log.close()
print(f"{len(checks)} runtime checks passed; {OUTPUT}")
