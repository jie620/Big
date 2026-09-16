#!/usr/bin/env python3
"""No model or renderer: check MuJoCo ROS actions, measured state and cancellation."""
import os
from pathlib import Path
import signal
import subprocess
import time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from action_msgs.msg import GoalStatus

root=Path(__file__).resolve().parents[1]
log_path=root/'artifacts/safe_integration_20260916/mujoco_backend.log';log_path.parent.mkdir(parents=True,exist_ok=True)
log=log_path.open('w')
process=subprocess.Popen([str(root/'.venv-vla/bin/python'),str(root/'install/edgepick_safe/lib/edgepick_safe/mujoco_backend'),'--ros-args','-p',f'model_xml:={root}/simulation/dofbot_pro/dofbot_pro.xml','-p','render:=false'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
rclpy.init();node=Node('mujoco_backend_check');states=[]
node.create_subscription(JointState,'/joint_states',states.append,10)
client=ActionClient(node,FollowJointTrajectory,'/arm_group_controller/follow_joint_trajectory')
def wait(predicate,seconds=15):
    deadline=time.monotonic()+seconds
    while time.monotonic()<deadline and not predicate():
        if process.poll() is not None:raise RuntimeError(log_path.read_text())
        rclpy.spin_once(node,timeout_sec=0.05)
    assert predicate(),'MuJoCo action/state timeout'
def send(delta,duration):
    request=FollowJointTrajectory.Goal();request.trajectory.joint_names=[f'Arm{i}_Joint' for i in range(1,6)]
    positions=list(states[-1].position[:5]);positions[0]+=delta
    request.trajectory.points=[JointTrajectoryPoint(positions=positions,time_from_start=Duration(sec=duration))]
    future=client.send_goal_async(request);wait(future.done);handle=future.result();assert handle.accepted
    return handle,positions
try:
    wait(lambda:states and client.server_is_ready(),20)
    assert len(states[-1].position)==6
    handle,target=send(0.03,1);result=handle.get_result_async();wait(result.done,10)
    assert result.result().status==GoalStatus.STATUS_SUCCEEDED
    assert max(abs(a-b) for a,b in zip(states[-1].position,target))<0.05
    handle,_=send(0.20,5);cancel=handle.cancel_goal_async();wait(cancel.done)
    result=handle.get_result_async();wait(result.done)
    assert result.result().status==GoalStatus.STATUS_CANCELED
    print('MuJoCo measured trajectory and cancellation checks passed')
finally:
    node.destroy_node()
    if rclpy.ok():rclpy.shutdown()
    if process.poll() is None:
        os.killpg(process.pid,signal.SIGINT)
        try:process.wait(timeout=5)
        except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
    log.close()
