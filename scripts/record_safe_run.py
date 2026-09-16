#!/usr/bin/env python3
"""Record task outcomes with explicit timeout/failure denominators."""
import argparse
import json
from pathlib import Path
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
p=argparse.ArgumentParser();p.add_argument("--output",type=Path,required=True);p.add_argument("--seconds",type=float,default=120)
p.add_argument("--scope",choices=["mock","mujoco","real"],required=True);p.add_argument("--model",required=True)
a=p.parse_args()
if a.output.exists():p.error("refusing to overwrite previous run")
rclpy.init();node=Node("safe_run_recorder");events=[];states=[];sim=[]
node.create_subscription(String,"/edgepick/safe/event",lambda m:events.append({"elapsed":time.monotonic()-start,"event":m.data}),100)
node.create_subscription(String,"/edgepick/safe/state",lambda m:states.append(m.data),10)
node.create_subscription(String,"/edgepick/sim/event",lambda m:sim.append(m.data),100)
start=time.monotonic()
try:
    while time.monotonic()-start<a.seconds and "succeeded" not in states and "fault" not in states:rclpy.spin_once(node,timeout_sec=0.1)
except KeyboardInterrupt:pass
finally:
    report={"scope":a.scope,"model":a.model,"elapsed":time.monotonic()-start,"success":"succeeded" in states,
            "outcome":"success" if "succeeded" in states else "fault" if "fault" in states else "timeout_or_interrupted",
            "events":events,"simulation_attachment_events":sim}
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(report,indent=2)+"\n")
    node.destroy_node()
    if rclpy.ok():rclpy.shutdown()
print(json.dumps(report,indent=2))
