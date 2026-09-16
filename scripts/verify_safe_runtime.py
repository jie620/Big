#!/usr/bin/env python3
"""Short ROS integration checks; run only with mode:=mock on an isolated domain."""
import argparse
import json
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

parser=argparse.ArgumentParser()
parser.add_argument("--expect",required=True)
parser.add_argument("--timeout",type=float,default=35)
parser.add_argument("--output",default="/tmp/edgepick_safe_runtime.json")
args=parser.parse_args()
rclpy.init();node=Node("verify_safe_runtime");events=[]
node.create_subscription(String,"/edgepick/safe/event",lambda m:events.append({"time":time.monotonic(),"event":m.data}),100)
deadline=time.monotonic()+args.timeout
while time.monotonic()<deadline and not any(x["event"]==args.expect for x in events):rclpy.spin_once(node,timeout_sec=0.1)
passed=any(x["event"]==args.expect for x in events)
with open(args.output,"w") as f:json.dump({"expected":args.expect,"passed":passed,"events":events},f,indent=2)
node.destroy_node();rclpy.shutdown()
print(json.dumps({"passed":passed,"expected":args.expect,"events":[x["event"] for x in events]}))
raise SystemExit(0 if passed else 1)
