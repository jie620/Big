"""Inference only. This node has no motor, controller or MoveIt publisher."""
import json
import os
import sys
import tempfile
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Bool, Header, String
from edgepick_interfaces.msg import VLAAction
from .contract import JOINTS, checkpoint_contract, fresh, ordered_state, valid_action


class PolicyNode(Node):
    def __init__(self):
        super().__init__("safe_policy")
        checkpoint = self.declare_parameter("checkpoint", "").value
        self.task = self.declare_parameter("task", "把橘子放到右边").value
        source = self.declare_parameter("lerobot_source", "").value
        if source:
            sys.path.insert(0, str(Path(source).resolve()))
        self.device = self.declare_parameter("device", "cuda").value
        self.max_age = float(self.declare_parameter("observation_timeout", 1.0).value)
        self.ttl_ms = int(self.declare_parameter("proposal_ttl_ms", 8000).value)
        if not 100 <= self.ttl_ms <= 10000 or self.max_age <= 0:
            raise ValueError("invalid policy timing parameters")
        self.pub = self.create_publisher(VLAAction, "/edgepick/vla/proposal", 1)
        self.alive = self.create_publisher(Header, "/edgepick/safe/policy_alive", 1)
        self.rgb = self.joints = None
        self.ready = False
        self.sent = False
        self.seq = 0
        self.last_error = 0.0
        self.create_subscription(Image, self.declare_parameter("image_topic", "/camera/color/image_raw").value, self.on_image, qos_profile_sensor_data)
        self.create_subscription(JointState, "/joint_states", self.on_joints, qos_profile_sensor_data)
        self.create_subscription(Bool, "/edgepick/safe/ready", self.on_ready, 1)
        self.create_subscription(String, "/edgepick/safe/event", self.on_event, 10)
        self.policy, self.shape = self.load(checkpoint)
        self.create_timer(0.1, self.tick)
        self.get_logger().info("Model loaded; proposal-only mode, waiting for Safety Gate")

    def load(self, checkpoint):
        path, data, _ = checkpoint_contract(checkpoint)
        os.environ.setdefault("HF_HUB_OFFLINE", "1")
        os.environ.setdefault("TRANSFORMERS_OFFLINE", "1")
        import torch
        import draccus
        from lerobot.policies.smolvla.configuration_smolvla import SmolVLAConfig
        from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy
        if self.device == "cuda" and not torch.cuda.is_available():
            raise RuntimeError("CUDA requested but unavailable; choose device:=cpu explicitly")
        data.pop("type", None)
        with tempfile.NamedTemporaryFile(mode="w+", suffix=".json") as f:
            json.dump(data, f)
            f.flush()
            with draccus.config_type("json"):
                config = draccus.parse(SmolVLAConfig, f.name, args=[])
        config.device = self.device
        # A new observation is taken after each completed safe trajectory.
        config.n_action_steps = 1
        model = SmolVLAPolicy.from_pretrained(path, config=config, local_files_only=True, strict=True)
        model.eval()
        model.reset()
        return model, data["input_features"]["observation.image"]["shape"]

    def on_image(self, msg):
        self.rgb = msg

    def on_joints(self, msg):
        self.joints = msg

    def on_ready(self, msg):
        if not msg.data:
            self.sent = False
        self.ready = msg.data

    def on_event(self, msg):
        # Rejections do not enter busy state, so there may be no ready=False edge.
        if msg.data in {"joint_limit", "action_jump", "expired_proposal", "replayed_proposal",
                        "joint_order_mismatch", "invalid_action_shape"}:
            self.sent = False

    def stamp(self, header):
        return header.stamp.sec + header.stamp.nanosec * 1e-9

    def tick(self):
        import numpy as np
        import torch
        import cv2
        from cv_bridge import CvBridge
        now = self.get_clock().now()
        # Heartbeat proves that the inference process is responsive, even disarmed.
        self.alive.publish(Header(stamp=now.to_msg()))
        if not self.ready or self.sent or self.rgb is None or self.joints is None:
            return
        rgb, joints = self.rgb, self.joints
        acquired = min(self.stamp(rgb.header), self.stamp(joints.header))
        if not fresh(now.nanoseconds / 1e9, acquired, self.max_age) or abs(self.stamp(rgb.header)-self.stamp(joints.header)) > 0.2:
            return
        try:
            state = np.asarray(ordered_state(joints.name, joints.position), dtype=np.float32)
            frame = CvBridge().imgmsg_to_cv2(rgb, "rgb8")
            frame = cv2.resize(frame, (self.shape[2], self.shape[1]))
            batch = {"observation.state": torch.from_numpy(state).to(self.device)[None],
                     "observation.image": torch.from_numpy(frame.copy()).to(self.device, dtype=torch.float32).permute(2, 0, 1)[None] / 255,
                     "task": self.task}
            self.policy.reset()  # Never replay a queued chunk after a stop/replan.
            with torch.inference_mode():
                action = self.policy.select_action(batch).squeeze(0).cpu().numpy()
            values = valid_action(action)
            if not fresh(self.get_clock().now().nanoseconds/1e9, acquired, self.ttl_ms/1000):
                return
            self.seq += 1
            msg = VLAAction()
            msg.header.stamp = rclpy.time.Time(seconds=acquired).to_msg()
            msg.task = self.task
            msg.joint_names = JOINTS
            msg.positions = values
            msg.sequence_id = self.seq
            msg.valid_for_ms = self.ttl_ms
            self.pub.publish(msg)
            self.sent = True
        except (ValueError, KeyError, RuntimeError) as exc:
            if time.monotonic() - self.last_error > 2:
                self.get_logger().error(f"Proposal rejected: {exc}")
                self.last_error = time.monotonic()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PolicyNode()
        rclpy.spin(node)
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
