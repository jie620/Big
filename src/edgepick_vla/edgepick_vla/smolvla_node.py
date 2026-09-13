#!/usr/bin/env python3
"""ROS 2 bridge: RGB + joint state -> local SmolVLA -> safe joint trajectory."""
import json
import sys
import tempfile
from pathlib import Path

import numpy as np
import rclpy
import torch
from cv_bridge import CvBridge
from edgepick_interfaces.msg import VLAAction
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState, PointCloud2
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ROOT = Path(__file__).resolve().parents[3]
LEROBOT = ROOT.parent / "Edge_AI" / "lerobot-smolvla-jetson" / "src"
if str(LEROBOT) not in sys.path:
    sys.path.insert(0, str(LEROBOT))


class SmolVLARosNode(Node):
    def __init__(self):
        super().__init__("edgepick_smolvla")
        self.image_topic = self.declare_parameter("image_topic", "/camera/color/image_raw").value
        self.joints_topic = self.declare_parameter("joints_topic", "/joint_states").value
        self.pointcloud_topic = self.declare_parameter("pointcloud_topic", "/camera/depth/points").value
        self.task = self.declare_parameter("task", "抓取橘子").value
        checkpoint = Path(self.declare_parameter("checkpoint", str(ROOT.parent / "Edge_AI" / "smolvla_v3_360000" / "best_pretrained")).value)
        # Accept either the actual checkpoint directory or its model root.
        if not (checkpoint / "config.json").is_file() and (checkpoint / "best_pretrained" / "config.json").is_file():
            checkpoint = checkpoint / "best_pretrained"
        self.checkpoint = checkpoint
        self.rate_hz = max(1.0, float(self.declare_parameter("rate_hz", 10.0).value))
        self.max_age_ms = max(50, int(self.declare_parameter("max_observation_age_ms", 500).value))
        self.action_time = max(0.05, float(self.declare_parameter("action_time_sec", 0.2).value))
        self.execute_trajectory = bool(self.declare_parameter("execute_trajectory", False).value)
        self.joint_names = ["Arm1_Joint", "Arm2_Joint", "Arm3_Joint", "Arm4_Joint", "Arm5_Joint", "grip_joint"]
        self.bridge, self.image, self.joints, self.cloud = CvBridge(), None, None, None
        self.pub = self.create_publisher(VLAAction, "/edgepick/vla/action", 10)
        self.traj_pub = self.create_publisher(JointTrajectory, "/arm_group_controller/joint_trajectory", 10)
        self.image_sub = self.create_subscription(Image, self.image_topic, lambda m: setattr(self, "image", m), qos_profile_sensor_data)
        self.joint_sub = self.create_subscription(JointState, self.joints_topic, lambda m: setattr(self, "joints", m), qos_profile_sensor_data)
        self.cloud_sub = self.create_subscription(PointCloud2, self.pointcloud_topic, lambda m: setattr(self, "cloud", m), qos_profile_sensor_data)
        self.policy = self._load()
        self.seq = 0
        self.timer = self.create_timer(1.0 / self.rate_hz, self.tick)
        self.get_logger().info(f"SmolVLA ready: checkpoint={self.checkpoint}, task={self.task}")

    def _load(self):
        import draccus
        from lerobot.policies.smolvla.configuration_smolvla import SmolVLAConfig
        from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy
        data = json.loads((self.checkpoint / "config.json").read_text())
        data.pop("type", None)
        with tempfile.NamedTemporaryFile(mode="w+") as f:
            json.dump(data, f); f.flush()
            with draccus.config_type("json"):
                cfg = draccus.parse(SmolVLAConfig, f.name, args=[])
        cfg.device = "cuda" if torch.cuda.is_available() else "cpu"
        policy = SmolVLAPolicy.from_pretrained(self.checkpoint, config=cfg, local_files_only=True, strict=False)
        policy.reset(); return policy

    def tick(self):
        if self.image is None or self.joints is None:
            return
        now = self.get_clock().now()
        if (now - rclpy.time.Time.from_msg(self.image.header.stamp)).nanoseconds / 1e6 > self.max_age_ms:
            return
        values = dict(zip(self.joints.name, self.joints.position))
        if any(n not in values for n in self.joint_names):
            return
        state = np.asarray([values[n] for n in self.joint_names], dtype=np.float32)
        frame = self.bridge.imgmsg_to_cv2(self.image, desired_encoding="rgb8")
        dev = next(self.policy.parameters()).device
        image = torch.from_numpy(frame).to(dev, dtype=torch.float32).div(255).permute(2, 0, 1)[None]
        batch = {"observation.state": torch.from_numpy(state).to(dev)[None], "observation.image": image, "task": self.task}
        with torch.inference_mode():
            action = self.policy.select_action(batch).squeeze(0).detach().cpu().numpy()
        if action.shape != (6,) or not np.isfinite(action).all():
            self.get_logger().error("Rejected invalid VLA action"); return
        self.seq += 1
        msg = VLAAction(); msg.header = self.image.header; msg.task = self.task; msg.joint_names = self.joint_names; msg.positions = action.tolist(); msg.sequence_id = self.seq; msg.valid_for_ms = int(self.action_time * 1000)
        self.pub.publish(msg)
        if self.execute_trajectory:
            trajectory = JointTrajectory(); trajectory.header.stamp = self.get_clock().now().to_msg(); trajectory.joint_names = self.joint_names[:5]
            point = JointTrajectoryPoint(); point.positions = action[:5].tolist(); point.time_from_start.sec = int(self.action_time); point.time_from_start.nanosec = int((self.action_time % 1) * 1e9); trajectory.points = [point]
            self.traj_pub.publish(trajectory)


def main(args=None):
    rclpy.init(args=args); node = SmolVLARosNode()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
