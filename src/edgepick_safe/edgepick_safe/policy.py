"""Inference only. This node has no motor, controller or MoveIt publisher."""
import json
import os
import sys
import tempfile
import threading
import time
from contextlib import nullcontext
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Bool, Header, String
from edgepick_interfaces.msg import VLAAction
from .contract import CHUNK_SIZE, JOINTS, N_ACTION_STEPS, checkpoint_contract, fresh, ordered_state, valid_action


class PolicyNode(Node):
    def __init__(self):
        super().__init__("safe_policy")
        checkpoint = self.declare_parameter("checkpoint", "").value
        self.task = self.declare_parameter("task", "抓取方块，绕开障碍物，放置到红色方形区域").value
        source = self.declare_parameter("lerobot_source", "").value
        if source:
            sys.path.insert(0, str(Path(source).resolve()))
        self.device = self.declare_parameter("device", "cuda").value
        self.use_amp = bool(self.declare_parameter("use_amp", False).value)
        self.max_age = float(self.declare_parameter("observation_timeout", 1.0).value)
        self.ttl_ms = int(self.declare_parameter("proposal_ttl_ms", 8000).value)
        if not 100 <= self.ttl_ms <= 10000 or self.max_age <= 0:
            raise ValueError("invalid policy timing parameters")
        self.pub = self.create_publisher(VLAAction, "/edgepick/vla/proposal", 1)
        self.alive = self.create_publisher(Header, "/edgepick/safe/policy_alive", 1)
        self._state_lock = threading.Lock()
        self._pending_condition = threading.Condition(self._state_lock)
        self._pending = None
        self._last_snapshot = None
        self._generation = 0
        self._stop_worker = False
        self.visual_inputs = None
        self.rgb = self.depth = self.joints = None
        self.ready = False
        self.sent = False
        self.seq = 0
        self.last_error = 0.0
        self.create_subscription(Image, self.declare_parameter("image_topic", "/camera/color/image_raw").value, self.on_image, qos_profile_sensor_data)
        self.create_subscription(Image, self.declare_parameter("depth_topic", "/camera/aligned_depth_to_color/image_raw").value, self.on_depth, qos_profile_sensor_data)
        self.create_subscription(JointState, "/joint_states", self.on_joints, qos_profile_sensor_data)
        self.create_subscription(Bool, "/edgepick/safe/ready", self.on_ready, 1)
        self.create_subscription(String, "/edgepick/safe/event", self.on_event, 10)
        self.policy, self.shape = self.load(checkpoint)
        import cv2
        from cv_bridge import CvBridge
        self._cv2 = cv2
        self._bridge = CvBridge()
        self._worker = threading.Thread(target=self._inference_loop, name="smolvla-inference")
        self._worker.start()
        self.create_timer(0.1, self.tick)
        self.get_logger().info("Model loaded; proposal-only mode, waiting for Safety Gate")

    def load(self, checkpoint):
        path, data, contract = checkpoint_contract(checkpoint)
        self.visual_inputs = tuple(contract["visual_inputs"])
        os.environ.setdefault("HF_HUB_OFFLINE", "1")
        os.environ.setdefault("TRANSFORMERS_OFFLINE", "1")
        import torch
        import draccus
        from lerobot.policies.smolvla.configuration_smolvla import SmolVLAConfig
        from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy
        if str(self.device).startswith("cuda") and not torch.cuda.is_available():
            raise RuntimeError("CUDA requested but unavailable; choose device:=cpu explicitly")
        data.pop("type", None)
        with tempfile.NamedTemporaryFile(mode="w+", suffix=".json") as f:
            json.dump(data, f)
            f.flush()
            with draccus.config_type("json"):
                config = draccus.parse(SmolVLAConfig, f.name, args=[])
        config.device = self.device
        # A new observation is taken after each completed safe trajectory.
        if contract["chunk_size"] != CHUNK_SIZE or contract["n_action_steps"] != N_ACTION_STEPS:
            raise ValueError("Checkpoint contract must use chunk_size=10 and n_action_steps=1")
        config.chunk_size = CHUNK_SIZE
        config.n_action_steps = N_ACTION_STEPS
        model = SmolVLAPolicy.from_pretrained(path, config=config, local_files_only=True, strict=True)
        model.eval()
        model.reset()
        self._torch = torch
        return model, data["input_features"]["observation.image"]["shape"]

    def on_image(self, msg):
        with (getattr(self, "_state_lock", None) or nullcontext()):
            self.rgb = msg

    def on_depth(self, msg):
        with (getattr(self, "_state_lock", None) or nullcontext()):
            self.depth = msg

    def on_joints(self, msg):
        with (getattr(self, "_state_lock", None) or nullcontext()):
            self.joints = msg

    def on_ready(self, msg):
        with (getattr(self, "_pending_condition", None) or nullcontext()):
            changed = self.ready != msg.data
            if not msg.data:
                self.sent = False
                self._pending = None
            if changed:
                self._generation = getattr(self, "_generation", 0) + 1
            self.ready = msg.data
            condition = getattr(self, "_pending_condition", None)
            if condition is not None:
                condition.notify_all()

    def on_event(self, msg):
        # Rejections do not enter busy state, so there may be no ready=False edge.
        with (getattr(self, "_pending_condition", None) or nullcontext()):
            self._generation = getattr(self, "_generation", 0) + 1
            # A safety event invalidates any snapshot waiting for inference.
            # An in-flight call is rejected by the generation check below.
            if hasattr(self, "_pending"):
                self._pending = None
            if msg.data in {"joint_limit", "action_jump", "expired_proposal", "replayed_proposal",
                            "joint_order_mismatch", "invalid_action_shape"}:
                self.sent = False
            condition = getattr(self, "_pending_condition", None)
            if condition is not None:
                condition.notify_all()

    def stamp(self, header):
        return header.stamp.sec + header.stamp.nanosec * 1e-9

    def tick(self):
        now = self.get_clock().now()
        # Heartbeat proves that the inference process is responsive, even disarmed.
        self.alive.publish(Header(stamp=now.to_msg()))
        with self._pending_condition:
            if not self.ready or self.sent:
                return
            if self.rgb is None or self.depth is None or self.joints is None:
                return
            rgb, depth, joints = self.rgb, self.depth, self.joints
            rgb_at, depth_at, joints_at = (self.stamp(rgb.header), self.stamp(depth.header),
                                            self.stamp(joints.header))
            acquired = min(rgb_at, depth_at, joints_at)
            if not fresh(now.nanoseconds / 1e9, acquired, self.max_age):
                return
            if (abs(rgb_at - depth_at) > 0.1 or abs(rgb_at - joints_at) > 0.2 or
                    rgb.header.frame_id != depth.header.frame_id or
                    (rgb.width, rgb.height) != (depth.width, depth.height)):
                return
            # One bounded slot: a camera frame that arrives while inference runs
            # replaces the waiting frame instead of extending inference latency.
            snapshot = (id(rgb), id(depth), id(joints))
            if snapshot == self._last_snapshot:
                return
            self._last_snapshot = snapshot
            self._pending = ((rgb, depth, joints, acquired), self._generation)
            self._pending_condition.notify()

    def _inference_loop(self):
        while True:
            with self._pending_condition:
                while self._pending is None and not self._stop_worker:
                    self._pending_condition.wait(timeout=0.5)
                if self._stop_worker:
                    return
                (rgb, depth, joints, acquired), generation = self._pending
                self._pending = None
            try:
                batch = self._prepare_batch(rgb, depth, joints)
                self.policy.reset()  # Never replay a queued chunk after a stop/replan.
                with self._torch.inference_mode():
                    if self.use_amp and str(self.device).startswith("cuda"):
                        inference_context = self._torch.autocast(
                            device_type="cuda", dtype=self._torch.float16
                        )
                    else:
                        inference_context = nullcontext()
                    with inference_context:
                        action = self.policy.select_action(batch).squeeze(0).float().cpu().numpy()
                values = valid_action(action)
                with self._pending_condition:
                    if (self._stop_worker or generation != self._generation or not self.ready or self.sent):
                        continue
                    now = self.get_clock().now().nanoseconds / 1e9
                    latest = (self.rgb, self.depth, self.joints)
                    newer_frame = False
                    if all(message is not None for message in latest):
                        newer_frame = any(
                            self.stamp(message.header) > old_stamp
                            for message, old_stamp in zip(
                                latest,
                                (self.stamp(rgb.header), self.stamp(depth.header), self.stamp(joints.header)),
                            )
                        )
                    if (newer_frame or
                            not fresh(now, acquired, min(self.max_age, self.ttl_ms / 1000))):
                        continue
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
            except Exception as exc:
                if self._stop_worker:
                    return
                self._report_error(exc)

    def _prepare_batch(self, rgb, depth, joints):
        import numpy as np
        torch = self._torch
        state = np.asarray(ordered_state(joints.name, joints.position), dtype=np.float32)
        frame = self._bridge.imgmsg_to_cv2(rgb, "rgb8")
        frame = self._cv2.resize(frame, (self.shape[2], self.shape[1]), interpolation=self._cv2.INTER_AREA)
        frame = np.ascontiguousarray(frame)
        transfer_async = str(self.device).startswith("cuda")
        state_tensor = torch.from_numpy(state).to(self.device, non_blocking=transfer_async)[None]
        rgb_tensor = torch.from_numpy(frame).to(self.device, dtype=torch.float32, non_blocking=transfer_async).permute(2, 0, 1)[None] / 255
        batch = {"observation.state": state_tensor,
                 "observation.image": rgb_tensor,
                 "task": self.task}
        if "observation.images.depth" in self.visual_inputs:
            depth_frame = self._bridge.imgmsg_to_cv2(depth)
            if depth.encoding == "16UC1":
                depth_frame = depth_frame.astype(np.float32) * 0.001
            elif depth.encoding == "32FC1":
                depth_frame = depth_frame.astype(np.float32)
            else:
                raise ValueError(f"Unsupported depth encoding: {depth.encoding}")
            if (depth_frame.ndim != 2 or
                    np.count_nonzero(np.isfinite(depth_frame) & (depth_frame > 0.05) & (depth_frame < 1.5)) < depth_frame.size * 0.25):
                raise ValueError("Insufficient metric depth coverage")
            depth_frame = np.nan_to_num(depth_frame, nan=1.5, posinf=1.5, neginf=0.0)
            depth_frame = np.clip(depth_frame / 1.5, 0.0, 1.0)
            depth_frame = self._cv2.resize(depth_frame, (self.shape[2], self.shape[1]), interpolation=self._cv2.INTER_NEAREST)
            depth_frame = np.ascontiguousarray(depth_frame, dtype=np.float32)
            depth_tensor = torch.from_numpy(depth_frame).to(
                self.device, dtype=torch.float32, non_blocking=transfer_async
            )[None, None]
            # Expand preserves the three-channel contract without making three CPU copies.
            depth_tensor = depth_tensor.expand(-1, 3, -1, -1)
            batch["observation.images.depth"] = depth_tensor
        return batch

    def _report_error(self, exc):
        if time.monotonic() - self.last_error > 2:
            self.get_logger().error(f"Proposal rejected: {exc}")
            self.last_error = time.monotonic()

    def destroy_node(self):
        if hasattr(self, "_pending_condition"):
            with self._pending_condition:
                self._stop_worker = True
                self._pending = None
                self._pending_condition.notify_all()
            if hasattr(self, "_worker") and self._worker.is_alive():
                # Do not let a background worker publish through a partially
                # destroyed ROS node. Inference is finite and the worker is
                # the only owner of the model, so a complete join is safe.
                self._worker.join()
        return super().destroy_node()


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
