#!/usr/bin/env python3

"""Run COCO object detection on the color image stream and publish EdgePick detections."""

from pathlib import Path

import cv2 as cv
import rclpy
from cv_bridge import CvBridge
from edgepick_interfaces.msg import TargetDetection, TargetDetectionArray
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


DEFAULT_MODEL_PATH = "/home/jetson/dofbot_pro_ws/src/dofbot_pro_vision/config/frozen_inference_graph.pb"
DEFAULT_CONFIG_PATH = "/home/jetson/dofbot_pro_ws/src/dofbot_pro_vision/config/ssd_mobilenet_v2_coco.txt"
DEFAULT_LABEL_PATH = "/home/jetson/dofbot_pro_ws/src/dofbot_pro_vision/config/object_detection_coco.txt"


class CocoDetectorNode(Node):
    def __init__(self):
        super().__init__("edgepick_coco_detector")

        self.image_topic = self.declare_parameter("image_topic", "/camera/color/image_raw").value
        self.detections_topic = self.declare_parameter(
            "detections_topic", "/edgepick/perception/detections"
        ).value
        self.model_path = self.declare_parameter("model_path", DEFAULT_MODEL_PATH).value
        self.config_path = self.declare_parameter("config_path", DEFAULT_CONFIG_PATH).value
        self.label_path = self.declare_parameter("label_path", DEFAULT_LABEL_PATH).value
        self.target_label = self.declare_parameter("target_label", "orange").value
        self.frame_id_override = self.declare_parameter("frame_id", "").value
        self.conf_threshold = float(self.declare_parameter("conf_threshold", 0.4).value)
        self.max_detections = int(self.declare_parameter("max_detections", 20).value)
        self.publish_empty_frames = bool(
            self.declare_parameter("publish_empty_frames", True).value
        )

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(TargetDetectionArray, self.detections_topic, 10)
        self.subscription = self.create_subscription(
            Image, self.image_topic, self.on_image, qos_profile_sensor_data
        )
        self.class_names = self._load_class_names(Path(self.label_path))
        self.model = self._load_model(Path(self.model_path), Path(self.config_path))

        self.get_logger().info(
            f"COCO detector listening on '{self.image_topic}' and publishing "
            f"'{self.detections_topic}' with target_label='{self.target_label}'."
        )

    def _load_class_names(self, label_path: Path) -> list[str]:
        if not label_path.exists():
            self.get_logger().error(f"COCO label file does not exist: {label_path}")
            return []
        return [line.strip() for line in label_path.read_text(encoding="utf-8").splitlines() if line.strip()]

    def _load_model(self, model_path: Path, config_path: Path):
        if not model_path.exists():
            self.get_logger().error(f"COCO model path does not exist: {model_path}")
            return None
        if not config_path.exists():
            self.get_logger().error(f"COCO config path does not exist: {config_path}")
            return None
        try:
            return cv.dnn.readNet(
                model=str(model_path), config=str(config_path), framework="TensorFlow"
            )
        except Exception as exc:  # pragma: no cover - runtime model load path.
            self.get_logger().error(f"Failed to load COCO detector model: {exc}")
            return None

    def on_image(self, message: Image) -> None:
        detections = TargetDetectionArray()
        detections.header.stamp = message.header.stamp
        detections.header.frame_id = self.frame_id_override or message.header.frame_id

        if self.model is None or not self.class_names:
            if self.publish_empty_frames:
                self.publisher.publish(detections)
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            height, width = frame.shape[:2]
            blob = cv.dnn.blobFromImage(
                image=frame, size=(300, 300), mean=(104, 117, 123), swapRB=True
            )
            self.model.setInput(blob)
            output = self.model.forward()
            detections.detections = self._convert_output(output, width, height)
        except Exception as exc:  # pragma: no cover - runtime prediction path.
            self.get_logger().error(f"COCO inference failed: {exc}")
            if self.publish_empty_frames:
                self.publisher.publish(detections)
            return

        self.publisher.publish(detections)

    def _convert_output(self, output, image_width: int, image_height: int) -> list[TargetDetection]:
        candidates: list[TargetDetection] = []
        if output is None or len(output) == 0:
            return candidates

        for detection in output[0, 0, :, :]:
            confidence = float(detection[2])
            if confidence < self.conf_threshold:
                continue

            class_id = int(detection[1])
            class_index = class_id - 1
            if class_index < 0 or class_index >= len(self.class_names):
                continue
            label = self.class_names[class_index]
            if self.target_label and label != self.target_label:
                continue

            x_min = float(detection[3]) * image_width
            y_min = float(detection[4]) * image_height
            x_max = float(detection[5]) * image_width
            y_max = float(detection[6]) * image_height

            message = TargetDetection()
            message.class_id = class_id
            message.label = label
            message.score = confidence
            message.center_u = (x_min + x_max) * 0.5
            message.center_v = (y_min + y_max) * 0.5
            message.size_u = max(0.0, x_max - x_min)
            message.size_v = max(0.0, y_max - y_min)
            candidates.append(message)

            if len(candidates) >= self.max_detections:
                break

        return candidates


def main(args=None):
    rclpy.init(args=args)
    node = CocoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
