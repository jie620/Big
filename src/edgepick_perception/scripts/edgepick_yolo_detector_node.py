#!/usr/bin/env python3

"""Run YOLO detection on the color image stream and publish EdgePick detections."""

from pathlib import Path

import rclpy
from cv_bridge import CvBridge
from edgepick_interfaces.msg import TargetDetection, TargetDetectionArray
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

try:
    from ultralytics import YOLO
except Exception as exc:  # pragma: no cover - import failure is runtime only.
    YOLO = None
    YOLO_IMPORT_ERROR = exc
else:
    YOLO_IMPORT_ERROR = None


DEFAULT_MODEL_PATH = (
    "/home/jetson/dofbot_pro_ws/src/"
    "dofbot_pro_yolov11/dofbot_pro_yolov11/best.engine"
)


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__("edgepick_yolo_detector")

        self.image_topic = self.declare_parameter(
            "image_topic", "/camera/color/image_raw"
        ).value
        self.detections_topic = self.declare_parameter(
            "detections_topic", "/edgepick/perception/detections"
        ).value
        self.model_path = self.declare_parameter("model_path", DEFAULT_MODEL_PATH).value
        self.frame_id_override = self.declare_parameter("frame_id", "").value
        self.conf_threshold = float(self.declare_parameter("conf_threshold", 0.25).value)
        self.iou_threshold = float(self.declare_parameter("iou_threshold", 0.7).value)
        self.max_det = int(self.declare_parameter("max_det", 20).value)
        self.publish_empty_frames = bool(
            self.declare_parameter("publish_empty_frames", True).value
        )

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(TargetDetectionArray, self.detections_topic, 10)
        self.subscription = self.create_subscription(
            Image, self.image_topic, self.on_image, qos_profile_sensor_data
        )
        self.model = self._load_model()
        self._prediction_error_logged = False

        self.get_logger().info(
            f"YOLO detector listening on '{self.image_topic}' and publishing "
            f"'{self.detections_topic}'."
        )

    def _load_model(self):
        if YOLO is None:
            self.get_logger().error(f"ultralytics import failed: {YOLO_IMPORT_ERROR}")
            return None

        model_path = Path(self.model_path)
        if not model_path.exists():
            self.get_logger().error(f"YOLO model path does not exist: {model_path}")
            return None

        try:
            return YOLO(str(model_path), task="detect")
        except Exception as exc:  # pragma: no cover - runtime model load path.
            self.get_logger().error(f"Failed to load YOLO model '{model_path}': {exc}")
            return None

    def on_image(self, message: Image) -> None:
        detections = TargetDetectionArray()
        detections.header.stamp = message.header.stamp
        detections.header.frame_id = self.frame_id_override or message.header.frame_id

        if self.model is None:
            if self.publish_empty_frames:
                self.publisher.publish(detections)
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            results = self.model.predict(
                source=frame,
                verbose=False,
                conf=self.conf_threshold,
                iou=self.iou_threshold,
                max_det=self.max_det,
            )
            detections.detections = self._convert_results(results[0])
        except Exception as exc:  # pragma: no cover - runtime prediction path.
            if not self._prediction_error_logged:
                self.get_logger().error(f"YOLO inference failed: {exc}")
                self._prediction_error_logged = True
            if self.publish_empty_frames:
                self.publisher.publish(detections)
            return

        self.publisher.publish(detections)

    def _convert_results(self, result) -> list[TargetDetection]:
        boxes = getattr(result, "boxes", None)
        if boxes is None:
            return []

        names = getattr(result, "names", None) or {}
        output: list[TargetDetection] = []
        for box in boxes:
            detection = TargetDetection()
            detection.class_id = int(_scalar_value(box.cls))
            detection.label = str(names.get(detection.class_id, detection.class_id))
            detection.score = float(_scalar_value(box.conf))

            x1, y1, x2, y2 = [float(value) for value in box.xyxy[0].tolist()]
            detection.center_u = (x1 + x2) * 0.5
            detection.center_v = (y1 + y2) * 0.5
            detection.size_u = max(0.0, x2 - x1)
            detection.size_v = max(0.0, y2 - y1)
            output.append(detection)
        return output


def _scalar_value(value) -> float:
    if hasattr(value, "item"):
        return float(value.item())
    return float(value)


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
