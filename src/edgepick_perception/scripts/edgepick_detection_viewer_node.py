#!/usr/bin/env python3

"""Display the camera stream with the latest EdgePick detection boxes overlaid."""

from threading import Lock

import cv2 as cv
import rclpy
from cv_bridge import CvBridge
from edgepick_interfaces.msg import TargetDetectionArray
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class DetectionViewerNode(Node):
    def __init__(self):
        super().__init__("edgepick_detection_viewer")

        self.image_topic = self.declare_parameter("image_topic", "/camera/color/image_raw").value
        self.detections_topic = self.declare_parameter(
            "detections_topic", "/edgepick/perception/detections"
        ).value
        self.window_name = self.declare_parameter("window_name", "edgepick_detection").value
        self.frame_scale = float(self.declare_parameter("frame_scale", 1.0).value)

        self.bridge = CvBridge()
        self.lock = Lock()
        self.latest_detections = None

        self.create_subscription(
            TargetDetectionArray,
            self.detections_topic,
            self.on_detections,
            10,
        )
        self.create_subscription(
            Image,
            self.image_topic,
            self.on_image,
            qos_profile_sensor_data,
        )

        cv.namedWindow(self.window_name, cv.WINDOW_NORMAL)
        self.get_logger().info(
            f"Detection viewer subscribed to '{self.image_topic}' and "
            f"'{self.detections_topic}'."
        )

    def on_detections(self, message: TargetDetectionArray) -> None:
        with self.lock:
            self.latest_detections = message

    def on_image(self, message: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except Exception as exc:  # pragma: no cover - runtime GUI path.
            self.get_logger().error(f"Failed to convert image: {exc}")
            return

        with self.lock:
            detections = self.latest_detections

        if detections is not None:
            for detection in detections.detections:
                self._draw_detection(frame, detection)

        if self.frame_scale != 1.0:
            width = max(1, int(frame.shape[1] * self.frame_scale))
            height = max(1, int(frame.shape[0] * self.frame_scale))
            frame = cv.resize(frame, (width, height))

        cv.imshow(self.window_name, frame)
        cv.waitKey(1)

    def _draw_detection(self, frame, detection) -> None:
        half_w = max(1.0, float(detection.size_u) * 0.5)
        half_h = max(1.0, float(detection.size_v) * 0.5)
        center_x = float(detection.center_u)
        center_y = float(detection.center_v)
        x1 = int(round(center_x - half_w))
        y1 = int(round(center_y - half_h))
        x2 = int(round(center_x + half_w))
        y2 = int(round(center_y + half_h))

        color = (0, 165, 255)
        cv.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        label = f"{detection.label} {float(detection.score):.2f}"
        cv.putText(
            frame,
            label,
            (x1, max(20, y1 - 8)),
            cv.FONT_HERSHEY_SIMPLEX,
            0.6,
            color,
            2,
        )
        cv.circle(frame, (int(round(center_x)), int(round(center_y))), 3, color, -1)


def main(args=None):
    rclpy.init(args=args)
    node = DetectionViewerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
