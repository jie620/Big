"""Start YOLO detection and convert detections into the EdgePick perception contract."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


DEFAULT_MODEL_PATH = (
    "/home/jetson/dofbot_pro_ws/src/"
    "dofbot_pro_yolov11/dofbot_pro_yolov11/best.engine"
)


def generate_launch_description():
    detections_topic = "/edgepick/perception/detections"
    target_topic = "/edgepick/perception/target_point"
    event_topic = "/edgepick/task/event"

    yolo_detector = Node(
        package="edgepick_perception",
        executable="edgepick_yolo_detector_node.py",
        name="edgepick_yolo_detector",
        output="screen",
        parameters=[
            {
                "image_topic": LaunchConfiguration("image_topic"),
                "detections_topic": detections_topic,
                "model_path": LaunchConfiguration("model_path"),
                "frame_id": LaunchConfiguration("frame_id"),
                "conf_threshold": LaunchConfiguration("conf_threshold"),
                "iou_threshold": LaunchConfiguration("iou_threshold"),
                "max_det": LaunchConfiguration("max_det"),
                "publish_empty_frames": LaunchConfiguration("publish_empty_frames"),
            }
        ],
    )

    detected_target_candidate = Node(
        package="edgepick_perception",
        executable="detected_target_candidate_node",
        name="edgepick_detected_target_candidate",
        output="screen",
        parameters=[
            {
                "detections_topic": detections_topic,
                "depth_topic": LaunchConfiguration("depth_topic"),
                "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                "target_topic": target_topic,
                "event_topic": event_topic,
                "target_class_id": LaunchConfiguration("target_class_id"),
                "target_label": LaunchConfiguration("target_label"),
                "min_detection_score": LaunchConfiguration("min_detection_score"),
                "max_detection_age_ms": LaunchConfiguration("max_detection_age_ms"),
                "min_depth_m": LaunchConfiguration("min_depth_m"),
                "max_depth_m": LaunchConfiguration("max_depth_m"),
                "publish_task_events": LaunchConfiguration("publish_task_events"),
                "publish_target_lost": LaunchConfiguration("publish_target_lost"),
                "publish_event_once": LaunchConfiguration("publish_event_once"),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("image_topic", default_value="/camera/color/image_raw"),
            DeclareLaunchArgument("depth_topic", default_value="/camera/depth/image_raw"),
            DeclareLaunchArgument("camera_info_topic", default_value="/camera/depth/camera_info"),
            DeclareLaunchArgument("model_path", default_value=DEFAULT_MODEL_PATH),
            DeclareLaunchArgument("frame_id", default_value=""),
            DeclareLaunchArgument("conf_threshold", default_value="0.25"),
            DeclareLaunchArgument("iou_threshold", default_value="0.70"),
            DeclareLaunchArgument("max_det", default_value="20"),
            DeclareLaunchArgument("publish_empty_frames", default_value="true"),
            DeclareLaunchArgument("target_class_id", default_value="-1"),
            DeclareLaunchArgument("target_label", default_value=""),
            DeclareLaunchArgument("min_detection_score", default_value="0.50"),
            DeclareLaunchArgument("max_detection_age_ms", default_value="500"),
            DeclareLaunchArgument("min_depth_m", default_value="0.05"),
            DeclareLaunchArgument("max_depth_m", default_value="1.20"),
            DeclareLaunchArgument("publish_task_events", default_value="true"),
            DeclareLaunchArgument("publish_target_lost", default_value="true"),
            DeclareLaunchArgument("publish_event_once", default_value="true"),
            yolo_detector,
            detected_target_candidate,
        ]
    )
