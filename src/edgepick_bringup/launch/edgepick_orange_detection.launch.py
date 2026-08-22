"""Start COCO orange detection and convert detections into the EdgePick perception contract."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    detections_topic = "/edgepick/perception/detections"
    target_topic = "/edgepick/perception/target_point"
    event_topic = "/edgepick/task/event"

    coco_detector = Node(
        package="edgepick_perception",
        executable="edgepick_coco_detector_node.py",
        name="edgepick_coco_detector",
        output="screen",
        parameters=[
            {
                "image_topic": LaunchConfiguration("image_topic"),
                "detections_topic": detections_topic,
                "model_path": LaunchConfiguration("model_path"),
                "config_path": LaunchConfiguration("config_path"),
                "label_path": LaunchConfiguration("label_path"),
                "target_label": LaunchConfiguration("target_label"),
                "frame_id": LaunchConfiguration("frame_id"),
                "conf_threshold": LaunchConfiguration("conf_threshold"),
                "max_detections": LaunchConfiguration("max_detections"),
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

    detection_viewer = Node(
        package="edgepick_perception",
        executable="edgepick_detection_viewer_node.py",
        name="edgepick_detection_viewer",
        output="screen",
        condition=IfCondition(LaunchConfiguration("show_viewer")),
        parameters=[
            {
                "image_topic": LaunchConfiguration("image_topic"),
                "detections_topic": detections_topic,
                "window_name": LaunchConfiguration("window_name"),
                "frame_scale": LaunchConfiguration("frame_scale"),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("image_topic", default_value="/camera/color/image_raw"),
            DeclareLaunchArgument("depth_topic", default_value="/camera/depth/image_raw"),
            DeclareLaunchArgument("camera_info_topic", default_value="/camera/depth/camera_info"),
            DeclareLaunchArgument(
                "model_path",
                default_value="/home/jetson/dofbot_pro_ws/src/dofbot_pro_vision/config/frozen_inference_graph.pb",
            ),
            DeclareLaunchArgument(
                "config_path",
                default_value="/home/jetson/dofbot_pro_ws/src/dofbot_pro_vision/config/ssd_mobilenet_v2_coco.txt",
            ),
            DeclareLaunchArgument(
                "label_path",
                default_value="/home/jetson/dofbot_pro_ws/src/dofbot_pro_vision/config/object_detection_coco.txt",
            ),
            DeclareLaunchArgument("target_label", default_value="orange"),
            DeclareLaunchArgument("frame_id", default_value=""),
            DeclareLaunchArgument("conf_threshold", default_value="0.40"),
            DeclareLaunchArgument("max_detections", default_value="20"),
            DeclareLaunchArgument("publish_empty_frames", default_value="true"),
            DeclareLaunchArgument("show_viewer", default_value="false"),
            DeclareLaunchArgument("window_name", default_value="edgepick_detection"),
            DeclareLaunchArgument("frame_scale", default_value="1.0"),
            DeclareLaunchArgument("min_detection_score", default_value="0.50"),
            DeclareLaunchArgument("max_detection_age_ms", default_value="500"),
            DeclareLaunchArgument("min_depth_m", default_value="0.05"),
            DeclareLaunchArgument("max_depth_m", default_value="1.20"),
            DeclareLaunchArgument("publish_task_events", default_value="true"),
            DeclareLaunchArgument("publish_target_lost", default_value="true"),
            DeclareLaunchArgument("publish_event_once", default_value="true"),
            coco_detector,
            detected_target_candidate,
            detection_viewer,
        ]
    )
