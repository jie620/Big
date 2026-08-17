"""Start mock detection and RGB-D projection through the Stage 9 detection contract."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    detections_topic = "/edgepick/perception/detections"
    target_topic = "/edgepick/perception/target_point"
    event_topic = "/edgepick/task/event"

    mock_detector = Node(
        package="edgepick_perception",
        executable="mock_detector_node",
        name="edgepick_mock_detector",
        output="screen",
        parameters=[
            {
                "detections_topic": detections_topic,
                "frame_id": LaunchConfiguration("camera_frame_id"),
                "class_id": LaunchConfiguration("target_class_id"),
                "label": LaunchConfiguration("target_label"),
                "score": LaunchConfiguration("mock_score"),
                "center_u": LaunchConfiguration("mock_center_u"),
                "center_v": LaunchConfiguration("mock_center_v"),
                "size_u": LaunchConfiguration("mock_size_u"),
                "size_v": LaunchConfiguration("mock_size_v"),
                "publish_hz": LaunchConfiguration("mock_detection_hz"),
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
            DeclareLaunchArgument("depth_topic", default_value="/camera/depth/image_raw"),
            DeclareLaunchArgument("camera_info_topic", default_value="/camera/depth/camera_info"),
            DeclareLaunchArgument("camera_frame_id", default_value="camera_color_optical_frame"),
            DeclareLaunchArgument("target_class_id", default_value="1"),
            DeclareLaunchArgument("target_label", default_value="target"),
            DeclareLaunchArgument("min_detection_score", default_value="0.50"),
            DeclareLaunchArgument("max_detection_age_ms", default_value="500"),
            DeclareLaunchArgument("mock_detection_hz", default_value="10.0"),
            DeclareLaunchArgument("mock_score", default_value="0.90"),
            DeclareLaunchArgument("mock_center_u", default_value="320.0"),
            DeclareLaunchArgument("mock_center_v", default_value="240.0"),
            DeclareLaunchArgument("mock_size_u", default_value="80.0"),
            DeclareLaunchArgument("mock_size_v", default_value="80.0"),
            DeclareLaunchArgument("min_depth_m", default_value="0.05"),
            DeclareLaunchArgument("max_depth_m", default_value="1.20"),
            DeclareLaunchArgument("publish_task_events", default_value="true"),
            DeclareLaunchArgument("publish_target_lost", default_value="true"),
            DeclareLaunchArgument("publish_event_once", default_value="true"),
            mock_detector,
            detected_target_candidate,
        ]
    )
