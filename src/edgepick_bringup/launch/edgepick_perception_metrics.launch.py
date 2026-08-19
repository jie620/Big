"""Run Stage 10 perception measurement for a real detector or rosbag replay."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    detections_topic = "/edgepick/perception/detections"
    target_topic = "/edgepick/perception/target_point"
    event_topic = "/edgepick/task/event"
    metrics_topic = "/edgepick/perception/metrics"

    detected_target_candidate = Node(
        package="edgepick_perception",
        executable="detected_target_candidate_node",
        name="edgepick_detected_target_candidate",
        output="screen",
        condition=IfCondition(LaunchConfiguration("run_candidate_node")),
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
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

    perception_metrics = Node(
        package="edgepick_perception",
        executable="perception_metrics_node",
        name="edgepick_perception_metrics",
        output="screen",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "detections_topic": detections_topic,
                "target_topic": target_topic,
                "event_topic": event_topic,
                "metrics_topic": metrics_topic,
                "publish_period_ms": LaunchConfiguration("metrics_period_ms"),
            }
        ],
    )

    rosbag_play = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "play",
            LaunchConfiguration("bag_path"),
            "--clock",
            "--rate",
            LaunchConfiguration("bag_rate"),
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("play_bag")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("play_bag", default_value="false"),
            DeclareLaunchArgument("bag_path", default_value=""),
            DeclareLaunchArgument("bag_rate", default_value="1.0"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("run_candidate_node", default_value="true"),
            DeclareLaunchArgument("depth_topic", default_value="/camera/depth/image_raw"),
            DeclareLaunchArgument("camera_info_topic", default_value="/camera/depth/camera_info"),
            DeclareLaunchArgument("target_class_id", default_value="1"),
            DeclareLaunchArgument("target_label", default_value="target"),
            DeclareLaunchArgument("min_detection_score", default_value="0.50"),
            DeclareLaunchArgument("max_detection_age_ms", default_value="500"),
            DeclareLaunchArgument("min_depth_m", default_value="0.05"),
            DeclareLaunchArgument("max_depth_m", default_value="1.20"),
            DeclareLaunchArgument("publish_task_events", default_value="true"),
            DeclareLaunchArgument("publish_target_lost", default_value="true"),
            DeclareLaunchArgument("publish_event_once", default_value="true"),
            DeclareLaunchArgument("metrics_period_ms", default_value="5000"),
            rosbag_play,
            detected_target_candidate,
            perception_metrics,
        ]
    )
