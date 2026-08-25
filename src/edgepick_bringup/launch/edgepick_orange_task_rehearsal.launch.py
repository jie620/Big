"""Run orange detection, target transform, grasp target building, and task rehearsal."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    edgepick_share = Path(get_package_share_directory("edgepick_bringup"))
    orange_detection_launch = edgepick_share / "launch" / "edgepick_orange_detection.launch.py"

    orange_detection = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(orange_detection_launch)),
        launch_arguments={
            "image_topic": LaunchConfiguration("image_topic"),
            "depth_topic": LaunchConfiguration("depth_topic"),
            "camera_info_topic": LaunchConfiguration("camera_info_topic"),
            "model_path": LaunchConfiguration("model_path"),
            "config_path": LaunchConfiguration("config_path"),
            "label_path": LaunchConfiguration("label_path"),
            "target_label": LaunchConfiguration("target_label"),
            "frame_id": LaunchConfiguration("frame_id"),
            "conf_threshold": LaunchConfiguration("conf_threshold"),
            "max_detections": LaunchConfiguration("max_detections"),
            "publish_empty_frames": LaunchConfiguration("publish_empty_frames"),
            "show_viewer": LaunchConfiguration("show_viewer"),
            "window_name": LaunchConfiguration("window_name"),
            "frame_scale": LaunchConfiguration("frame_scale"),
            "min_detection_score": LaunchConfiguration("min_detection_score"),
            "max_detection_age_ms": LaunchConfiguration("max_detection_age_ms"),
            "min_depth_m": LaunchConfiguration("min_depth_m"),
            "max_depth_m": LaunchConfiguration("max_depth_m"),
            "publish_task_events": LaunchConfiguration("publish_task_events"),
            "publish_target_lost": LaunchConfiguration("publish_target_lost"),
            "publish_event_once": LaunchConfiguration("publish_event_once"),
            "state_topic": LaunchConfiguration("state_topic"),
            "gate_events_by_task_state": "true",
            "target_event_state": "perceiving",
        }.items(),
    )

    static_camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="edgepick_orange_camera_static_tf",
        output="screen",
        condition=IfCondition(LaunchConfiguration("publish_camera_static_tf")),
        arguments=[
            "--x",
            LaunchConfiguration("camera_tf_x"),
            "--y",
            LaunchConfiguration("camera_tf_y"),
            "--z",
            LaunchConfiguration("camera_tf_z"),
            "--roll",
            LaunchConfiguration("camera_tf_roll"),
            "--pitch",
            LaunchConfiguration("camera_tf_pitch"),
            "--yaw",
            LaunchConfiguration("camera_tf_yaw"),
            "--frame-id",
            LaunchConfiguration("target_frame"),
            "--child-frame-id",
            LaunchConfiguration("camera_frame_id"),
        ],
    )

    target_frame_transform = Node(
        package="edgepick_perception",
        executable="target_frame_transform_node",
        name="edgepick_target_frame_transform",
        output="screen",
        parameters=[
            {
                "input_topic": "/edgepick/perception/target_point",
                "output_topic": "/edgepick/perception/target_point_base",
                "target_frame": LaunchConfiguration("target_frame"),
                "transform_timeout_ms": LaunchConfiguration("transform_timeout_ms"),
            }
        ],
    )

    grasp_target_builder = Node(
        package="edgepick_task",
        executable="grasp_target_builder_node",
        name="edgepick_grasp_target_builder",
        output="screen",
        parameters=[
            {
                "target_point_topic": "/edgepick/perception/target_point_base",
                "pregrasp_pose_topic": "/edgepick/task/pregrasp_pose",
                "grasp_pose_topic": "/edgepick/task/grasp_pose",
                "expected_frame": LaunchConfiguration("target_frame"),
                "pregrasp_offset_m": LaunchConfiguration("pregrasp_offset_m"),
                "grasp_z_offset_m": LaunchConfiguration("grasp_z_offset_m"),
                "end_effector_orientation_xyzw": [0.0, 1.0, 0.0, 0.0],
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
                "detections_topic": "/edgepick/perception/detections",
                "target_topic": "/edgepick/perception/target_point",
                "event_topic": "/edgepick/task/event",
                "metrics_topic": "/edgepick/perception/metrics",
                "metrics_period_ms": LaunchConfiguration("metrics_period_ms"),
            }
        ],
    )

    task_node = Node(
        package="edgepick_task",
        executable="task_node",
        name="edgepick_task_node",
        output="screen",
        parameters=[
            {
                "max_recovery_attempts": LaunchConfiguration("max_recovery_attempts"),
                "event_topic": "/edgepick/task/event",
                "state_topic": "/edgepick/task/state",
                "failure_topic": "/edgepick/task/failure",
                "diagnostics_topic": "/diagnostics",
            }
        ],
    )

    mock_task_driver = Node(
        package="edgepick_task",
        executable="mock_task_driver_node",
        name="edgepick_orange_task_driver",
        output="screen",
        parameters=[
            {
                "scenario": LaunchConfiguration("task_scenario"),
                "event_period_ms": LaunchConfiguration("event_period_ms"),
                "initial_delay_ms": LaunchConfiguration("initial_delay_ms"),
                "event_topic": "/edgepick/task/event",
                "state_topic": "/edgepick/task/state",
            }
        ],
    )

    moveit_action_adapter = Node(
        package="edgepick_task",
        executable="moveit_action_adapter_node",
        name="edgepick_moveit_action_adapter",
        output="screen",
        parameters=[
            {
                "use_mock_action_results": LaunchConfiguration("use_mock_action_results"),
                "planning_outcome": LaunchConfiguration("planning_outcome"),
                "execution_outcome": LaunchConfiguration("execution_outcome"),
                "action_result_delay_ms": LaunchConfiguration("action_result_delay_ms"),
                "event_topic": "/edgepick/task/event",
                "state_topic": "/edgepick/task/state",
                "move_group_action_name": "/move_action",
                "execute_trajectory_action_name": "/execute_trajectory",
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
            DeclareLaunchArgument("state_topic", default_value="/edgepick/task/state"),
            DeclareLaunchArgument("target_frame", default_value="base_link"),
            DeclareLaunchArgument("publish_camera_static_tf", default_value="false"),
            DeclareLaunchArgument("camera_frame_id", default_value="camera_color_optical_frame"),
            DeclareLaunchArgument("camera_tf_x", default_value="0.0"),
            DeclareLaunchArgument("camera_tf_y", default_value="0.0"),
            DeclareLaunchArgument("camera_tf_z", default_value="0.0"),
            DeclareLaunchArgument("camera_tf_roll", default_value="0.0"),
            DeclareLaunchArgument("camera_tf_pitch", default_value="0.0"),
            DeclareLaunchArgument("camera_tf_yaw", default_value="0.0"),
            DeclareLaunchArgument("transform_timeout_ms", default_value="100"),
            DeclareLaunchArgument("pregrasp_offset_m", default_value="0.08"),
            DeclareLaunchArgument("grasp_z_offset_m", default_value="0.02"),
            DeclareLaunchArgument("metrics_period_ms", default_value="2000"),
            DeclareLaunchArgument("task_scenario", default_value="system_rehearsal_success"),
            DeclareLaunchArgument("event_period_ms", default_value="300"),
            DeclareLaunchArgument("initial_delay_ms", default_value="500"),
            DeclareLaunchArgument("use_mock_action_results", default_value="true"),
            DeclareLaunchArgument("planning_outcome", default_value="success"),
            DeclareLaunchArgument("execution_outcome", default_value="success"),
            DeclareLaunchArgument("action_result_delay_ms", default_value="300"),
            DeclareLaunchArgument("max_recovery_attempts", default_value="2"),
            DeclareLaunchArgument("min_detection_score", default_value="0.50"),
            DeclareLaunchArgument("max_detection_age_ms", default_value="500"),
            DeclareLaunchArgument("min_depth_m", default_value="0.05"),
            DeclareLaunchArgument("max_depth_m", default_value="1.20"),
            DeclareLaunchArgument("publish_task_events", default_value="true"),
            DeclareLaunchArgument("publish_target_lost", default_value="true"),
            DeclareLaunchArgument("publish_event_once", default_value="true"),
            static_camera_tf,
            orange_detection,
            target_frame_transform,
            grasp_target_builder,
            perception_metrics,
            task_node,
            mock_task_driver,
            moveit_action_adapter,
        ]
    )
