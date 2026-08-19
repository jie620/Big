"""Run the full mock rehearsal chain before enabling real hardware."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    depth_topic = "/camera/depth/image_raw"
    camera_info_topic = "/camera/depth/camera_info"
    detections_topic = "/edgepick/perception/detections"
    target_point_topic = "/edgepick/perception/target_point"
    target_point_base_topic = "/edgepick/perception/target_point_base"
    pregrasp_pose_topic = "/edgepick/task/pregrasp_pose"
    grasp_pose_topic = "/edgepick/task/grasp_pose"
    event_topic = "/edgepick/task/event"
    state_topic = "/edgepick/task/state"
    failure_topic = "/edgepick/task/failure"
    diagnostics_topic = "/diagnostics"
    metrics_topic = "/edgepick/perception/metrics"

    mock_rgbd_source = Node(
        package="edgepick_perception",
        executable="mock_rgbd_source_node",
        name="edgepick_mock_rgbd_source",
        output="screen",
        parameters=[
            {
                "depth_topic": depth_topic,
                "camera_info_topic": camera_info_topic,
                "frame_id": LaunchConfiguration("camera_frame_id"),
                "width": LaunchConfiguration("image_width"),
                "height": LaunchConfiguration("image_height"),
                "fx": LaunchConfiguration("fx"),
                "fy": LaunchConfiguration("fy"),
                "cx": LaunchConfiguration("cx"),
                "cy": LaunchConfiguration("cy"),
                "depth_m": LaunchConfiguration("mock_depth_m"),
                "publish_hz": LaunchConfiguration("mock_rgbd_hz"),
            }
        ],
    )

    static_camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="edgepick_mock_camera_static_tf",
        output="screen",
        arguments=[
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "0",
            "--roll",
            "0",
            "--pitch",
            "0",
            "--yaw",
            "0",
            "--frame-id",
            LaunchConfiguration("target_frame"),
            "--child-frame-id",
            LaunchConfiguration("camera_frame_id"),
        ],
    )

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
                "depth_topic": depth_topic,
                "camera_info_topic": camera_info_topic,
                "target_topic": target_point_topic,
                "event_topic": event_topic,
                "state_topic": state_topic,
                "target_class_id": LaunchConfiguration("target_class_id"),
                "target_label": LaunchConfiguration("target_label"),
                "min_detection_score": LaunchConfiguration("min_detection_score"),
                "max_detection_age_ms": LaunchConfiguration("max_detection_age_ms"),
                "min_depth_m": LaunchConfiguration("min_depth_m"),
                "max_depth_m": LaunchConfiguration("max_depth_m"),
                "publish_task_events": True,
                "publish_target_lost": False,
                "publish_event_once": True,
                "gate_events_by_task_state": True,
                "target_event_state": "perceiving",
            }
        ],
    )

    target_frame_transform = Node(
        package="edgepick_perception",
        executable="target_frame_transform_node",
        name="edgepick_target_frame_transform",
        output="screen",
        parameters=[
            {
                "input_topic": target_point_topic,
                "output_topic": target_point_base_topic,
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
                "target_point_topic": target_point_base_topic,
                "pregrasp_pose_topic": pregrasp_pose_topic,
                "grasp_pose_topic": grasp_pose_topic,
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
                "detections_topic": detections_topic,
                "target_topic": target_point_topic,
                "event_topic": event_topic,
                "metrics_topic": metrics_topic,
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
                "event_topic": event_topic,
                "state_topic": state_topic,
                "failure_topic": failure_topic,
                "diagnostics_topic": diagnostics_topic,
            }
        ],
    )

    mock_task_driver = Node(
        package="edgepick_task",
        executable="mock_task_driver_node",
        name="edgepick_system_rehearsal_driver",
        output="screen",
        parameters=[
            {
                "scenario": "system_rehearsal_success",
                "event_period_ms": LaunchConfiguration("event_period_ms"),
                "initial_delay_ms": LaunchConfiguration("initial_delay_ms"),
                "event_topic": event_topic,
                "state_topic": state_topic,
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
                "use_mock_action_results": True,
                "planning_outcome": LaunchConfiguration("planning_outcome"),
                "execution_outcome": LaunchConfiguration("execution_outcome"),
                "action_result_delay_ms": LaunchConfiguration("action_result_delay_ms"),
                "event_topic": event_topic,
                "state_topic": state_topic,
                "move_group_action_name": "/move_action",
                "execute_trajectory_action_name": "/execute_trajectory",
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("camera_frame_id", default_value="camera_color_optical_frame"),
            DeclareLaunchArgument("target_frame", default_value="base_link"),
            DeclareLaunchArgument("image_width", default_value="640"),
            DeclareLaunchArgument("image_height", default_value="480"),
            DeclareLaunchArgument("fx", default_value="600.0"),
            DeclareLaunchArgument("fy", default_value="600.0"),
            DeclareLaunchArgument("cx", default_value="320.0"),
            DeclareLaunchArgument("cy", default_value="240.0"),
            DeclareLaunchArgument("mock_depth_m", default_value="0.60"),
            DeclareLaunchArgument("mock_rgbd_hz", default_value="10.0"),
            DeclareLaunchArgument("target_class_id", default_value="1"),
            DeclareLaunchArgument("target_label", default_value="target"),
            DeclareLaunchArgument("min_detection_score", default_value="0.50"),
            DeclareLaunchArgument("max_detection_age_ms", default_value="1000"),
            DeclareLaunchArgument("mock_detection_hz", default_value="10.0"),
            DeclareLaunchArgument("mock_score", default_value="0.90"),
            DeclareLaunchArgument("mock_center_u", default_value="320.0"),
            DeclareLaunchArgument("mock_center_v", default_value="240.0"),
            DeclareLaunchArgument("mock_size_u", default_value="80.0"),
            DeclareLaunchArgument("mock_size_v", default_value="80.0"),
            DeclareLaunchArgument("min_depth_m", default_value="0.05"),
            DeclareLaunchArgument("max_depth_m", default_value="1.20"),
            DeclareLaunchArgument("transform_timeout_ms", default_value="100"),
            DeclareLaunchArgument("pregrasp_offset_m", default_value="0.08"),
            DeclareLaunchArgument("grasp_z_offset_m", default_value="0.02"),
            DeclareLaunchArgument("metrics_period_ms", default_value="2000"),
            DeclareLaunchArgument("event_period_ms", default_value="300"),
            DeclareLaunchArgument("initial_delay_ms", default_value="500"),
            DeclareLaunchArgument("action_result_delay_ms", default_value="300"),
            DeclareLaunchArgument("max_recovery_attempts", default_value="2"),
            DeclareLaunchArgument("planning_outcome", default_value="success"),
            DeclareLaunchArgument("execution_outcome", default_value="success"),
            mock_rgbd_source,
            static_camera_tf,
            mock_detector,
            detected_target_candidate,
            target_frame_transform,
            grasp_target_builder,
            perception_metrics,
            task_node,
            mock_task_driver,
            moveit_action_adapter,
        ]
    )
