"""Run orange detection and execute a real grasp on the DOFBOT."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
            "require_startup_ready": LaunchConfiguration("require_startup_pose_ready"),
            "startup_ready_topic": LaunchConfiguration("startup_ready_topic"),
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
            "gate_events_by_task_state": LaunchConfiguration("gate_events_by_task_state"),
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

    kinematics_service = Node(
        package="dofbot_pro_info",
        executable="kinemarics_dofbot",
        name="edgepick_dofbot_kinematics",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_kinematics_service")),
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
        name="edgepick_orange_grasp_start_driver",
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

    orange_grasp_executor = Node(
        package="edgepick_task",
        executable="orange_grasp_executor_node",
        name="edgepick_orange_grasp_executor",
        output="screen",
        parameters=[
            {
                "event_topic": "/edgepick/task/event",
                "state_topic": "/edgepick/task/state",
                "target_point_topic": LaunchConfiguration("target_point_topic"),
                "startup_ready_topic": LaunchConfiguration("startup_ready_topic"),
                "require_startup_pose_ready": LaunchConfiguration("require_startup_pose_ready"),
                "use_real_i2c": LaunchConfiguration("use_real_i2c"),
                "i2c_device": LaunchConfiguration("i2c_device"),
                "i2c_address": ParameterValue(
                    LaunchConfiguration("i2c_address"), value_type=str
                ),
                "use_kinematics_service": LaunchConfiguration("use_kinematics_service"),
                "kinematics_service_name": LaunchConfiguration("kinematics_service_name"),
                "kinematics_wait_sec": LaunchConfiguration("kinematics_wait_sec"),
                "state_transition_wait_sec": LaunchConfiguration("state_transition_wait_sec"),
                "startup_wait_sec": LaunchConfiguration("startup_wait_sec"),
                "target_wait_sec": LaunchConfiguration("target_wait_sec"),
                "settle_time_ms": LaunchConfiguration("settle_time_ms"),
                "verification_settle_ms": LaunchConfiguration("verification_settle_ms"),
                "pregrasp_motion_time_ms": LaunchConfiguration("pregrasp_motion_time_ms"),
                "descend_motion_time_ms": LaunchConfiguration("descend_motion_time_ms"),
                "grip_motion_time_ms": LaunchConfiguration("grip_motion_time_ms"),
                "lift_motion_time_ms": LaunchConfiguration("lift_motion_time_ms"),
                "finish_motion_time_ms": LaunchConfiguration("finish_motion_time_ms"),
                "gripper_open_angle_deg": LaunchConfiguration("gripper_open_angle_deg"),
                "gripper_close_angle_deg": LaunchConfiguration("gripper_close_angle_deg"),
                "apply_target_yaw": LaunchConfiguration("apply_target_yaw"),
                "target_point_mode": LaunchConfiguration("target_point_mode"),
                "expected_target_frame": LaunchConfiguration("expected_target_frame"),
                "target_yaw_gain": LaunchConfiguration("target_yaw_gain"),
                "target_yaw_offset_deg": LaunchConfiguration("target_yaw_offset_deg"),
                "target_yaw_min_deg": LaunchConfiguration("target_yaw_min_deg"),
                "target_yaw_max_deg": LaunchConfiguration("target_yaw_max_deg"),
                "tracking_enabled": LaunchConfiguration("tracking_enabled"),
                "tracking_max_updates": LaunchConfiguration("tracking_max_updates"),
                "tracking_centered_updates": LaunchConfiguration("tracking_centered_updates"),
                "tracking_center_tolerance_deg": LaunchConfiguration(
                    "tracking_center_tolerance_deg"
                ),
                "tracking_yaw_gain": LaunchConfiguration("tracking_yaw_gain"),
                "tracking_motion_time_ms": LaunchConfiguration("tracking_motion_time_ms"),
                "tracking_target_max_age_ms": LaunchConfiguration("tracking_target_max_age_ms"),
                "target_world_offset_x_m": LaunchConfiguration("target_world_offset_x_m"),
                "target_world_offset_y_m": LaunchConfiguration("target_world_offset_y_m"),
                "target_world_offset_z_m": LaunchConfiguration("target_world_offset_z_m"),
                "ik_target_z_radius_origin_m": LaunchConfiguration(
                    "ik_target_z_radius_origin_m"
                ),
                "ik_target_z_radius_gain": LaunchConfiguration("ik_target_z_radius_gain"),
                "ik_joint4_max_deg": LaunchConfiguration("ik_joint4_max_deg"),
                "ik_wrist_angle_deg": LaunchConfiguration("ik_wrist_angle_deg"),
                "use_ik_joint5": LaunchConfiguration("use_ik_joint5"),
                "ik_lift_servo2_angle_deg": LaunchConfiguration("ik_lift_servo2_angle_deg"),
                "min_target_depth_m": LaunchConfiguration("min_target_depth_m"),
                "max_target_depth_m": LaunchConfiguration("max_target_depth_m"),
                "max_abs_target_lateral_m": LaunchConfiguration("max_abs_target_lateral_m"),
                "min_target_vertical_m": LaunchConfiguration("min_target_vertical_m"),
                "max_target_vertical_m": LaunchConfiguration("max_target_vertical_m"),
                "return_to_finish_pose": LaunchConfiguration("return_to_finish_pose"),
                "pregrasp_servo_angle1": LaunchConfiguration("pregrasp_servo_angle1"),
                "pregrasp_servo_angle2": LaunchConfiguration("pregrasp_servo_angle2"),
                "pregrasp_servo_angle3": LaunchConfiguration("pregrasp_servo_angle3"),
                "pregrasp_servo_angle4": LaunchConfiguration("pregrasp_servo_angle4"),
                "pregrasp_servo_angle5": LaunchConfiguration("pregrasp_servo_angle5"),
                "pregrasp_servo_angle6": LaunchConfiguration("pregrasp_servo_angle6"),
                "grasp_servo_angle1": LaunchConfiguration("grasp_servo_angle1"),
                "grasp_servo_angle2": LaunchConfiguration("grasp_servo_angle2"),
                "grasp_servo_angle3": LaunchConfiguration("grasp_servo_angle3"),
                "grasp_servo_angle4": LaunchConfiguration("grasp_servo_angle4"),
                "grasp_servo_angle5": LaunchConfiguration("grasp_servo_angle5"),
                "grasp_servo_angle6": LaunchConfiguration("grasp_servo_angle6"),
                "lift_servo_angle1": LaunchConfiguration("lift_servo_angle1"),
                "lift_servo_angle2": LaunchConfiguration("lift_servo_angle2"),
                "lift_servo_angle3": LaunchConfiguration("lift_servo_angle3"),
                "lift_servo_angle4": LaunchConfiguration("lift_servo_angle4"),
                "lift_servo_angle5": LaunchConfiguration("lift_servo_angle5"),
                "lift_servo_angle6": LaunchConfiguration("lift_servo_angle6"),
                "finish_servo_angle1": LaunchConfiguration("finish_servo_angle1"),
                "finish_servo_angle2": LaunchConfiguration("finish_servo_angle2"),
                "finish_servo_angle3": LaunchConfiguration("finish_servo_angle3"),
                "finish_servo_angle4": LaunchConfiguration("finish_servo_angle4"),
                "finish_servo_angle5": LaunchConfiguration("finish_servo_angle5"),
                "finish_servo_angle6": LaunchConfiguration("finish_servo_angle6"),
                "ik_reference_servo_angle1": LaunchConfiguration("ik_reference_servo_angle1"),
                "ik_reference_servo_angle2": LaunchConfiguration("ik_reference_servo_angle2"),
                "ik_reference_servo_angle3": LaunchConfiguration("ik_reference_servo_angle3"),
                "ik_reference_servo_angle4": LaunchConfiguration("ik_reference_servo_angle4"),
                "ik_reference_servo_angle5": LaunchConfiguration("ik_reference_servo_angle5"),
                "ik_reference_servo_angle6": LaunchConfiguration("ik_reference_servo_angle6"),
            }
        ],
    )

    startup_pose_sequence = Node(
        package="edgepick_task",
        executable="startup_pose_sequence_node",
        name="edgepick_startup_pose_sequence",
        output="screen",
        parameters=[
            {
                "use_real_i2c": LaunchConfiguration("use_real_i2c"),
                "i2c_device": LaunchConfiguration("i2c_device"),
                # Force the address to remain a string. YAML otherwise parses
                # 0x15 as integer 21 before the startup node receives it.
                "i2c_address": ParameterValue(
                    LaunchConfiguration("i2c_address"), value_type=str
                ),
                "ready_topic": LaunchConfiguration("startup_ready_topic"),
                "settle_time_ms": LaunchConfiguration("startup_pose_settle_time_ms"),
                "zero_motion_time_ms": LaunchConfiguration("zero_motion_time_ms"),
                "restore_motion_time_ms": LaunchConfiguration("restore_motion_time_ms"),
                "zero_servo_angle1": LaunchConfiguration("zero_servo_angle1"),
                "zero_servo_angle2": LaunchConfiguration("zero_servo_angle2"),
                "zero_servo_angle3": LaunchConfiguration("zero_servo_angle3"),
                "zero_servo_angle4": LaunchConfiguration("zero_servo_angle4"),
                "zero_servo_angle5": LaunchConfiguration("zero_servo_angle5"),
                "zero_servo_angle6": LaunchConfiguration("zero_servo_angle6"),
                "restore_servo_angle1": LaunchConfiguration("restore_servo_angle1"),
                "restore_servo_angle2": LaunchConfiguration("restore_servo_angle2"),
                "restore_servo_angle3": LaunchConfiguration("restore_servo_angle3"),
                "restore_servo_angle4": LaunchConfiguration("restore_servo_angle4"),
                "restore_servo_angle5": LaunchConfiguration("restore_servo_angle5"),
                "restore_servo_angle6": LaunchConfiguration("restore_servo_angle6"),
            },
        ],
    )

    def start_workload_after_startup(event, _context):
        if event.returncode == 0:
            return [
                TimerAction(
                    period=LaunchConfiguration("post_startup_wait_sec"),
                    actions=[
                        orange_detection,
                        perception_metrics,
                        orange_grasp_executor,
                    ],
                ),
            ]
        return [
            LogInfo(
                msg=(
                    "Startup pose failed; orange grasp execution will remain stopped. "
                    f"returncode={event.returncode}"
                )
            ),
            EmitEvent(event=Shutdown(reason="startup pose failed")),
        ]

    startup_pose_start = TimerAction(
        period=LaunchConfiguration("startup_pose_start_delay_sec"),
        actions=[startup_pose_sequence],
    )

    workload_after_startup = RegisterEventHandler(
        OnProcessExit(
            target_action=startup_pose_sequence,
            on_exit=start_workload_after_startup,
        )
    )

    shutdown_when_done = RegisterEventHandler(
        OnProcessExit(
            target_action=orange_grasp_executor,
            on_exit=[EmitEvent(event=Shutdown(reason="orange grasp execution finished"))],
        )
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
            DeclareLaunchArgument(
                "startup_ready_topic", default_value="/edgepick/startup_pose/ready"
            ),
            DeclareLaunchArgument("require_startup_pose_ready", default_value="false"),
            DeclareLaunchArgument("target_frame", default_value="base_link"),
            DeclareLaunchArgument("publish_camera_static_tf", default_value="false"),
            DeclareLaunchArgument("camera_frame_id", default_value="camera_color_optical_frame"),
            DeclareLaunchArgument("camera_tf_x", default_value="0.0"),
            DeclareLaunchArgument("camera_tf_y", default_value="0.0"),
            DeclareLaunchArgument("camera_tf_z", default_value="0.0"),
            DeclareLaunchArgument("camera_tf_roll", default_value="0.0"),
            DeclareLaunchArgument("camera_tf_pitch", default_value="0.0"),
            DeclareLaunchArgument("camera_tf_yaw", default_value="0.0"),
            DeclareLaunchArgument("min_detection_score", default_value="0.50"),
            DeclareLaunchArgument("max_detection_age_ms", default_value="500"),
            DeclareLaunchArgument("min_depth_m", default_value="0.05"),
            DeclareLaunchArgument("max_depth_m", default_value="1.20"),
            DeclareLaunchArgument("metrics_period_ms", default_value="2000"),
            DeclareLaunchArgument("task_scenario", default_value="start_only"),
            DeclareLaunchArgument("event_period_ms", default_value="300"),
            DeclareLaunchArgument("initial_delay_ms", default_value="500"),
            DeclareLaunchArgument("max_recovery_attempts", default_value="2"),
            DeclareLaunchArgument("publish_task_events", default_value="true"),
            DeclareLaunchArgument("publish_target_lost", default_value="true"),
            DeclareLaunchArgument("publish_event_once", default_value="true"),
            DeclareLaunchArgument("gate_events_by_task_state", default_value="true"),
            DeclareLaunchArgument("publish_frequency", default_value="15.0"),
            DeclareLaunchArgument("use_real_i2c", default_value="true"),
            DeclareLaunchArgument("i2c_device", default_value="/dev/i2c-7"),
            DeclareLaunchArgument("i2c_address", default_value="0x15"),
            DeclareLaunchArgument("start_kinematics_service", default_value="true"),
            DeclareLaunchArgument("use_kinematics_service", default_value="true"),
            DeclareLaunchArgument("kinematics_service_name", default_value="dofbot_kinemarics"),
            DeclareLaunchArgument("kinematics_wait_sec", default_value="5.0"),
            DeclareLaunchArgument("startup_pose_start_delay_sec", default_value="0.5"),
            DeclareLaunchArgument("post_startup_wait_sec", default_value="1.0"),
            DeclareLaunchArgument("startup_pose_settle_time_ms", default_value="250"),
            DeclareLaunchArgument("zero_motion_time_ms", default_value="3000"),
            DeclareLaunchArgument("restore_motion_time_ms", default_value="2000"),
            DeclareLaunchArgument("state_transition_wait_sec", default_value="10.0"),
            DeclareLaunchArgument("startup_wait_sec", default_value="30.0"),
            DeclareLaunchArgument("target_wait_sec", default_value="30.0"),
            DeclareLaunchArgument("target_point_topic", default_value="/edgepick/perception/target_point"),
            DeclareLaunchArgument("target_point_mode", default_value="camera_optical"),
            DeclareLaunchArgument("expected_target_frame", default_value="camera_color_optical_frame"),
            DeclareLaunchArgument("settle_time_ms", default_value="250"),
            DeclareLaunchArgument("verification_settle_ms", default_value="300"),
            DeclareLaunchArgument("pregrasp_motion_time_ms", default_value="1000"),
            DeclareLaunchArgument("descend_motion_time_ms", default_value="1000"),
            DeclareLaunchArgument("grip_motion_time_ms", default_value="600"),
            DeclareLaunchArgument("lift_motion_time_ms", default_value="1000"),
            DeclareLaunchArgument("finish_motion_time_ms", default_value="1000"),
            DeclareLaunchArgument("gripper_open_angle_deg", default_value="30.0"),
            DeclareLaunchArgument("gripper_close_angle_deg", default_value="142.0"),
            DeclareLaunchArgument("apply_target_yaw", default_value="true"),
            DeclareLaunchArgument("target_yaw_gain", default_value="1.0"),
            DeclareLaunchArgument("target_yaw_offset_deg", default_value="0.0"),
            DeclareLaunchArgument("target_yaw_min_deg", default_value="20.0"),
            DeclareLaunchArgument("target_yaw_max_deg", default_value="160.0"),
            DeclareLaunchArgument("tracking_enabled", default_value="true"),
            DeclareLaunchArgument("tracking_max_updates", default_value="6"),
            DeclareLaunchArgument("tracking_centered_updates", default_value="2"),
            DeclareLaunchArgument("tracking_center_tolerance_deg", default_value="4.0"),
            DeclareLaunchArgument("tracking_yaw_gain", default_value="0.65"),
            DeclareLaunchArgument("tracking_motion_time_ms", default_value="250"),
            DeclareLaunchArgument("tracking_target_max_age_ms", default_value="500"),
            DeclareLaunchArgument("target_world_offset_x_m", default_value="0.0"),
            DeclareLaunchArgument("target_world_offset_y_m", default_value="0.0"),
            DeclareLaunchArgument("target_world_offset_z_m", default_value="0.0"),
            DeclareLaunchArgument("ik_target_z_radius_origin_m", default_value="0.181"),
            DeclareLaunchArgument("ik_target_z_radius_gain", default_value="0.15"),
            DeclareLaunchArgument("ik_joint4_max_deg", default_value="90.0"),
            DeclareLaunchArgument("ik_wrist_angle_deg", default_value="90.0"),
            DeclareLaunchArgument("use_ik_joint5", default_value="false"),
            DeclareLaunchArgument("ik_lift_servo2_angle_deg", default_value="120.0"),
            DeclareLaunchArgument("min_target_depth_m", default_value="0.05"),
            DeclareLaunchArgument("max_target_depth_m", default_value="1.20"),
            DeclareLaunchArgument("max_abs_target_lateral_m", default_value="0.35"),
            DeclareLaunchArgument("min_target_vertical_m", default_value="-0.35"),
            DeclareLaunchArgument("max_target_vertical_m", default_value="0.35"),
            DeclareLaunchArgument("return_to_finish_pose", default_value="false"),
            DeclareLaunchArgument("pregrasp_servo_angle1", default_value="90.0"),
            DeclareLaunchArgument("pregrasp_servo_angle2", default_value="80.0"),
            DeclareLaunchArgument("pregrasp_servo_angle3", default_value="50.0"),
            DeclareLaunchArgument("pregrasp_servo_angle4", default_value="50.0"),
            DeclareLaunchArgument("pregrasp_servo_angle5", default_value="90.0"),
            DeclareLaunchArgument("pregrasp_servo_angle6", default_value="30.0"),
            DeclareLaunchArgument("grasp_servo_angle1", default_value="90.0"),
            DeclareLaunchArgument("grasp_servo_angle2", default_value="35.0"),
            DeclareLaunchArgument("grasp_servo_angle3", default_value="65.0"),
            DeclareLaunchArgument("grasp_servo_angle4", default_value="0.0"),
            DeclareLaunchArgument("grasp_servo_angle5", default_value="90.0"),
            DeclareLaunchArgument("grasp_servo_angle6", default_value="30.0"),
            DeclareLaunchArgument("lift_servo_angle1", default_value="90.0"),
            DeclareLaunchArgument("lift_servo_angle2", default_value="80.0"),
            DeclareLaunchArgument("lift_servo_angle3", default_value="50.0"),
            DeclareLaunchArgument("lift_servo_angle4", default_value="50.0"),
            DeclareLaunchArgument("lift_servo_angle5", default_value="90.0"),
            DeclareLaunchArgument("lift_servo_angle6", default_value="135.0"),
            DeclareLaunchArgument("finish_servo_angle1", default_value="90.0"),
            DeclareLaunchArgument("finish_servo_angle2", default_value="80.0"),
            DeclareLaunchArgument("finish_servo_angle3", default_value="50.0"),
            DeclareLaunchArgument("finish_servo_angle4", default_value="50.0"),
            DeclareLaunchArgument("finish_servo_angle5", default_value="90.0"),
            DeclareLaunchArgument("finish_servo_angle6", default_value="135.0"),
            DeclareLaunchArgument("ik_reference_servo_angle1", default_value="90.0"),
            DeclareLaunchArgument("ik_reference_servo_angle2", default_value="120.0"),
            DeclareLaunchArgument("ik_reference_servo_angle3", default_value="0.0"),
            DeclareLaunchArgument("ik_reference_servo_angle4", default_value="0.0"),
            DeclareLaunchArgument("ik_reference_servo_angle5", default_value="90.0"),
            DeclareLaunchArgument("ik_reference_servo_angle6", default_value="20.0"),
            # Yahboom Arm_Lib startup home pose, in servo degrees.
            DeclareLaunchArgument("zero_servo_angle1", default_value="90.0"),
            DeclareLaunchArgument("zero_servo_angle2", default_value="90.0"),
            DeclareLaunchArgument("zero_servo_angle3", default_value="90.0"),
            DeclareLaunchArgument("zero_servo_angle4", default_value="90.0"),
            DeclareLaunchArgument("zero_servo_angle5", default_value="90.0"),
            DeclareLaunchArgument("zero_servo_angle6", default_value="30.0"),
            # Fixed Yahboom Arm_Lib restore pose, in servo degrees.
            DeclareLaunchArgument("restore_servo_angle1", default_value="90.0"),
            DeclareLaunchArgument("restore_servo_angle2", default_value="165.0"),
            DeclareLaunchArgument("restore_servo_angle3", default_value="18.0"),
            DeclareLaunchArgument("restore_servo_angle4", default_value="0.0"),
            DeclareLaunchArgument("restore_servo_angle5", default_value="90.0"),
            DeclareLaunchArgument("restore_servo_angle6", default_value="30.0"),
            static_camera_tf,
            kinematics_service,
            task_node,
            mock_task_driver,
            startup_pose_start,
            workload_after_startup,
            shutdown_when_done,
        ]
    )
