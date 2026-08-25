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
from moveit_configs_utils import MoveItConfigsBuilder


def _edgepick_moveit_config():
    """Reuse vendor MoveIt config while matching the real hardware launch inputs."""
    edgepick_share = Path(get_package_share_directory("edgepick_bringup"))
    robot_xacro = edgepick_share / "urdf" / "edgepick_dofbot.urdf.xacro"
    initial_positions = edgepick_share / "config" / "initial_positions.yaml"

    return (
        MoveItConfigsBuilder("DOFBOT_Pro-V24", package_name="dofbot_pro_moveit")
        .robot_description(
            file_path=str(robot_xacro),
            mappings={
                "initial_positions_file": str(initial_positions),
                "use_real_i2c": "true",
                "i2c_device": "/dev/i2c-7",
                "i2c_address": "0x15",
            },
        )
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )


def generate_launch_description():
    edgepick_share = Path(get_package_share_directory("edgepick_bringup"))
    orange_detection_launch = edgepick_share / "launch" / "edgepick_orange_detection.launch.py"
    real_moveit_launch = edgepick_share / "launch" / "edgepick_moveit_real.launch.py"
    moveit_config = _edgepick_moveit_config()

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
            "gate_events_by_task_state": "true",
            "target_event_state": "perceiving",
        }.items(),
    )

    real_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(real_moveit_launch)),
        launch_arguments={
            "publish_frequency": LaunchConfiguration("publish_frequency"),
            "use_real_i2c": LaunchConfiguration("use_real_i2c"),
            "i2c_device": LaunchConfiguration("i2c_device"),
            "i2c_address": LaunchConfiguration("i2c_address"),
            "motion_time_ms": LaunchConfiguration("motion_time_ms"),
            "use_rviz": LaunchConfiguration("use_rviz"),
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
            moveit_config.to_dict(),
            {
                "move_group_name": LaunchConfiguration("move_group_name"),
                "event_topic": "/edgepick/task/event",
                "state_topic": "/edgepick/task/state",
                "pregrasp_pose_topic": "/edgepick/task/pregrasp_pose",
                "grasp_pose_topic": "/edgepick/task/grasp_pose",
                "gripper_action_name": "/grip_group_controller/gripper_cmd",
                "startup_ready_topic": LaunchConfiguration("startup_ready_topic"),
                "require_startup_pose_ready": LaunchConfiguration("require_startup_pose_ready"),
                "planning_time_sec": LaunchConfiguration("planning_time_sec"),
                "planning_attempts": LaunchConfiguration("planning_attempts"),
                "state_monitor_wait_sec": LaunchConfiguration("state_monitor_wait_sec"),
                "state_transition_wait_sec": LaunchConfiguration("state_transition_wait_sec"),
                "startup_wait_sec": LaunchConfiguration("startup_wait_sec"),
                "settle_time_ms": LaunchConfiguration("settle_time_ms"),
                "verification_settle_ms": LaunchConfiguration("verification_settle_ms"),
                "goal_position_tolerance_m": LaunchConfiguration("goal_position_tolerance_m"),
                "goal_orientation_tolerance_rad": LaunchConfiguration(
                    "goal_orientation_tolerance_rad"
                ),
                "velocity_scaling_factor": LaunchConfiguration("velocity_scaling_factor"),
                "acceleration_scaling_factor": LaunchConfiguration(
                    "acceleration_scaling_factor"
                ),
                "gripper_open_position": LaunchConfiguration("gripper_open_position"),
                "gripper_close_position": LaunchConfiguration("gripper_close_position"),
                "gripper_max_effort": LaunchConfiguration("gripper_max_effort"),
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
                real_moveit,
                TimerAction(
                    period=LaunchConfiguration("post_startup_moveit_wait_sec"),
                    actions=[
                        orange_detection,
                        target_frame_transform,
                        grasp_target_builder,
                        perception_metrics,
                        orange_grasp_executor,
                    ],
                ),
            ]
        return [
            LogInfo(
                msg=(
                    "Startup pose failed; MoveIt and orange grasp execution will remain stopped. "
                    f"returncode={event.returncode}"
                )
            )
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
            DeclareLaunchArgument("transform_timeout_ms", default_value="100"),
            DeclareLaunchArgument("pregrasp_offset_m", default_value="0.08"),
            DeclareLaunchArgument("grasp_z_offset_m", default_value="0.02"),
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
            DeclareLaunchArgument("publish_frequency", default_value="15.0"),
            DeclareLaunchArgument("use_real_i2c", default_value="true"),
            DeclareLaunchArgument("i2c_device", default_value="/dev/i2c-7"),
            DeclareLaunchArgument("i2c_address", default_value="0x15"),
            DeclareLaunchArgument("motion_time_ms", default_value="30"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("startup_pose_start_delay_sec", default_value="0.5"),
            DeclareLaunchArgument("post_startup_moveit_wait_sec", default_value="8.0"),
            DeclareLaunchArgument("startup_pose_settle_time_ms", default_value="250"),
            DeclareLaunchArgument("zero_motion_time_ms", default_value="3000"),
            DeclareLaunchArgument("restore_motion_time_ms", default_value="2000"),
            DeclareLaunchArgument(
                "gripper_action_name", default_value="/grip_group_controller/gripper_cmd"
            ),
            DeclareLaunchArgument("move_group_name", default_value="arm_group"),
            DeclareLaunchArgument("planning_time_sec", default_value="5.0"),
            DeclareLaunchArgument("planning_attempts", default_value="10"),
            DeclareLaunchArgument("state_monitor_wait_sec", default_value="2.0"),
            DeclareLaunchArgument("state_transition_wait_sec", default_value="10.0"),
            DeclareLaunchArgument("startup_wait_sec", default_value="30.0"),
            DeclareLaunchArgument("settle_time_ms", default_value="500"),
            DeclareLaunchArgument("verification_settle_ms", default_value="300"),
            DeclareLaunchArgument("goal_position_tolerance_m", default_value="0.01"),
            DeclareLaunchArgument("goal_orientation_tolerance_rad", default_value="0.05"),
            DeclareLaunchArgument("velocity_scaling_factor", default_value="0.1"),
            DeclareLaunchArgument("acceleration_scaling_factor", default_value="0.1"),
            DeclareLaunchArgument("gripper_open_position", default_value="-0.0796"),
            DeclareLaunchArgument("gripper_close_position", default_value="-1.4939"),
            DeclareLaunchArgument("gripper_max_effort", default_value="0.0"),
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
            task_node,
            mock_task_driver,
            startup_pose_start,
            workload_after_startup,
            shutdown_when_done,
        ]
    )
