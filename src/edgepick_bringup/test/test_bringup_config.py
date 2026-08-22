import os
import subprocess
from pathlib import Path

import yaml


def package_source_dir() -> Path:
    return Path(os.environ["EDGE_PICK_BRINGUP_SOURCE_DIR"])


def test_xacro_expands_to_edgepick_mock_hardware():
    root = package_source_dir()
    xacro = root / "urdf" / "edgepick_dofbot.urdf.xacro"
    initial_positions = root / "config" / "initial_positions.yaml"

    result = subprocess.run(
        ["xacro", str(xacro), f"initial_positions_file:={initial_positions}"],
        check=True,
        text=True,
        capture_output=True,
    )

    robot_description = result.stdout
    assert "edgepick_hardware/MockSystemInterface" in robot_description
    assert 'name="use_real_i2c"' in robot_description
    assert 'name="i2c_device"' in robot_description
    assert 'name="i2c_address"' in robot_description
    assert "mock_components/GenericSystem" not in robot_description
    for joint_name in [
        "Arm1_Joint",
        "Arm2_Joint",
        "Arm3_Joint",
        "Arm4_Joint",
        "Arm5_Joint",
        "grip_joint",
    ]:
        assert f'name="{joint_name}"' in robot_description


def test_controller_yaml_matches_moveit_controller_names():
    config_path = package_source_dir() / "config" / "edgepick_ros2_controllers.yaml"
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))

    manager = config["controller_manager"]["ros__parameters"]
    assert manager["update_rate"] == 100
    assert manager["joint_state_broadcaster"]["type"] == "joint_state_broadcaster/JointStateBroadcaster"
    assert manager["arm_group_controller"]["type"] == "joint_trajectory_controller/JointTrajectoryController"
    assert manager["grip_group_controller"]["type"] == "position_controllers/GripperActionController"
    assert config["arm_group_controller"]["ros__parameters"]["joints"] == [
        "Arm1_Joint",
        "Arm2_Joint",
        "Arm3_Joint",
        "Arm4_Joint",
        "Arm5_Joint",
    ]
    assert config["grip_group_controller"]["ros__parameters"]["joint"] == "grip_joint"


def test_moveit_launch_replaces_only_robot_description_path():
    launch_file = package_source_dir() / "launch" / "edgepick_moveit_mock.launch.py"
    launch_text = launch_file.read_text(encoding="utf-8")

    assert "MoveItConfigsBuilder" in launch_text
    assert "edgepick_dofbot.urdf.xacro" in launch_text
    assert "dofbot_pro_moveit" in launch_text
    assert "edgepick_ros2_controllers.yaml" in launch_text


def test_moveit_real_launch_includes_real_control_and_move_group():
    launch_file = package_source_dir() / "launch" / "edgepick_moveit_real.launch.py"
    launch_text = launch_file.read_text(encoding="utf-8")

    assert "IncludeLaunchDescription" in launch_text
    assert "edgepick_real_control.launch.py" in launch_text
    assert 'DeclareLaunchArgument("use_real_i2c", default_value="true")' in launch_text
    assert 'DeclareLaunchArgument("use_rviz", default_value="false")' in launch_text
    assert "MoveItConfigsBuilder" in launch_text
    assert "moveit_ros_move_group" in launch_text
    assert "moveit.rviz" in launch_text
    assert '"use_real_i2c":' in launch_text
    assert '"i2c_device":' in launch_text
    assert '"i2c_address":' in launch_text


def test_moveit_real_validation_launch_includes_minimal_execution_node():
    launch_file = package_source_dir() / "launch" / "edgepick_moveit_real_validation.launch.py"
    launch_text = launch_file.read_text(encoding="utf-8")

    assert 'IncludeLaunchDescription' in launch_text
    assert 'edgepick_moveit_real.launch.py' in launch_text
    assert 'executable="moveit_real_validation_node"' in launch_text
    assert 'move_group_name' in launch_text
    assert 'test_joint_delta_rad' in launch_text
    assert 'home_tolerance_rad' in launch_text
    assert 'validation_start_delay_sec' in launch_text
    assert "MoveItConfigsBuilder" in launch_text
    assert 'OnProcessExit' in launch_text
    assert 'Shutdown' in launch_text


def test_real_control_launch_enables_explicit_i2c_parameters():
    launch_file = package_source_dir() / "launch" / "edgepick_real_control.launch.py"
    launch_text = launch_file.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument("use_real_i2c", default_value="true")' in launch_text
    assert 'DeclareLaunchArgument("i2c_device", default_value="/dev/i2c-7")' in launch_text
    assert 'DeclareLaunchArgument("i2c_address", default_value="0x15")' in launch_text
    assert "use_real_i2c:=" in launch_text
    assert "i2c_device:=" in launch_text
    assert "i2c_address:=" in launch_text


def test_task_mock_launch_starts_task_node_with_topic_contract():
    launch_file = package_source_dir() / "launch" / "edgepick_task_mock.launch.py"
    launch_text = launch_file.read_text(encoding="utf-8")

    assert 'package="edgepick_task"' in launch_text
    assert 'executable="task_node"' in launch_text
    assert "/edgepick/task/event" in launch_text
    assert "/edgepick/task/state" in launch_text
    assert "/edgepick/task/failure" in launch_text
    assert "/diagnostics" in launch_text


def test_task_closed_loop_launch_starts_task_and_mock_driver():
    launch_file = package_source_dir() / "launch" / "edgepick_task_closed_loop.launch.py"
    launch_text = launch_file.read_text(encoding="utf-8")

    assert 'executable="task_node"' in launch_text
    assert 'executable="mock_task_driver_node"' in launch_text
    assert 'DeclareLaunchArgument("scenario", default_value="success")' in launch_text
    assert "/edgepick/task/event" in launch_text
    assert "/edgepick/task/state" in launch_text
    assert "/edgepick/task/failure" in launch_text
    assert "/diagnostics" in launch_text


def test_moveit_action_mock_launch_splits_task_driver_from_action_adapter():
    launch_file = package_source_dir() / "launch" / "edgepick_moveit_action_mock.launch.py"
    launch_text = launch_file.read_text(encoding="utf-8")

    assert 'executable="task_node"' in launch_text
    assert 'executable="mock_task_driver_node"' in launch_text
    assert 'executable="moveit_action_adapter_node"' in launch_text
    assert '"scenario": "moveit_success"' in launch_text
    assert 'DeclareLaunchArgument("planning_outcome", default_value="success")' in launch_text
    assert 'DeclareLaunchArgument("execution_outcome", default_value="success")' in launch_text
    assert "/move_action" in launch_text
    assert "/execute_trajectory" in launch_text


def test_rgbd_perception_launch_starts_target_candidate_node():
    launch_file = package_source_dir() / "launch" / "edgepick_rgbd_perception.launch.py"
    launch_text = launch_file.read_text(encoding="utf-8")

    assert 'package="edgepick_perception"' in launch_text
    assert 'executable="rgbd_target_candidate_node"' in launch_text
    assert "/camera/depth/image_raw" in launch_text
    assert "/camera/depth/camera_info" in launch_text
    assert "/edgepick/perception/target_point" in launch_text
    assert "/edgepick/task/event" in launch_text
    assert 'DeclareLaunchArgument("target_pixel_u", default_value="-1")' in launch_text


def test_detection_perception_mock_launch_connects_detector_to_rgbd_projection():
    launch_file = package_source_dir() / "launch" / "edgepick_detection_perception_mock.launch.py"
    launch_text = launch_file.read_text(encoding="utf-8")

    assert 'executable="mock_detector_node"' in launch_text
    assert 'executable="detected_target_candidate_node"' in launch_text
    assert "/edgepick/perception/detections" in launch_text
    assert "/edgepick/perception/target_point" in launch_text
    assert "/edgepick/task/event" in launch_text
    assert 'DeclareLaunchArgument("target_label", default_value="target")' in launch_text
    assert 'DeclareLaunchArgument("min_detection_score", default_value="0.50")' in launch_text


def test_perception_metrics_launch_observes_real_or_bag_detection_chain():
    launch_file = package_source_dir() / "launch" / "edgepick_perception_metrics.launch.py"
    launch_text = launch_file.read_text(encoding="utf-8")

    assert 'executable="detected_target_candidate_node"' in launch_text
    assert 'executable="perception_metrics_node"' in launch_text
    assert "ros2" in launch_text
    assert "bag" in launch_text
    assert "play" in launch_text
    assert "/edgepick/perception/detections" in launch_text
    assert "/edgepick/perception/target_point" in launch_text
    assert "/edgepick/perception/metrics" in launch_text
    assert 'DeclareLaunchArgument("play_bag", default_value="false")' in launch_text
    assert 'DeclareLaunchArgument("run_candidate_node", default_value="true")' in launch_text
    assert 'DeclareLaunchArgument("metrics_period_ms", default_value="5000")' in launch_text


def test_target_frame_transform_launch_publishes_base_frame_target_point():
    launch_file = package_source_dir() / "launch" / "edgepick_target_frame_transform.launch.py"
    launch_text = launch_file.read_text(encoding="utf-8")

    assert 'executable="target_frame_transform_node"' in launch_text
    assert "/edgepick/perception/target_point" in launch_text
    assert "/edgepick/perception/target_point_base" in launch_text
    assert 'DeclareLaunchArgument("target_frame", default_value="base_link")' in launch_text
    assert 'DeclareLaunchArgument("transform_timeout_ms", default_value="100")' in launch_text


def test_mock_grasp_target_launch_builds_pregrasp_and_grasp_poses():
    launch_file = package_source_dir() / "launch" / "edgepick_mock_grasp_target.launch.py"
    launch_text = launch_file.read_text(encoding="utf-8")

    assert 'executable="grasp_target_builder_node"' in launch_text
    assert "/edgepick/perception/target_point_base" in launch_text
    assert "/edgepick/task/pregrasp_pose" in launch_text
    assert "/edgepick/task/grasp_pose" in launch_text
    assert 'DeclareLaunchArgument("target_frame", default_value="base_link")' in launch_text
    assert 'DeclareLaunchArgument("pregrasp_offset_m", default_value="0.08")' in launch_text
    assert 'DeclareLaunchArgument("grasp_z_offset_m", default_value="0.02")' in launch_text


def test_prehardware_mock_rehearsal_launch_wires_safe_full_chain():
    launch_file = package_source_dir() / "launch" / "edgepick_prehardware_mock_rehearsal.launch.py"
    launch_text = launch_file.read_text(encoding="utf-8")

    assert 'executable="mock_rgbd_source_node"' in launch_text
    assert 'executable="mock_detector_node"' in launch_text
    assert 'executable="detected_target_candidate_node"' in launch_text
    assert 'executable="target_frame_transform_node"' in launch_text
    assert 'executable="grasp_target_builder_node"' in launch_text
    assert 'executable="perception_metrics_node"' in launch_text
    assert 'executable="task_node"' in launch_text
    assert 'executable="mock_task_driver_node"' in launch_text
    assert 'executable="moveit_action_adapter_node"' in launch_text
    assert 'executable="static_transform_publisher"' in launch_text
    assert '"scenario": "system_rehearsal_success"' in launch_text
    assert '"publish_task_events": True' in launch_text
    assert '"publish_event_once": True' in launch_text
    assert '"gate_events_by_task_state": True' in launch_text
    assert '"target_event_state": "perceiving"' in launch_text
    assert "/edgepick/perception/target_point_base" in launch_text
    assert "/edgepick/task/pregrasp_pose" in launch_text
    assert "/edgepick/task/grasp_pose" in launch_text
    assert "/edgepick/perception/metrics" in launch_text


def test_yolo_detection_launch_connects_real_detector_to_detection_contract():
    launch_file = package_source_dir() / "launch" / "edgepick_yolo_detection.launch.py"
    launch_text = launch_file.read_text(encoding="utf-8")

    assert 'executable="edgepick_yolo_detector_node.py"' in launch_text
    assert 'executable="detected_target_candidate_node"' in launch_text
    assert "/camera/color/image_raw" in launch_text
    assert "/camera/depth/image_raw" in launch_text
    assert "/camera/depth/camera_info" in launch_text
    assert "/edgepick/perception/detections" in launch_text
    assert "/edgepick/perception/target_point" in launch_text
    assert "DEFAULT_MODEL_PATH" in launch_text
    assert "dofbot_pro_yolov11" in launch_text
    assert "best.engine" in launch_text
    assert 'DeclareLaunchArgument("conf_threshold", default_value="0.25")' in launch_text
    assert 'DeclareLaunchArgument("publish_empty_frames", default_value="true")' in launch_text


def test_orange_detection_launch_uses_coco_detector_and_orange_target():
    launch_file = package_source_dir() / "launch" / "edgepick_orange_detection.launch.py"
    launch_text = launch_file.read_text(encoding="utf-8")

    assert 'executable="edgepick_coco_detector_node.py"' in launch_text
    assert 'executable="detected_target_candidate_node"' in launch_text
    assert 'executable="edgepick_detection_viewer_node.py"' in launch_text
    assert "/camera/color/image_raw" in launch_text
    assert "/edgepick/perception/detections" in launch_text
    assert "/edgepick/perception/target_point" in launch_text
    assert 'default_value="orange"' in launch_text
    assert "frozen_inference_graph.pb" in launch_text
    assert "ssd_mobilenet_v2_coco.txt" in launch_text
    assert "object_detection_coco.txt" in launch_text
    assert 'DeclareLaunchArgument("show_viewer", default_value="false")' in launch_text
    assert 'DeclareLaunchArgument("window_name", default_value="edgepick_detection")' in launch_text
