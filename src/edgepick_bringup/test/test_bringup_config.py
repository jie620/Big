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


def test_task_mock_launch_starts_task_node_with_topic_contract():
    launch_file = package_source_dir() / "launch" / "edgepick_task_mock.launch.py"
    launch_text = launch_file.read_text(encoding="utf-8")

    assert 'package="edgepick_task"' in launch_text
    assert 'executable="task_node"' in launch_text
    assert "/edgepick/task/event" in launch_text
    assert "/edgepick/task/state" in launch_text
    assert "/edgepick/task/failure" in launch_text
    assert "/diagnostics" in launch_text
