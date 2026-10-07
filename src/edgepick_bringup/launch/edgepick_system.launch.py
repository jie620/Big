"""Canonical EdgePick runtime entry point.

The ``mode`` argument selects the hardware boundary while keeping the same
MoveIt and Safety Gate path:

* ``mock`` uses EdgePick's mock ros2_control backend;
* ``mujoco`` uses the MuJoCo action backend from ``edgepick_safe``;
* ``real`` is the only mode that enables the I2C transport.

The camera driver is intentionally external because its topic and launch
parameters depend on the installed Orbbec firmware. The real mode validates
the measured camera transform and RGB-D registration before it starts motion.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


MODES = ("mock", "mujoco", "real")
CONTROLLERS = ("joint_state_broadcaster", "arm_group_controller", "grip_group_controller")


def _value(context, name: str) -> str:
    return LaunchConfiguration(name).perform(context)


def _truth(context, name: str) -> bool:
    return _value(context, name).strip().lower() == "true"


def _moveit_config(context, bringup: Path, real: bool):
    mappings = {
        "initial_positions_file": str(bringup / "config" / "initial_positions.yaml"),
        "use_real_i2c": str(real).lower(),
        "i2c_device": _value(context, "i2c_device"),
        "i2c_address": _value(context, "i2c_address"),
        "motion_time_ms": _value(context, "motion_time_ms"),
    }
    config = (
        MoveItConfigsBuilder("DOFBOT_Pro-V24", package_name="dofbot_pro_moveit")
        .robot_description(
            file_path=str(bringup / "urdf" / "edgepick_dofbot.urdf.xacro"),
            mappings=mappings,
        )
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .to_moveit_configs()
    )

    # The vendor SRDF disables some collision pairs as "Never". Those pairs
    # are unsafe for a scene with a held cube and static obstacles, so only
    # those exclusions are removed; all other vendor settings remain intact.
    semantic = ET.fromstring(config.robot_description_semantic["robot_description_semantic"])
    for pair in list(semantic.findall("disable_collisions")):
        if pair.get("reason") == "Never":
            semantic.remove(pair)
    config.robot_description_semantic["robot_description_semantic"] = ET.tostring(
        semantic, encoding="unicode"
    )
    return config


def _camera_transform(context) -> list[str]:
    raw = _value(context, "camera_transform")
    values = [part.strip() for part in raw.split(",")]
    if len(values) != 6 or not raw.strip():
        raise ValueError("camera_transform must be measured x,y,z,roll,pitch,yaw")
    numbers = [float(part) for part in values]
    if not all(math.isfinite(number) for number in numbers):
        raise ValueError("camera_transform contains a non-finite value")
    return values


def build(context):
    mode = _value(context, "mode")
    if mode not in MODES:
        raise ValueError(f"mode must be one of: {', '.join(MODES)}")

    real = mode == "real"
    if mode in ("mujoco", "real"):
        checkpoint = _value(context, "checkpoint")
        yolo_model = _value(context, "yolo_model")
        if not checkpoint or not Path(checkpoint).is_dir():
            raise ValueError(f"{mode} mode requires an existing checkpoint directory")
        if not yolo_model or not Path(yolo_model).is_file():
            raise ValueError(f"{mode} mode requires an existing yolo_model file")
        if not _truth(context, "registered_depth_confirmed"):
            raise ValueError(f"{mode} mode requires registered_depth_confirmed:=true")

    if real:
        if not (
            _truth(context, "calibration_confirmed")
            and _truth(context, "registered_depth_confirmed")
        ):
            raise ValueError(
                "real mode requires measured camera calibration and verified RGB-D registration"
            )
        camera_tf = _camera_transform(context)

    bringup = Path(get_package_share_directory("edgepick_bringup"))
    safe = Path(get_package_share_directory("edgepick_safe"))
    config = _moveit_config(context, bringup, real)
    safe_params = str(safe / "config" / "safe.yaml")
    actions = []
    critical = []

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[config.robot_description],
    )
    actions.append(robot_state_publisher)

    if mode == "mujoco":
        mujoco = Node(
            package="edgepick_safe",
            executable="mujoco_backend",
            prefix=_value(context, "python"),
            output="screen",
            parameters=[
                {
                    "model_xml": _value(context, "model_xml"),
                    "render": _truth(context, "render"),
                }
            ],
        )
        actions.append(mujoco)
        critical.append(mujoco)
    else:
        control = Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="screen",
            parameters=[
                config.robot_description,
                str(bringup / "config" / "edgepick_ros2_controllers.yaml"),
            ],
        )
        actions.append(control)
        critical.append(control)

        # Controller manager claims interfaces serially. Spawning all three
        # controllers at once creates a race on slower Jetson boots.
        for index, controller in enumerate(CONTROLLERS):
            actions.append(
                TimerAction(
                    period=0.5 + index * 1.5,
                    actions=[
                        Node(
                            package="controller_manager",
                            executable="spawner",
                            arguments=[controller, "--controller-manager", "/controller_manager"],
                            output="screen",
                        )
                    ],
                )
            )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        parameters=[
            config.to_dict(),
            {
                "publish_robot_description_semantic": True,
                "publish_planning_scene": True,
                "publish_geometry_updates": True,
                "publish_state_updates": True,
            },
        ],
        output="screen",
    )
    gate = Node(
        package="edgepick_safe",
        executable="safety_executor",
        parameters=[
            config.to_dict(),
            safe_params,
            {
                "real_hardware": real,
                "calibration_confirmed": _truth(context, "calibration_confirmed"),
            },
        ],
        output="screen",
    )
    actions.extend([move_group, gate])
    critical.extend([move_group, gate])

    if mode == "mock" and _truth(context, "mock_inputs"):
        actions.append(
            Node(
                package="edgepick_safe",
                executable="mock_inputs",
                parameters=[
                    {
                        "auto_arm": _truth(context, "auto_arm"),
                        "scenario": _value(context, "scenario"),
                    }
                ],
                output="screen",
            )
        )

    if _value(context, "checkpoint"):
        policy = Node(
            package="edgepick_safe",
            executable="policy_node",
            prefix=_value(context, "python"),
            parameters=[
                safe_params,
                {
                    "checkpoint": _value(context, "checkpoint"),
                    "lerobot_source": _value(context, "lerobot_source"),
                    "task": _value(context, "task"),
                    "image_topic": _value(context, "image_topic"),
                    "depth_topic": _value(context, "depth_topic"),
                    "device": _value(context, "device"),
                    "use_amp": _truth(context, "use_amp"),
                },
            ],
            output="screen",
        )
        actions.append(policy)
        critical.append(policy)

    if real or (mode == "mujoco" and _value(context, "yolo_model")):
        perception = Node(
            package="edgepick_safe",
            executable="perception_node",
            prefix=_value(context, "python"),
            parameters=[
                safe_params,
                {
                    "yolo_model": _value(context, "yolo_model"),
                    "registered_depth_confirmed": _truth(context, "registered_depth_confirmed"),
                    "image_topic": _value(context, "image_topic"),
                    "depth_topic": _value(context, "depth_topic"),
                    "camera_info_topic": _value(context, "camera_info_topic"),
                    "detector_device": _value(context, "detector_device"),
                    "detector_half": _truth(context, "detector_half"),
                    "perception_rate_hz": float(_value(context, "perception_rate_hz")),
                },
            ],
            output="screen",
        )
        actions.append(perception)
        critical.append(perception)

    if real:
        actions.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "--x", camera_tf[0],
                    "--y", camera_tf[1],
                    "--z", camera_tf[2],
                    "--roll", camera_tf[3],
                    "--pitch", camera_tf[4],
                    "--yaw", camera_tf[5],
                    "--frame-id", "DaBai_DCW2_Link",
                    "--child-frame-id", _value(context, "camera_frame"),
                ],
                output="screen",
            )
        )

    for process in critical:
        actions.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=process,
                    on_exit=[EmitEvent(event=Shutdown(reason="critical EdgePick process exited"))],
                )
            )
        )
    return actions


def generate_launch_description():
    root = Path(get_package_share_directory("edgepick_bringup")).parents[3]
    defaults = {
        "mode": "mock",
        "checkpoint": "",
        "task": "抓取黄色方块，绕开静态障碍物并安全放置到红色区域",
        "yolo_model": "",
        "python": str(root / ".venv-vla/bin/python"),
        "lerobot_source": str(root / "vendor/lerobot-smolvla-jetson/src"),
        "model_xml": str(root / "simulation/dofbot_pro/dofbot_pro.xml"),
        "render": "false",
        "device": "cuda",
        "use_amp": "false",
        "detector_device": "auto",
        "detector_half": "true",
        "perception_rate_hz": "5.0",
        "auto_arm": "false",
        "mock_inputs": "true",
        "scenario": "normal",
        "calibration_confirmed": "false",
        "registered_depth_confirmed": "false",
        "camera_transform": "",
        "camera_frame": "camera_color_optical_frame",
        "image_topic": "/camera/color/image_raw",
        "depth_topic": "/camera/aligned_depth_to_color/image_raw",
        "camera_info_topic": "/camera/color/camera_info",
        "i2c_device": "/dev/i2c-7",
        "i2c_address": "0x15",
        "motion_time_ms": "80",
    }
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()]
        + [OpaqueFunction(function=build)]
    )
