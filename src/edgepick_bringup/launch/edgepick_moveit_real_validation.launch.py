"""Launch real MoveIt and run the minimal Stage 17 joint validation."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
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
    real_moveit_launch = edgepick_share / "launch" / "edgepick_moveit_real.launch.py"
    moveit_config = _edgepick_moveit_config()

    real_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(real_moveit_launch)),
        launch_arguments={
            "publish_frequency": LaunchConfiguration("publish_frequency"),
            "use_real_i2c": LaunchConfiguration("use_real_i2c"),
            "i2c_device": LaunchConfiguration("i2c_device"),
            "i2c_address": LaunchConfiguration("i2c_address"),
            "use_rviz": LaunchConfiguration("use_rviz"),
        }.items(),
    )

    moveit_real_validation = Node(
        package="edgepick_task",
        executable="moveit_real_validation_node",
        name="edgepick_moveit_real_validation",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "move_group_name": LaunchConfiguration("move_group_name"),
                "test_joint_index": LaunchConfiguration("test_joint_index"),
                "test_joint_delta_rad": LaunchConfiguration("test_joint_delta_rad"),
                "planning_time_sec": LaunchConfiguration("planning_time_sec"),
                "planning_attempts": LaunchConfiguration("planning_attempts"),
                "validation_attempts": LaunchConfiguration("validation_attempts"),
                "state_monitor_wait_sec": LaunchConfiguration("state_monitor_wait_sec"),
                "settle_time_ms": LaunchConfiguration("settle_time_ms"),
                "home_tolerance_rad": LaunchConfiguration("home_tolerance_rad"),
                "velocity_scaling_factor": LaunchConfiguration("velocity_scaling_factor"),
                "acceleration_scaling_factor": LaunchConfiguration("acceleration_scaling_factor"),
            }
        ],
    )

    validation_start = TimerAction(
        period=LaunchConfiguration("validation_start_delay_sec"),
        actions=[moveit_real_validation],
    )

    shutdown_when_done = RegisterEventHandler(
        OnProcessExit(
            target_action=moveit_real_validation,
            on_exit=[EmitEvent(event=Shutdown(reason="stage 17 validation finished"))],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("publish_frequency", default_value="15.0"),
            DeclareLaunchArgument("use_real_i2c", default_value="true"),
            DeclareLaunchArgument("i2c_device", default_value="/dev/i2c-7"),
            DeclareLaunchArgument("i2c_address", default_value="0x15"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("validation_start_delay_sec", default_value="8.0"),
            DeclareLaunchArgument("move_group_name", default_value="arm_group"),
            DeclareLaunchArgument("test_joint_index", default_value="0"),
            DeclareLaunchArgument("test_joint_delta_rad", default_value="0.05"),
            DeclareLaunchArgument("planning_time_sec", default_value="5.0"),
            DeclareLaunchArgument("planning_attempts", default_value="10"),
            DeclareLaunchArgument("validation_attempts", default_value="3"),
            DeclareLaunchArgument("state_monitor_wait_sec", default_value="2.0"),
            DeclareLaunchArgument("settle_time_ms", default_value="500"),
            DeclareLaunchArgument("home_tolerance_rad", default_value="0.03"),
            DeclareLaunchArgument("velocity_scaling_factor", default_value="0.1"),
            DeclareLaunchArgument("acceleration_scaling_factor", default_value="0.1"),
            real_moveit,
            validation_start,
            shutdown_when_done,
        ]
    )
