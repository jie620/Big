"""Launch MoveIt against EdgePick's real ros2_control hardware interface."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
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
    vendor_moveit_share = Path(get_package_share_directory("dofbot_pro_moveit"))
    real_control_launch = edgepick_share / "launch" / "edgepick_real_control.launch.py"
    moveit_config = _edgepick_moveit_config()

    real_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(real_control_launch)),
        launch_arguments={
            "publish_frequency": LaunchConfiguration("publish_frequency"),
            "use_real_i2c": LaunchConfiguration("use_real_i2c"),
            "i2c_device": LaunchConfiguration("i2c_device"),
            "i2c_address": LaunchConfiguration("i2c_address"),
        }.items(),
    )

    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": True,
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "monitor_dynamics": False,
    }

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), move_group_configuration],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", str(vendor_moveit_share / "config" / "moveit.rviz")],
        parameters=[
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("publish_frequency", default_value="15.0"),
            DeclareLaunchArgument("use_real_i2c", default_value="true"),
            DeclareLaunchArgument("i2c_device", default_value="/dev/i2c-7"),
            DeclareLaunchArgument("i2c_address", default_value="0x15"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            real_control,
            move_group,
            rviz,
        ]
    )
