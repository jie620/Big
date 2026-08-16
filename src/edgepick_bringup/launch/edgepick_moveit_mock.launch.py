"""Launch MoveIt/RViz against EdgePick's mock ros2_control hardware interface."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def _edgepick_moveit_config():
    """Reuse vendor MoveIt config while replacing only robot_description xacro."""
    edgepick_share = Path(get_package_share_directory("edgepick_bringup"))
    robot_xacro = edgepick_share / "urdf" / "edgepick_dofbot.urdf.xacro"
    initial_positions = edgepick_share / "config" / "initial_positions.yaml"

    return (
        MoveItConfigsBuilder("DOFBOT_Pro-V24", package_name="dofbot_pro_moveit")
        .robot_description(
            file_path=str(robot_xacro),
            mappings={"initial_positions_file": str(initial_positions)},
        )
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )


def generate_launch_description():
    edgepick_share = Path(get_package_share_directory("edgepick_bringup"))
    vendor_moveit_share = Path(get_package_share_directory("dofbot_pro_moveit"))
    controllers_file = edgepick_share / "config" / "edgepick_ros2_controllers.yaml"
    moveit_config = _edgepick_moveit_config()

    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": True,
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "monitor_dynamics": False,
    }

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {"publish_frequency": LaunchConfiguration("publish_frequency")},
        ],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[moveit_config.robot_description, str(controllers_file)],
    )

    spawners = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[controller, "--controller-manager", "/controller_manager"],
            output="screen",
        )
        for controller in [
            "joint_state_broadcaster",
            "arm_group_controller",
            "grip_group_controller",
        ]
    ]

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
            DeclareLaunchArgument("use_rviz", default_value="true"),
            robot_state_publisher,
            ros2_control_node,
            *spawners,
            move_group,
            rviz,
        ]
    )
