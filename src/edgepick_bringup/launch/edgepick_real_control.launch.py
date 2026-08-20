"""Start the EdgePick ros2_control chain with explicit real I2C enablement."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _robot_description():
    edgepick_share = FindPackageShare("edgepick_bringup")
    xacro_file = PathJoinSubstitution([edgepick_share, "urdf", "edgepick_dofbot.urdf.xacro"])
    initial_positions = PathJoinSubstitution([edgepick_share, "config", "initial_positions.yaml"])

    return {
        "robot_description": ParameterValue(
            Command(
                [
                    FindExecutable(name="xacro"),
                    " ",
                    xacro_file,
                    " ",
                    "initial_positions_file:=",
                    initial_positions,
                    " ",
                    "use_real_i2c:=",
                    LaunchConfiguration("use_real_i2c"),
                    " ",
                    "i2c_device:=",
                    LaunchConfiguration("i2c_device"),
                    " ",
                    "i2c_address:=",
                    LaunchConfiguration("i2c_address"),
                ]
            ),
            value_type=str,
        )
    }


def generate_launch_description():
    edgepick_share = FindPackageShare("edgepick_bringup")
    controllers_file = PathJoinSubstitution(
        [edgepick_share, "config", "edgepick_ros2_controllers.yaml"]
    )
    robot_description = _robot_description()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {"publish_frequency": LaunchConfiguration("publish_frequency")},
        ],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[robot_description, controllers_file],
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

    return LaunchDescription(
        [
            DeclareLaunchArgument("publish_frequency", default_value="15.0"),
            DeclareLaunchArgument("use_real_i2c", default_value="true"),
            DeclareLaunchArgument("i2c_device", default_value="/dev/i2c-7"),
            DeclareLaunchArgument("i2c_address", default_value="0x15"),
            robot_state_publisher,
            ros2_control_node,
            *spawners,
        ]
    )
