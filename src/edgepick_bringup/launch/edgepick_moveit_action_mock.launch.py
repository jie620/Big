"""Start task node, mock perception/verification, and mock-safe MoveIt action adapter."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    event_topic = "/edgepick/task/event"
    state_topic = "/edgepick/task/state"
    failure_topic = "/edgepick/task/failure"
    diagnostics_topic = "/diagnostics"

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
        name="edgepick_mock_perception_verification_driver",
        output="screen",
        parameters=[
            {
                "scenario": "moveit_success",
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
                "use_mock_action_results": LaunchConfiguration("use_mock_action_results"),
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
            DeclareLaunchArgument("event_period_ms", default_value="300"),
            DeclareLaunchArgument("initial_delay_ms", default_value="500"),
            DeclareLaunchArgument("action_result_delay_ms", default_value="300"),
            DeclareLaunchArgument("max_recovery_attempts", default_value="2"),
            DeclareLaunchArgument("use_mock_action_results", default_value="true"),
            DeclareLaunchArgument("planning_outcome", default_value="success"),
            DeclareLaunchArgument("execution_outcome", default_value="success"),
            task_node,
            mock_task_driver,
            moveit_action_adapter,
        ]
    )
