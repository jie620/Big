"""Start the EdgePick task node plus a state-gated mock task driver."""

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
        name="edgepick_mock_task_driver",
        output="screen",
        parameters=[
            {
                "scenario": LaunchConfiguration("scenario"),
                "event_period_ms": LaunchConfiguration("event_period_ms"),
                "initial_delay_ms": LaunchConfiguration("initial_delay_ms"),
                "event_topic": event_topic,
                "state_topic": state_topic,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("scenario", default_value="success"),
            DeclareLaunchArgument("event_period_ms", default_value="300"),
            DeclareLaunchArgument("initial_delay_ms", default_value="500"),
            DeclareLaunchArgument("max_recovery_attempts", default_value="2"),
            task_node,
            mock_task_driver,
        ]
    )
