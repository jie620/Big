"""Start the EdgePick task node with mock string-event input."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
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

    return LaunchDescription(
        [
            DeclareLaunchArgument("max_recovery_attempts", default_value="2"),
            task_node,
        ]
    )
