from launch import LaunchDescription
from launch.actions import EmitEvent
from launch.events import Shutdown
from launch_ros.actions import LifecycleNode, Node
from launch.event_handlers import OnProcessExit
from launch.actions import RegisterEventHandler
from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    event_handlers = []
    First_process_nodes = []

    checker_node = Node(
        package="manager", executable="node_checker", name="node_checker", output="both"
    )
    event_handlers.append(checker_node)

    # First Process
    manager_node = Node(
        package="manager",
        executable="lc_manager",
        name="lc_manager",
        namespace="",
        output="both",
        arguments=[
            "--ros-args",
            "--log-level",
            "info",
        ],
        # on_exit=EmitEvent(event=Shutdown(reason="Window closed")),
    )
    First_process_nodes.append(manager_node)

    lifecycle_talker_node = LifecycleNode(
        package="lifecycle_talker",
        executable="lifecycle_talker",
        name="lc_talker",
        namespace="",
        output="both",
        arguments=[
            "--ros-args",
            "--log-level",
            "info",
        ],
    )

    lifecycle_listener_node = LifecycleNode(
        package="lifecycle_listener",
        executable="lifecycle_listener",
        name="lc_listener",
        namespace="",
        output="both",
        arguments=[
            "--ros-args",
            "--log-level",
            "info",
        ],
    )
    First_process_nodes.append(lifecycle_talker_node)
    First_process_nodes.append(lifecycle_listener_node)

    for node in First_process_nodes:
        event_handlers.append(node)

    # Second Process
    package_process_name = "manager"
    launch_descriptions = []
    launch_descriptions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare(package_process_name),
                        "launch",
                        "second_process.launch.py",
                    ]
                )
            ),
        )
    )

    event_handlers.append(
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=First_process_nodes[0],
                on_exit=[
                    TimerAction(
                        period=2.0,
                        actions=[launch_descriptions[0]],
                    )
                ],
            )
        )
    )

    return LaunchDescription(event_handlers)
