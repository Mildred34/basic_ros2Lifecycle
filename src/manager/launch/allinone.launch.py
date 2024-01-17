from launch import LaunchDescription
from launch.actions import EmitEvent
from launch.events import Shutdown
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    nodes = []

    checker_node = Node(
        package="manager", executable="node_checker", name="node_checker", output="both"
    )
    nodes.append(checker_node)

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
        on_exit=EmitEvent(event=Shutdown(reason="Window closed")),
    )
    nodes.append(manager_node)

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
    nodes.append(lifecycle_talker_node)
    nodes.append(lifecycle_listener_node)

    return LaunchDescription(nodes)
