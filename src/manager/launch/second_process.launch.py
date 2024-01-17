from launch import LaunchDescription
from launch.actions import EmitEvent
from launch.events import Shutdown
from launch_ros.actions import LifecycleNode, Node
from launch.event_handlers import OnProcessExit
from launch.actions import RegisterEventHandler


def generate_launch_description():
    # Second Process
    Second_process_nodes = []
    manager_node_2 = Node(
        package="manager",
        executable="lc_manager2",
        name="lc_manager2",
        namespace="",
        output="both",
        arguments=[
            "--ros-args",
            "--log-level",
            "info",
        ],
        # on_exit=EmitEvent(event=Shutdown(reason="Window closed")),
    )
    Second_process_nodes.append(manager_node_2)

    lifecycle_talker2_node = LifecycleNode(
        package="lifecycle_talker2",
        executable="lifecycle_talker2",
        name="lc_talker2",
        namespace="",
        output="both",
        arguments=[
            "--ros-args",
            "--log-level",
            "info",
        ],
    )

    Second_process_nodes.append(lifecycle_talker2_node)

    return LaunchDescription(Second_process_nodes)
