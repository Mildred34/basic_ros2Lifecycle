from launch import LaunchDescription
from launch.actions import EmitEvent
from launch.events import Shutdown
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    ld = LaunchDescription()

    checker_node = Node(
            package="manager",
            executable="node_checker",
            name='node_checker',
            output="both"
        )
        
    manager_node = Node(
        package='manager',
        executable='lc_manager',
        name='lc_manager',
        namespace='',
        output='both',
        on_exit=EmitEvent(event=Shutdown(reason='Window closed'))
    )

    lifecycle_talker_node = LifecycleNode(
        package='lifecycle_talker',
        executable='lifecycle_talker',
        name='lc_talker',
        namespace='',
        output='both'
    )
    
    lifecycle_listener_node = LifecycleNode(
        package='lifecycle_listener',
        executable='lifecycle_listener',
        name='lc_listener',
        namespace='',
        output='both'
    )

    ld.add_action(manager_node)
    ld.add_action(checker_node)
    ld.add_action(lifecycle_talker_node)
    ld.add_action(lifecycle_listener_node)

    return ld