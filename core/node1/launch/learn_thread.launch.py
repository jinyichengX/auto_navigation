from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='node1',
            executable='process_learn_thread',
            name='learn_thread_node',
            output='screen'
        ),
        Node(
            package='node1',
            executable='process1',
            name='python_node',
            output='screen'
        ),
    ])
