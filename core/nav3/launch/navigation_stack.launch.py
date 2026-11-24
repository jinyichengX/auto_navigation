from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os


def generate_launch_description():
    fast_lio_launch = os.path.join(
        get_package_share_directory('fast_lio'), 'launch', 'mapping.launch.py'
    )
    octomap_launch = os.path.join(
        get_package_share_directory('px4_control'), 'launch', 'octomap_mapping.launch.py'
    )

    stages = [
        Node(package='nav3', executable='fastlio2_input_bridge', output='screen'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(fast_lio_launch),
            launch_arguments={'rviz': 'false'}.items(),
        ),
        Node(package='nav3', executable='fastlio_odom_bridge', output='screen'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(octomap_launch)),
        Node(package='nav3', executable='global_planner_3d', output='screen'),
        Node(package='nav3', executable='path_to_target', output='screen'),
    ]

    actions = [stages[-1]]
    for stage in reversed(stages[:-1]):
        actions = [stage, TimerAction(period=3.0, actions=actions)]
    return LaunchDescription(actions)
