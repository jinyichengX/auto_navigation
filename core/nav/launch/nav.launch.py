from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('nav')
    default_params = os.path.join(pkg, 'config', 'nav.yaml')

    params_file = LaunchConfiguration('params_file')
    input_map = LaunchConfiguration('input_map_topic')
    odom_topic = LaunchConfiguration('odom_topic')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('input_map_topic', default_value='/projected_map'),
        DeclareLaunchArgument('odom_topic', default_value='/Odometry'),

        Node(
            package='nav',
            executable='map_inflation',
            name='map_inflation',
            output='screen',
            parameters=[
                params_file,
                {'input_map_topic': input_map},
            ],
        ),
        Node(
            package='nav',
            executable='rrt_planner',
            name='rrt_planner',
            output='screen',
            parameters=[
                params_file,
                {'odom_topic': odom_topic},
            ],
        ),
        Node(
            package='nav',
            executable='local_planner',
            name='local_planner',
            output='screen',
            parameters=[
                params_file,
                {'odom_topic': odom_topic},
            ],
        ),
        Node(
            package='nav',
            executable='path_tracker',
            name='path_tracker',
            output='screen',
            parameters=[params_file],
        ),
    ])
