from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('mine_nav')
    default_params = os.path.join(pkg, 'config', 'default_mission.yaml')

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='任务与四层节点参数'
    )
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        params_arg,
        Node(
            package='mine_nav',
            executable='task_manager',
            name='task_manager',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='mine_nav',
            executable='explorer',
            name='explorer',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='mine_nav',
            executable='global_planner',
            name='global_planner',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='mine_nav',
            executable='local_controller',
            name='local_controller',
            output='screen',
            parameters=[params_file],
        ),
    ])
