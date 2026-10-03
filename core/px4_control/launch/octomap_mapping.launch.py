from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('px4_control')
    default_params = os.path.join(pkg_share, 'config', 'octomap_tunnel.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='octomap 参数文件',
        ),
        DeclareLaunchArgument(
            'cloud_topic',
            default_value='/cloud_registered_body',
            description='机体坐标系点云（需要有 camera_init -> body 的 TF）',
        ),
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
            remappings=[
                ('cloud_in', LaunchConfiguration('cloud_topic')),
            ],
        ),
    ])
