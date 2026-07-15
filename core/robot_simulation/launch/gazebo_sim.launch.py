import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # 包路径
    pkg_share = get_package_share_directory('robot_simulation')

    # 默认 xacro 和 world 文件路径
    default_model_path = os.path.join(pkg_share, 'urdf/fishrot/fishbot.xacro')
    default_world_path = os.path.join(pkg_share, 'world/world1.sdf') #字符串拼接路径

    # 运行时参数：模型路径、世界路径
    model_arg = DeclareLaunchArgument(
        'model', default_value=str(default_model_path),
        description='fishbot xacro 文件的绝对路径'
    )
    world_arg = DeclareLaunchArgument(
        'world', default_value=str(default_world_path),
        description='Gazebo 世界 SDF 文件的绝对路径'
    )

    # WSL2 需要软件渲染，否则 Gazebo 黑屏/崩溃
    libgl_env = SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1')

    # 将 xacro 展开为 URDF，注入 /robot_description 参数
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str
    )

    # 1. Robot State Publisher：广播 TF + 发布 /robot_description
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    # 2. Joint State Publisher：发布关节状态（驱动连续转动）
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{
            'use_sim_time': True,
            'rate': 50.0,
        }],
        output='screen'
    )

    # 3. 启动 Gazebo Fortress（使用 ros_gz_sim 的官方 launch）
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': ['-r ', LaunchConfiguration('world')],
        }.items()
    )

    # 4. Spawn 机器人到 Gazebo（ros_gz_sim 内部自动 URDF→SDF 转换）
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'fishbot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0',
        ],
        output='screen'
    )

    return LaunchDescription([
        libgl_env,
        model_arg,
        world_arg,
        robot_state_publisher,
        joint_state_publisher,
        gz_sim,
        spawn_robot,
    ])
