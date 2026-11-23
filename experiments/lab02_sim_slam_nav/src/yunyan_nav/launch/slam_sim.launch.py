"""一键启动 SLAM Toolbox 在线异步建图与 RViz2 可视化."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """生成并返回 SLAM 建图 Launch 描述."""
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    launch_sim = LaunchConfiguration('launch_sim')
    launch_rviz = LaunchConfiguration('launch_rviz')

    yunyan_nav_share = FindPackageShare('yunyan_nav')
    yunyan_sim_share = FindPackageShare('yunyan_sim')
    slam_toolbox_share = FindPackageShare('slam_toolbox')

    default_params_file = PathJoinSubstitution(
        [yunyan_nav_share, 'config', 'mapper_params_online_async.yaml']
    )
    rviz_config_path = PathJoinSubstitution(
        [yunyan_nav_share, 'rviz', 'slam_view.rviz']
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file for slam_toolbox'
    )

    declare_launch_sim_cmd = DeclareLaunchArgument(
        'launch_sim',
        default_value='false',
        description='Whether to also launch Gazebo simulation (yunyan_sim)'
    )

    declare_launch_rviz_cmd = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Whether to launch RViz2 with slam_view.rviz'
    )

    # 1. 终端提示：若 launch_sim 为 false，提示用户需单独启动仿真
    sim_notice = LogInfo(
        condition=UnlessCondition(launch_sim),
        msg='[yunyan_nav] launch_sim is False: expecting Gazebo simulation '
            'to run (e.g. ros2 launch yunyan_sim sim_bringup.launch.py).'
    )

    # 2. 可选联合拉起 Gazebo 仿真环境 (yunyan_sim)
    sim_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution(
                [yunyan_sim_share, 'launch', 'sim_bringup.launch.py']
            )]
        ),
        condition=IfCondition(launch_sim),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 3. 官方 slam_toolbox online_async_launch.py
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution(
                [slam_toolbox_share, 'launch', 'online_async_launch.py']
            )]
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_file
        }.items()
    )

    # 4. RViz2 (加载 slam_view.rviz，显式设置 use_sim_time: True 避免时钟不同步)
    rviz_node = Node(
        condition=IfCondition(launch_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2_slam',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_slam_params_file_cmd,
        declare_launch_sim_cmd,
        declare_launch_rviz_cmd,
        sim_notice,
        sim_bringup,
        slam_toolbox_launch,
        rviz_node,
    ])
