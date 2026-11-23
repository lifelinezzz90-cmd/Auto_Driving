"""一键启动 Nav2 仿真导航栈 (支持静态地图导航与 SLAM 联调双模式)."""

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
    """生成并返回 Nav2 仿真导航 Launch 描述."""
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    slam = LaunchConfiguration('slam')
    launch_rviz = LaunchConfiguration('launch_rviz')

    yunyan_nav_share = FindPackageShare('yunyan_nav')
    nav2_bringup_share = FindPackageShare('nav2_bringup')
    slam_toolbox_share = FindPackageShare('slam_toolbox')

    default_map_file = PathJoinSubstitution(
        [yunyan_nav_share, 'maps', 'slam_room_map.yaml']
    )
    default_params_file = PathJoinSubstitution(
        [yunyan_nav_share, 'config', 'nav2_params.yaml']
    )
    default_slam_params_file = PathJoinSubstitution(
        [yunyan_nav_share, 'config', 'mapper_params_online_async.yaml']
    )
    rviz_config_path = PathJoinSubstitution(
        [yunyan_nav_share, 'rviz', 'nav2_view.rviz']
    )

    # 1. 声明启动参数
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=default_map_file,
        description='Full path to map yaml file to load'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file for Nav2'
    )

    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Whether to run SLAM (True) or static localization (False)'
    )

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=default_slam_params_file,
        description='Full path to the ROS2 parameters file for slam_toolbox'
    )

    declare_launch_rviz_cmd = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Whether to launch RViz2 with nav2_view.rviz'
    )

    # 2. 状态提示信息
    static_mode_notice = LogInfo(
        condition=UnlessCondition(slam),
        msg='[yunyan_nav] 模式 1: 已知静态地图自主导航 (map_server + amcl).'
    )

    slam_mode_notice = LogInfo(
        condition=IfCondition(slam),
        msg='[yunyan_nav] 模式 2: 实时 SLAM 建图 + Nav2 导航联调 (slam_toolbox).'
    )

    # 3. 模式 1 (slam:=False): 静态地图定位 (map_server + amcl)
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [nav2_bringup_share, 'launch', 'localization_launch.py']
            )
        ),
        condition=UnlessCondition(slam),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true'
        }.items()
    )

    # 4. 模式 2 (slam:=True): 实时 SLAM 异步在线建图
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [slam_toolbox_share, 'launch', 'online_async_launch.py']
            )
        ),
        condition=IfCondition(slam),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': LaunchConfiguration('slam_params_file')
        }.items()
    )

    # 5. Nav2 路径规划与控制核心节点 (双模式共用)
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [nav2_bringup_share, 'launch', 'navigation_launch.py']
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true'
        }.items()
    )

    # 6. RViz2 (加载专属 nav2_view.rviz，显式传入 use_sim_time)
    rviz_node = Node(
        condition=IfCondition(launch_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2_nav',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_map_yaml_cmd,
        declare_params_file_cmd,
        declare_slam_cmd,
        declare_slam_params_file_cmd,
        declare_launch_rviz_cmd,
        static_mode_notice,
        slam_mode_notice,
        localization_launch,
        slam_toolbox_launch,
        navigation_launch,
        rviz_node,
    ])
