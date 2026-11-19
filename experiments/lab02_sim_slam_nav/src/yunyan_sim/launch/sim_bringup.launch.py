"""一键启动 Gazebo 仿真环境与 RViz2 可视化."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """生成并返回仿真环境 Launch 描述."""
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_rviz = LaunchConfiguration('launch_rviz')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_launch_rviz_cmd = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Whether to launch RViz2'
    )

    yunyan_sim_share = FindPackageShare('yunyan_sim')
    gazebo_ros_share = FindPackageShare('gazebo_ros')

    world_path = PathJoinSubstitution(
        [yunyan_sim_share, 'worlds', 'slam_room.world']
    )
    urdf_path = PathJoinSubstitution(
        [yunyan_sim_share, 'urdf', 'yunyan_car.urdf.xacro']
    )
    rviz_config_path = PathJoinSubstitution(
        [yunyan_sim_share, 'rviz', 'sim_view.rviz']
    )

    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', urdf_path]),
        value_type=str
    )

    # 1. 启动 Gazebo 服务端与客户端并加载封闭房间世界
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution(
                [gazebo_ros_share, 'launch', 'gazebo.launch.py']
            )]
        ),
        launch_arguments={'world': world_path}.items()
    )

    # 2. 发布机器人模型与 TF 树
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }]
    )

    # 3. 在仿真世界原点生成小车模型
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'yunyan_car',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.05'
        ],
        output='screen'
    )

    # 4. 启动 RViz2 可视化界面
    rviz_node = Node(
        condition=IfCondition(launch_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_launch_rviz_cmd,
        gazebo,
        robot_state_publisher_node,
        spawn_entity_node,
        rviz_node,
    ])
