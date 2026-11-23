"""Bringup launch file for PadBot E2 ROS 2 bridge."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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
    """Generate launch description for PadBot E2 bridge."""
    base_url = LaunchConfiguration('base_url')
    mock = LaunchConfiguration('mock')
    launch_rviz = LaunchConfiguration('launch_rviz')
    map_export_dir = LaunchConfiguration('map_export_dir')

    declare_base_url_cmd = DeclareLaunchArgument(
        'base_url',
        default_value='http://127.0.0.1:8080',
        description='E2 navigation module base URL',
    )

    declare_mock_cmd = DeclareLaunchArgument(
        'mock',
        default_value='true',
        description='Launch local E2 mock server if true',
    )

    declare_launch_rviz_cmd = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Whether to launch RViz2',
    )

    declare_map_export_dir_cmd = DeclareLaunchArgument(
        'map_export_dir',
        default_value='',
        description='Custom local directory to export map files',
    )

    pkg_share = FindPackageShare('yunyan_e2_bridge')

    urdf_path = PathJoinSubstitution(
        [pkg_share, 'urdf', 'e2_base.urdf.xacro']
    )
    rviz_config_path = PathJoinSubstitution(
        [pkg_share, 'rviz', 'e2_view.rviz']
    )
    params_path = PathJoinSubstitution(
        [pkg_share, 'config', 'e2_bridge_params.yaml']
    )

    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', urdf_path]),
        value_type=str,
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'robot_description': robot_description,
        }],
    )

    # Mock server node
    mock_server_node = Node(
        package='yunyan_e2_bridge',
        executable='e2_mock_server',
        name='e2_mock_server',
        output='screen',
        condition=IfCondition(mock),
    )

    # Core Bridge Node
    bridge_node = Node(
        package='yunyan_e2_bridge',
        executable='e2_bridge',
        name='e2_bridge_node',
        output='screen',
        parameters=[
            params_path,
            {
                'base_url': base_url,
                'map_export_dir': map_export_dir,
                'use_sim_time': False,
            },
        ],
    )

    # RViz2 Node
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen',
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(launch_rviz),
    )

    return LaunchDescription([
        declare_base_url_cmd,
        declare_mock_cmd,
        declare_launch_rviz_cmd,
        declare_map_export_dir_cmd,
        robot_state_publisher_node,
        mock_server_node,
        bridge_node,
        rviz2_node,
    ])
