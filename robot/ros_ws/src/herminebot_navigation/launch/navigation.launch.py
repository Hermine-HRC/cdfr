import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_name = "herminebot_navigation"
    map_file_name = "board_map.yaml"
    param_file_name = "nav2_params.yaml"

    pkg_share = FindPackageShare(package=pkg_name).find(pkg_name)
    map_file_path = os.path.join(pkg_share, "maps", map_file_name)
    nav_params = os.path.join(pkg_share, "params", param_file_name)

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    slam = LaunchConfiguration('slam', default='false')
    map_dir = LaunchConfiguration('map', default=map_file_path)

    param_dir = LaunchConfiguration('params_file', default=nav_params)

    rviz_config_dir = os.path.join(pkg_share, 'rviz', 'nav2_config.rviz')

    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')


    declare_map_path_cmd = DeclareLaunchArgument(
        'map',
        default_value=map_dir,
        description='Full path to map file to load'
    )

    declare_slam_cmd = DeclareLaunchArgument(
        name='slam',
        default_value='False',
        description='Whether to run SLAM'
    )

    declare_params_path_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=param_dir,
        description='Full path to param file to load'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value=use_sim_time,
        description='Use simulation (Gazebo) clock if true'
    )

    start_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch_file_dir,'/bringup_launch.py']),
        launch_arguments={
            'map': map_dir,
            'slam': slam,
            'use_sim_time': use_sim_time,
            'params_file': param_dir}.items(),
    )

    start_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(declare_params_path_cmd)
    ld.add_action(declare_map_path_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_slam_cmd)

    ld.add_action(start_nav2)
    ld.add_action(start_rviz)

    return ld
