import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    herminebot_model: str = os.environ.get('HERMINEBOT_MODEL', 'diff')
    pkg_name = 'herminebot_description'
    robot_model_file_name = f'herminebot_{herminebot_model}.urdf'
    localization_file_name = f'ekf_{herminebot_model}.yaml'

    pkg_share = FindPackageShare(package=pkg_name).find(pkg_name)
    robot_localization_file_path = os.path.join(pkg_share, 'config', localization_file_name)
    herminebot_description_config_file_path = os.path.join(pkg_share, 'config', f'herminebot_{herminebot_model}.yaml')
    default_robot_path = os.path.join(pkg_share, 'models', robot_model_file_name)

    # Launch configuration variables
    gui = LaunchConfiguration('gui')
    use_sim_time = LaunchConfiguration('use_sim_time')
    urdf_model = LaunchConfiguration('urdf_model')

    # Declare the launch arguments
    declare_use_joint_state_publisher_cmd = DeclareLaunchArgument(
        name='gui',
        default_value='True',
        description='Flag to enable joint_state_publisher_gui'
    )

    declare_urdf_model_path_cmd = DeclareLaunchArgument(
        name='urdf_model',
        default_value=default_robot_path,
        description='Absolute path to robot urdf file'
    )

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        name='use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # Start nodes and launches
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', urdf_model])}]
    )

    start_joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=UnlessCondition(gui)
    )

    start_robot_localization_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[robot_localization_file_path, {'use_sim_time': use_sim_time}]
    )

    start_global_localization_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_global',
        output='screen',
        parameters=[robot_localization_file_path, {'use_sim_time': use_sim_time}]
    )

    start_services_server = Node(
        package='herminebot_description',
        executable='services_common_server.py',
        name='services_common_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    start_omnibot_cmd_converter = None
    if herminebot_model == 'omni':
        start_omnibot_cmd_converter = Node(
            package='herminebot_description',
            executable='omnibot_cmd_converter.py',
            output='screen',
            parameters=[herminebot_description_config_file_path]
        )

    ld = LaunchDescription()

    # Add declarations
    ld.add_action(declare_use_joint_state_publisher_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_urdf_model_path_cmd)

    # Add nodes and actions
    ld.add_action(start_joint_state_publisher_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_robot_localization_cmd)
    ld.add_action(start_services_server)
    if herminebot_model == 'omni' and start_omnibot_cmd_converter is not None:
        ld.add_action(start_omnibot_cmd_converter)
        ld.add_action(start_global_localization_cmd)

    return ld
