import os

from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription, \
    SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_name = "herminebot_gazebo"

    pkg_share = FindPackageShare(package=pkg_name).find(pkg_name)
    pkg_gazebo_ros = FindPackageShare(package="ros_gz_sim").find("ros_gz_sim")
    world_path = os.path.join(pkg_share, "worlds/")

    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(pkg_share, 'models')
    )

    # Launch configuration variables
    headless = LaunchConfiguration("headless")
    use_simulator = LaunchConfiguration("use_simulator")
    world_color = LaunchConfiguration("world_color")
    world_file = LaunchConfiguration("world_file")

    # Declare launch arguments
    declare_simulator_cmd = DeclareLaunchArgument(
        name="headless",
        default_value="False",
        description="Whether to execute gzclient"
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true"
    )

    declare_use_simulator_cmd = DeclareLaunchArgument(
        name="use_simulator",
        default_value="True",
        description="Whether to start the simulator"
    )

    declare_world_color_cmd = DeclareLaunchArgument(
        name="world_color",
        default_value="full",
        choices=["full", "yellow", "blue"],
        description="World to load in Gazebo"
    )

    declare_world_file_cmd = SetLaunchConfiguration(
        name="world_file",
        value=[TextSubstitution(text=world_path), world_color, TextSubstitution(text=".sdf")]
    )

    # Start nodes and launches
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, "launch", "gz_sim.launch.py")),
        condition=IfCondition(use_simulator),
        launch_arguments={'gz_args': ['-s -r -v1 ', world_file], 'on_exit_shutdown': 'true'}.items()
    )

    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, "launch", "gz_sim.launch.py")),
        launch_arguments={'gz_args': f'-g -v1 --gui-config {world_path}gui.config'}.items(),
        condition=IfCondition(PythonExpression([use_simulator, " and not ", headless]))
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add declarations
    ld.add_action(set_env_vars_resources)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_world_color_cmd)
    ld.add_action(declare_world_file_cmd)

    # Add nodes and actions
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(start_gazebo_server_cmd)

    return ld
