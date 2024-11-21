import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_name = "herminebot_gazebo"

    pkg_share = FindPackageShare(package=pkg_name).find(pkg_name)
    pkg_gazebo_ros = FindPackageShare(package="gazebo_ros").find("gazebo_ros")
    world_path = os.path.join(pkg_share, "worlds/")

    if "GAZEBO_MODEL_PATH" not in os.environ:
        os.environ["GAZEBO_MODEL_PATH"] = ""
    os.environ["GAZEBO_MODEL_PATH"] += ":" + os.path.join(pkg_share, "models")

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
        value=[TextSubstitution(text=world_path), world_color, TextSubstitution(text=".world")]
    )

    # Start nodes and launches
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, "launch", "gzserver.launch.py")),
        condition=IfCondition(use_simulator),
        launch_arguments={"world": world_file}.items()
    )

    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, "launch", "gzclient.launch.py")),
        condition=IfCondition(PythonExpression([use_simulator, " and not ", headless]))
    )

    start_laser_to_range_nodes = [
        Node(
            package="herminebot_gazebo",
            executable="laser_to_range_node.py",
            name=f"id_{i}",
            namespace="laser_to_range_node",
            parameters=[{
                "source_topic": f"/laser_sensor_scan/id_{i}",
                "output_topic": f"/laser_sensor_range/id_{i}"
            }]
        ) for i in range(1, 5)
    ]

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add declarations
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_world_color_cmd)
    ld.add_action(declare_world_file_cmd)

    # Add nodes and actions
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(start_gazebo_server_cmd)

    for start_node in start_laser_to_range_nodes:
        ld.add_action(start_node)

    return ld
