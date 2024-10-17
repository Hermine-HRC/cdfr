import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_description = FindPackageShare(package="herminebot_description").find("herminebot_description")
    pkg_nav = FindPackageShare(package="herminebot_navigation").find("herminebot_navigation")

    world = LaunchConfiguration("world")

    declare_world_cmd = DeclareLaunchArgument(
        name="world",
        default_value="full",
        choices=["full", "yellow", "blue"],
        description="World and map name to load"
    )

    # Start launches
    start_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_description, "launch", "description.launch.py")),
        launch_arguments={
            "world": world
        }.items(),
    )

    start_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav, "launch", "navigation.launch.py")),
        launch_arguments={
            "map": world
        }.items(),
    )

    ld = LaunchDescription()

    ld.add_action(declare_world_cmd)

    ld.add_action(start_nav)
    ld.add_action(start_description)

    return ld
