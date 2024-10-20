import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_description = FindPackageShare(package="herminebot_description").find("herminebot_description")
    pkg_nav = FindPackageShare(package="herminebot_navigation").find("herminebot_navigation")

    initial_pose = {axis: LaunchConfiguration(axis) for axis in ("x", "y")}
    world = LaunchConfiguration("world")

    declare_world_cmd = DeclareLaunchArgument(
        name="world",
        default_value="full",
        choices=["full", "yellow", "blue"],
        description="World and map name to load"
    )

    declare_initial_pose_cmd = [
        DeclareLaunchArgument(
            name=axis,
            default_value="0",
            description=f"Initial pose of the robot along {axis} axis in m"
        ) for axis in initial_pose.keys()
    ]

    # Start launches
    description_args = {"world": world}
    description_args.update(initial_pose)
    start_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_description, "launch", "description.launch.py")),
        launch_arguments={
            "world": world
        }.items(),
    )

    nav_args = {"map": world}
    nav_args.update(initial_pose)
    start_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav, "launch", "navigation.launch.py")),
        launch_arguments=nav_args.items(),
    )

    ld = LaunchDescription()

    ld.add_action(declare_world_cmd)
    for pose in declare_initial_pose_cmd:
        ld.add_action(pose)

    ld.add_action(start_nav)
    ld.add_action(start_description)

    return ld
