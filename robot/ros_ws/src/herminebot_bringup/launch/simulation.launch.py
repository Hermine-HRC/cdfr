import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_description = FindPackageShare(package="herminebot_description").find("herminebot_description")
    pkg_gazebo = FindPackageShare(package="herminebot_gazebo").find("herminebot_gazebo")
    pkg_nav = FindPackageShare(package="herminebot_navigation").find("herminebot_navigation")

    initial_pose = {axis: LaunchConfiguration(axis) for axis in ("x", "y", "z", "yaw")}
    world_color = LaunchConfiguration("world_color", default="full")
    use_nav2 = LaunchConfiguration("use_nav2")

    declare_initial_pose_cmd = [
        DeclareLaunchArgument(
            name=axis,
            default_value="0.0" if axis != "z" else "1.0",
            description=f"Initial pose of the robot along {axis} axis in SI"
        ) for axis in initial_pose.keys()
    ]

    declare_launch_nav2_cmd = DeclareLaunchArgument(
        name="use_nav2",
        default_value="True",
        description="Whether launch the nav2 stack"
    )

    # Start launches
    start_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_description, "launch", "description.launch.py")),
    )

    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo, "launch", "world.launch.py")),
        launch_arguments={
            "world_color": world_color
        }.items()
    )

    start_spawner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo, "launch", "spawn_herminebot.launch.py")),
        launch_arguments=initial_pose.items()
    )

    start_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav, "launch", "navigation.launch.py")),
        launch_arguments={"map": world_color}.items(),
        condition=IfCondition(use_nav2)
    )

    ld = LaunchDescription()

    ld.add_action(declare_world_cmd)
    ld.add_action(declare_launch_nav2_cmd)
    for pose in declare_initial_pose_cmd:
        ld.add_action(pose)

    ld.add_action(start_nav)
    ld.add_action(start_description)
    ld.add_action(start_gazebo)
    ld.add_action(start_spawner)

    return ld
