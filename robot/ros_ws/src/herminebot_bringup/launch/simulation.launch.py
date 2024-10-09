import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_description = FindPackageShare(package="herminebot_description").find("herminebot_description")
    pkg_nav = FindPackageShare(package="herminebot_navigation").find("herminebot_navigation")

    # Start launches
    start_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_description, "launch", "description.launch.py"))
    )

    start_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav, "launch", "navigation.launch.py"))
    )

    ld = LaunchDescription()

    ld.add_action(start_nav)
    ld.add_action(start_description)

    return ld
