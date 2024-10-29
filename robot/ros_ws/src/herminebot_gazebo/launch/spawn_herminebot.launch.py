import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the urdf file
    urdf_path = os.path.join(
        get_package_share_directory("herminebot_gazebo"),
        "models",
        "herminebot",
        "model.sdf"
    )

    # Launch configuration variables specific to simulation
    initial_pose = {axis: LaunchConfiguration(axis, default="0.0") for axis in ("x", "y", "z", "yaw")}

    # Declare the launch arguments
    declare_initial_pose_cmd = [
        DeclareLaunchArgument(
            name=axis,
            default_value="0",
            description=f"Initial pose of the robot along {axis} axis in SI"
        ) for axis in initial_pose.keys()
    ]

    start_gazebo_ros_spawner_cmd = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "herminebot",
            "-file", urdf_path,
            "-x", initial_pose["x"],
            "-y", initial_pose["y"],
            "-z", initial_pose["z"],
            "-Y", initial_pose["yaw"]
        ],
        output="screen",
    )

    ld = LaunchDescription()

    # Declare the launch options
    for pose in declare_initial_pose_cmd:
        ld.add_action(pose)

    # Add any conditioned actions
    ld.add_action(start_gazebo_ros_spawner_cmd)

    return ld