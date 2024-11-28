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

    bridge_params = os.path.join(
        get_package_share_directory('herminebot_gazebo'),
        'config',
        'ros_gz_config.yaml'
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

    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen',
    )

    start_gazebo_ros_spawner_cmd = Node(
        package="ros_gz_sim",
        executable="create",
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

    start_laser_to_range_nodes = [
        Node(
            package="herminebot_gazebo",
            executable="laser_to_range_node.py",
            name=f"id_{i}",
            namespace="laser_to_range_node",
            parameters=[{
                "source_topic": f"/gz/laser_sensor_scan/id_{i}",
                "output_topic": f"/laser_sensor_range/id_{i}",
                "frame_id": f"laser_sensor_{i}_link"
            }]
        ) for i in range(1, 5)
    ]

    start_frame_switcher = Node(
        package="herminebot_gazebo",
        executable="frame_switcher_node.py"
    )

    ld = LaunchDescription()

    # Declare the launch options
    for pose in declare_initial_pose_cmd:
        ld.add_action(pose)

    # Add any conditioned actions
    ld.add_action(start_gazebo_ros_spawner_cmd)
    ld.add_action(start_gazebo_ros_bridge_cmd)

    for start_node in start_laser_to_range_nodes:
        ld.add_action(start_node)

    ld.add_action(start_frame_switcher)

    return ld
