import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_name = "herminebot_description"
    robot_name_in_model = "herminebot"
    robot_model_file_name =  "herminebot.urdf"
    localization_file_name = "ekf.yaml"

    z_offset = "1.0"

    pkg_share = FindPackageShare(package=pkg_name).find(pkg_name)
    pkg_gazebo = FindPackageShare(package="herminebot_gazebo").find("herminebot_gazebo")
    robot_localization_file_path = os.path.join(pkg_share, "config", localization_file_name)
    default_robot_path = os.path.join(pkg_share, "models", robot_model_file_name)

    if "GAZEBO_MODEL_PATH" not in os.environ:
        os.environ["GAZEBO_MODEL_PATH"] = ""
    os.environ["GAZEBO_MODEL_PATH"] += ":" + os.path.join(pkg_share, "models")

    # Launch configuration variables
    gui = LaunchConfiguration("gui")
    initial_pose = {axis: LaunchConfiguration(axis, default="0") for axis in ("x", "y")}
    use_sim_time = LaunchConfiguration('use_sim_time')
    urdf_model = LaunchConfiguration("urdf_model")
    world = LaunchConfiguration("world")
    z_offset_config = LaunchConfiguration("z_offset")

    # Declare the launch arguments
    declare_use_joint_state_publisher_cmd = DeclareLaunchArgument(
        name="gui",
        default_value="True",
        description="Flag to enable joint_state_publisher_gui"
    )

    declare_urdf_model_path_cmd = DeclareLaunchArgument(
        name="urdf_model",
        default_value=default_robot_path,
        description="Absolute path to robot urdf file"
    )

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        name="use_robot_state_pub",
        default_value="True",
        description="Whether to start the robot state publisher"
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true"
    )

    declare_world_cmd = DeclareLaunchArgument(
        name="world",
        default_value="full",
        choices=["full", "yellow", "blue"],
        description="World to load in Gazebo"
    )

    declare_z_offset_cmd = DeclareLaunchArgument(
        name="z_offset",
        default_value=z_offset,
        description="Altitude where to spawn the robot"
    )

    declare_initial_pose_cmd = [
        DeclareLaunchArgument(
            name=axis,
            default_value="0",
            description=f"Initial pose of the robot along {axis} axis in m"
        ) for axis in initial_pose.keys()
    ]

    # Start nodes and launches
    start_robot_state_publisher_cmd = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": Command(["xacro ", urdf_model])}]
    )

    start_joint_state_publisher_cmd = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        condition=UnlessCondition(gui)
    )

    start_robot_localization_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[robot_localization_file_path, {'use_sim_time': use_sim_time}]
    )

    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo, "launch", "world.launch.py")),
        launch_arguments={
            "world": world
        }.items(),
    )

    spawn_entity_cmd = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", robot_name_in_model,
            "-topic", "robot_description",
            "-x", initial_pose["x"],
            "-y", initial_pose["y"],
            "-z", z_offset_config,
            "-Y", "0.0"
        ],
        output="screen"
    )

    ld = LaunchDescription()

    # Add declarations
    ld.add_action(declare_use_joint_state_publisher_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_urdf_model_path_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_z_offset_cmd)
    for pose in declare_initial_pose_cmd:
        ld.add_action(pose)

    # Add nodes and actions
    ld.add_action(start_joint_state_publisher_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_gazebo)
    ld.add_action(start_robot_localization_cmd)
    #ld.add_action(spawn_entity_cmd)

    return ld
