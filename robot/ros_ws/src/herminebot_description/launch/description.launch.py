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

    # Pose where we want to spawn the robot
    spawn_x_val = "0.0"
    spawn_y_val = "0.0"
    spawn_z_val = "1.02"
    spawn_yaw_val = "0.0"

    pkg_share = FindPackageShare(package=pkg_name).find(pkg_name)
    pkg_gazebo = FindPackageShare(package="herminebot_gazebo").find("herminebot_gazebo")
    robot_localization_file_path = os.path.join(pkg_share, "config", localization_file_name)
    default_robot_path = os.path.join(pkg_share, "models", robot_model_file_name)

    if "GAZEBO_MODEL_PATH" not in os.environ:
        os.environ["GAZEBO_MODEL_PATH"] = ""
    os.environ["GAZEBO_MODEL_PATH"] += ":" + os.path.join(pkg_share, "models")

    # Launch configuration variables
    gui = LaunchConfiguration("gui")
    use_sim_time = LaunchConfiguration('use_sim_time')
    urdf_model = LaunchConfiguration("urdf_model")

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
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo, "launch", "world.launch.py"))
    )

    spawn_entity_cmd = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", robot_name_in_model,
                   "-topic", "robot_description",
                   "-x", spawn_x_val,
                   "-y", spawn_y_val,
                   "-z", spawn_z_val,
                   "-Y", spawn_yaw_val],
        output="screen"
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
    ld.add_action(start_gazebo)
    ld.add_action(start_robot_localization_cmd)
    ld.add_action(spawn_entity_cmd)

    return ld
