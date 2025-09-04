import os

from ament_index_python.packages import get_package_share_directory
import hrc_utils
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    os.environ['PYTHONPATH'] += ':' + hrc_utils.get_venv_site_packages_dir()

    params_file = os.path.join(
        get_package_share_directory('herminebot_hardware'),
        'config',
        'herminebot.yaml'
    )

    i2c_interface_node = Node(
        package='herminebot_hardware',
        executable='i2c_interface_node',
        parameters=[params_file]
    )

    ldlidar_node = Node(
        package='ldlidar_stl_ros2',
        executable='ldlidar_stl_ros2_node',
        parameters=[params_file]
    )

    ld = LaunchDescription()

    ld.add_action(i2c_interface_node)
    ld.add_action(ldlidar_node)

    return ld
