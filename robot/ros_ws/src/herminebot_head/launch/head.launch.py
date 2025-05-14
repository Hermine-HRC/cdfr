import os

from ament_index_python.packages import get_package_share_directory
import hrc_utils
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    herminebot_model: str = hrc_utils.get_herminebot_model()
    color_teams = ['blue', 'yellow']

    params_file = os.path.join(
        get_package_share_directory('herminebot_head'),
        'params',
        f'head_params_herminebot_{herminebot_model}.yaml'
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    color_sequences = [LaunchConfiguration(f'{color}_sequence', default='') for color in color_teams]

    # Create a temporary param file to include parameters
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            param_rewrites={
                'use_sim_time': use_sim_time,
                f'{color_teams[0]}_sequence_file': color_sequences[0],
                f'{color_teams[1]}_sequence_file': color_sequences[1]
            },
            convert_types=True,
        ),
        allow_substs=True,
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value=use_sim_time,
        description='Use simulation (Gazebo) clock if true'
    )

    declare_color_sequences = [
        DeclareLaunchArgument(
            f'{color}_sequence',
            default_value=color_sequences[index],
            description=f'Sequence file for {color} team'
        ) for index, color in enumerate(color_teams)
    ]

    node = Node(
        package='herminebot_head',
        executable='head_node',
        parameters=[configured_params]
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)
    for index in range(len(color_teams)):
        ld.add_action(declare_color_sequences[index])

    ld.add_action(node)
    return ld
