#!/usr/bin/env python3

import math
import os

from ament_index_python.packages import get_package_share_directory
import geometry_msgs.msg as geo_msgs

SERVICE_TIMEOUT = 1.0


def robot_to_map(robot_pose: geo_msgs.Pose2D, robot_point: geo_msgs.Point32) -> geo_msgs.Point32:
    """
    Convert a point from the robot coordinates to the map coordinates.

    :param robot_pose: Pose of the robot
    :param robot_point: Point in the robot coordinates
    :return: Point in the map coordinates
    """
    map_point = geo_msgs.Point32()
    map_point.x = robot_point.x * math.cos(robot_pose.theta) - robot_point.y * math.sin(robot_pose.theta) + robot_pose.x
    map_point.y = robot_point.x * math.sin(robot_pose.theta) + robot_point.y * math.cos(robot_pose.theta) + robot_pose.y
    return map_point


def get_herminebot_model() -> str:
    """
    Return the current model of the herminebot. The default value is 'diff'.

    :return: The herminebot model
    """
    return os.environ.get('HERMINEBOT_MODEL', 'diff')


def get_venv_site_packages_dir() -> str:
    """
    Return the python virtual environment site-packages directory absolute path.

    :return:  The path
    """
    return (f"{'/'.join(get_package_share_directory('herminebot_bringup').split('/')[:-4])}"
            f'/.venv/lib/python3.12/site-packages')
