import math
import os

import geometry_msgs.msg as geo_msgs
import hrc_utils
import pytest


def test_robot_to_map():
    """Test the robotToMap function."""
    # Check robot at the origin
    robot_pose = geo_msgs.Pose2D(x=0.0, y=0.0, theta=0.0)
    robot_point = geo_msgs.Point32(x=1.0, y=1.0)
    map_point = hrc_utils.robot_to_map(robot_pose, robot_point)

    assert map_point.x == pytest.approx(1.0)
    assert map_point.y == pytest.approx(1.0)

    # Check robot still at the origin but rotated 90 degrees
    robot_pose.theta = math.pi / 2
    map_point = hrc_utils.robot_to_map(robot_pose, robot_point)

    assert map_point.x == pytest.approx(-1.0)
    assert map_point.y == pytest.approx(1.0)

    # Check robot not at the origin and still at 90 degrees
    robot_pose.x = 0.5
    robot_pose.y = 0.5
    map_point = hrc_utils.robot_to_map(robot_pose, robot_point)

    assert map_point.x == pytest.approx(-0.5)
    assert map_point.y == pytest.approx(1.5)


def test_get_herminebot_model():
    """Test the get_herminebot_model function."""
    assert hrc_utils.get_herminebot_model() == 'diff'

    os.environ['HERMINEBOT_MODEL'] = 'omni'
    assert hrc_utils.get_herminebot_model() == 'omni'


if __name__ == '__main__':
    pytest.main(['-v'])
