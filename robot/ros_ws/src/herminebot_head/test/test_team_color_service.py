from herminebot_head import ColorTeamService
import hrc_interfaces.srv as hrc_srvs
import pytest
import rclpy


def test_team_color_service():
    rclpy.init()
    try:
        node = ColorTeamService('blue')
        req = hrc_srvs.GetTeamColor.Request()
        res = node.get_team_color_cb(req, hrc_srvs.GetTeamColor.Response())
        assert res.team_color == 'blue'

    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    pytest.main(['-v'])
