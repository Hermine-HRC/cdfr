from hrc_interfaces.srv import GetTeamColor

import rclpy
from rclpy.node import Node


class ColorTeamService(Node):
    """Give the color of the team set by 'team_color' parameter."""

    def __init__(self, team_color: str = ''):
        super().__init__('color_team_service')
        self.srv = self.create_service(GetTeamColor, 'get_team_color', self.get_team_color_cb)

        self.team_color = self.declare_parameter('team_color', team_color).get_parameter_value().string_value

    def get_team_color_cb(self, _, response):
        """Give the color of the team."""
        response.team_color = self.team_color
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    color_team_service = ColorTeamService()
    rclpy.spin(color_team_service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
