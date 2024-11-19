import builtin_interfaces.msg as bint_msgs

import geometry_msgs.msg as geo_msgs

import nav2_msgs.action as nav_action
import nav2_simple_commander.robot_navigator as nav2

import rclpy
from rclpy.action import ActionClient


class HRCNavigator(nav2.BasicNavigator):
    """
    Class to realize different actions.
    """
    def __init__(self, node_name='hrc_navigator', namespace=''):
        super().__init__(node_name, namespace)

        self.drive_on_heading_client = ActionClient(self, nav_action.DriveOnHeading, "drive_on_heading")

    def destroy_node(self) -> None:
        self.drive_on_heading_client.destroy()
        super().destroy_node()

    def drive_on_heading(self, dist_to_travel: float = 0.15,
                         drive_speed: float = 0.025,
                         time_allowance: int = 10) -> bool:
        """
        Drive the robot on its current heading.
        To move backward, set dist_to_travel and drive_speed < 0
        :param dist_to_travel: Distance to be travelled
        :param drive_speed: Speed to travel the distance
        :param time_allowance: Time allowed to realize the action in seconds
        :return: Whether the action will be realized
        """
        self.debug("Waiting for 'DriveOnHeading' action server")
        while not self.drive_on_heading_client.wait_for_server(timeout_sec=1.0):
            self.info("'DriveOnHeading' action server not available, waiting...")

        goal_msg = nav_action.DriveOnHeading.Goal()
        goal_msg.target = geo_msgs.Point(x=dist_to_travel)
        goal_msg.speed = drive_speed
        goal_msg.time_allowance = bint_msgs.Duration(sec=time_allowance)

        self.info(f"Driving to {goal_msg.target} at {drive_speed} m/s")
        send_goal_future = self.drive_on_heading_client.send_goal_async(goal_msg, self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('DriveOnHeading request was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True
