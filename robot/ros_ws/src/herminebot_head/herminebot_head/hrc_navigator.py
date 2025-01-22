import math

import builtin_interfaces.msg as bint_msgs
import geometry_msgs.msg as geo_msgs
import hrc_interfaces.action as hrc_action
import hrc_interfaces.srv as hrc_srv
import nav2_msgs.action as nav_action
import nav2_simple_commander.robot_navigator as nav2
import rclpy
from rclpy.action import ActionClient


class HRCNavigator(nav2.BasicNavigator):
    """Class to realize different actions."""

    def __init__(self, node_name='hrc_navigator', namespace=''):
        super().__init__(node_name, namespace)

        self.drive_on_heading_client = ActionClient(self, nav_action.DriveOnHeading, 'drive_on_heading')
        self.wait_client = ActionClient(self, nav_action.Wait, 'wait')
        self.preemption_client = ActionClient(self, hrc_action.Preempt, 'preemption_navigator')
        self.manage_map_objects_client = self.create_client(hrc_srv.ManageObjectsMap, 'manage_object_map')
        self.get_robot_pose_client = self.create_client(hrc_srv.GetRobotPose, 'get_robot_pose')
        self.start_pami_client = self.create_client(hrc_srv.StartPami, 'start_pami')
        self.color_team_client = self.create_client(hrc_srv.GetTeamColor, 'get_team_color')

    def destroy_node(self) -> None:
        self.drive_on_heading_client.destroy()
        self.wait_client.destroy()
        self.preemption_client.destroy()
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

        self.info(f'Driving to {goal_msg.target} at {drive_speed} m/s')
        send_goal_future = self.drive_on_heading_client.send_goal_async(goal_msg, self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('DriveOnHeading request was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def wait(self, duration: float = 0.0) -> bool:
        """
        Wait for duration seconds.

        :param duration: Time to wait
        :return: Whether the action has been accepted
        """
        self.debug("Waiting for 'Wait' action server")
        while not self.wait_client.wait_for_server(timeout_sec=1.0):
            self.info("'Wait' action server not available, waiting...")

        goal_msg = nav_action.Wait.Goal()
        goal_msg.time.sec = int(duration)
        goal_msg.time.nanosec = int(duration % 1 * 10 ** 9)

        self.info(f'Waiting for {duration:.1f} seconds')
        send_goal_future = self.wait_client.send_goal_async(goal_msg, self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Wait request was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def preempt(self, behavior_tree='') -> bool:
        """
        Preempt an object.

        :param behavior_tree: The behavior tree to use. If not specified, use the default.
        :return: Whether the action has been accepted
        """
        self.debug("Waiting for 'Preempt' action server")
        while not self.preemption_client.wait_for_server(timeout_sec=1.0):
            self.info("'Preempt' action server not available, waiting...")

        goal_msg = hrc_action.Preempt.Goal()
        goal_msg.behavior_tree = behavior_tree

        self.info('Starting preemption')

        send_goal_future = self.preemption_client.send_goal_async(goal_msg, self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Preempt request was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def get_robot_pose(self, source_frame='base_link', target_frame='map') -> list:
        """
        Get the robot position in the target frame.

        :param source_frame: The frame of the robot
        :param target_frame: The frame where we want the robot position
        :return: A list containing the robot position [x, y, yaw] if the position could be fetched else an empty list
        """
        self.debug("Waiting for 'get_robot_pose' server")
        while not self.get_robot_pose_client.wait_for_service(timeout_sec=1.0):
            self.info("'get_robot_pose' service not available, waiting...")

        req = hrc_srv.GetRobotPose.Request()
        req.robot_frame = source_frame
        req.base_frame = target_frame

        future = self.get_robot_pose_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        result: hrc_srv.GetRobotPose.Response = future.result()
        if result is None:
            self.error('Failed to get robot pose')
            return []
        return [result.robot_pose.x, result.robot_pose.y, result.robot_pose.theta]

    def manage_map_objects(self,
                           new_objects: list[list[dict]],
                           points_to_remove: list[dict],
                           is_robot_relative=False,
                           source_frame='base_link',
                           target_frame='map') -> None:
        """
        Modify the keepout mask filter of the map.

        :param new_objects: Objects to add
        :param points_to_remove: Points where all the polygons containing at least one point is removed
        :param is_robot_relative: Whether the coordinates are relative to the robot position
        :param source_frame: The source frame (only used if 'is_robot_relative' is 'True')
        :param target_frame: The target frame (only used if 'is_robot_relative' is 'True')
        :return: None
        """
        robot_pose = [0.0, 0.0, 0.0]
        if is_robot_relative:
            if pose := self.get_robot_pose(source_frame, target_frame):
                robot_pose = pose
            else:
                return

        def robot_to_map(px, py):
            """Calculate position in the target frame."""
            x = px * math.cos(robot_pose[2]) - py * math.sin(robot_pose[2]) + robot_pose[0]
            y = py * math.cos(robot_pose[2]) + px * math.sin(robot_pose[2]) + robot_pose[1]
            return x, y

        req = hrc_srv.ManageObjectsMap.Request()

        # Set new objects
        objects = []
        for obj in new_objects:
            poly = geo_msgs.Polygon()
            for point in obj:
                p = geo_msgs.Point32()
                p.x = point['x']
                p.y = point['y']
                if is_robot_relative:
                    p.x, p.y = robot_to_map(p.x, p.y)
                poly.points.append(p)
            objects.append(poly)
        req.new_objects = objects

        # Set points to remove
        points = []
        for point in points_to_remove:
            p = geo_msgs.Point32()
            p.x = point['x']
            p.y = point['y']
            if is_robot_relative:
                p.x, p.y = robot_to_map(p.x, p.y)
            points.append(p)
        req.points_objects_to_remove = points

        future = self.manage_map_objects_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

    def start_pami(self) -> None:
        """Start the PAMI."""
        req = hrc_srv.StartPami.Request()
        future = self.start_pami_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

    def get_team_color(self) -> str:
        """
        Get the team color from the service.

        :return: The team color
        """
        req = hrc_srv.GetTeamColor.Request()
        future = self.color_team_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if future.result():
            return future.result().team_color
        self.get_logger().error('Could not get team color')
        return ''
