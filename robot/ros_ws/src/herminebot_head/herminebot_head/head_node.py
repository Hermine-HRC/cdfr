import json
import os

from launch_ros.substitutions import FindPackageShare
import rclpy
from rclpy.node import Node
import tf_transformations as tft

import geometry_msgs.msg as geo_msgs
import rcl_interfaces.msg as rcl_msgs

import nav2_simple_commander.robot_navigator as nav2

PKG = FindPackageShare(package="herminebot_head").find("herminebot_head")


class HeadNode(Node):
    def __init__(self):
        super().__init__("head_node")

        self.actions = list()
        self.action_idx = 0
        self.setup = dict()
        self.global_frame_param: rcl_msgs.Parameter = None
        self.is_going_to_end_pos = False

        self.init_parameters()

        self.navigator = nav2.BasicNavigator("basic_navigator")
        self.init_sequence(self.get_parameter("sequence_default_filename").value)
        self.navigator.waitUntilNav2Active()

        self.set_pose(**self.setup["initial_pose"])

        self.actions_manager_timer = self.create_timer(
            self.get_parameter("action_manager_period").value, self.actions_manager_callback
        )
        self.start_action_time = self.get_clock().now().seconds_nanoseconds()[0]

    def actions_manager_callback(self):
        """
        Callback for managing the sequence actions
        :return: None
        """
        if not self.navigator.isTaskComplete():
            return

        if self.navigator.getResult() == nav2.TaskResult.SUCCEEDED:
            now_time = self.get_clock().now().seconds_nanoseconds()[0]
            self.get_logger().info(f"Action {self.action_idx - 1} took {now_time - self.start_action_time} seconds")
            self.start_action_time = now_time

            if self.is_going_to_end_pos:
                self.get_logger().info("All actions realized. Stopping action manager")
                self.actions_manager_timer.destroy()
                return

        if len(self.actions) <= self.action_idx:
            self.get_logger().info("No more actions left. Going to end position")
            self.goto(**self.setup["end_pose"])
            self.is_going_to_end_pos = True
            self.action_idx += 1
            return

        action = self.actions[self.action_idx]
        if comment := action.get("comment", None):
            self.get_logger().info(f"Action {self.action_idx} comment: {comment}")

        if not action.get("skip", False):

            action_type = action.get("type", "no_type_given")
            match action_type:
                case "goto":
                    self.goto(**action["pose"])
                case "gothrough":
                    self.gothrough(action["poses"])
                case "set_pose":
                    self.set_pose(**action["pose"])
                case _:
                    self.get_logger().error(
                        f"Unexpected action type '{action_type}'. Ignoring action {self.action_idx}")
        else:
            self.get_logger().warn(f"Skipping action {self.action_idx}")

        self.action_idx += 1

    def init_sequence(self, filename: str) -> None:
        """
        Load the json sequence file
        :param filename: Sequence filename
        :return: None
        """
        with open(os.path.join(PKG, "sequences", filename), 'r') as file:
            data = json.load(file)

        self.actions: list = data["actions"]
        self.setup: dict = data["setup"]
        self.action_idx = 0

    def init_parameters(self) -> None:
        """
        Declare node parameters and assign them to  some attributes
        :return: None
        """
        self.declare_parameter("action_manager_period", 0.5)
        self.declare_parameter("sequence_default_filename", "demo_seq.json")
        self.global_frame_param = self.declare_parameter("global_frame_id", "map")

        self.add_on_set_parameters_callback(self.dynamic_parameters_callback)

    def dynamic_parameters_callback(self, params: list[rclpy.Parameter]) -> rcl_msgs.SetParametersResult:
        """
        Callback function for updating parameters at runtime
        :param params: List of parameters to modify
        :return: Whether the modification was successful
        """
        self.get_logger().info(f"Setting internal params: {[param.name for param in params]}")
        res = True
        for param in params:
            match param.type_:
                case _:
                    res = False
                    self.get_logger().warn(f"Unsupported modification for params of type {param.type_}")

        return rcl_msgs.SetParametersResult(successful=res)

    def set_pose(self, x, y, yaw) -> None:
        """
        Set current position of the robot
        :param x: Position along x-axis in meters
        :param y: Position along y-axis in meters
        :param yaw: Rotation around z-axis in radians
        :return: None
        """
        self.get_logger().info(f"Set initial position to x={x}, y={y}, yaw={yaw}")
        self.navigator.setInitialPose(self.get_pose_stamped(x, y, yaw))
        self.navigator.clearAllCostmaps()

    def goto(self, x, y, yaw) -> bool:
        """
        Set the goal position of the robot
        :param x: Position along x-axis in meters
        :param y: Position along y-axis in meters
        :param yaw: Rotation around z-axis in radians
        :return: Whether the goal has been accepted
        """
        self.get_logger().info(f"Set goal position to x={x}, y={y}, yaw={yaw}")
        return self.navigator.goToPose(self.get_pose_stamped(x, y, yaw))

    def gothrough(self, poses: list[dict]) -> bool:
        """
        Set the positions the robot will have to go through in the order
        :param poses: List of x, y and yaw
        :return: Whether the goals have been accepted
        """
        self.get_logger().info(f"Going through {len(poses)} poses. "
                               f"Last position: x={poses[-1].get('x', 0.0)}, y={poses[-1].get('y', 0.0)}, "
                               f"yaw={poses[-1].get('yaw', 0.0)}")

        goal_poses = [self.get_pose_stamped(pos.get("x", 0.0), pos.get("y", 0.0), pos.get("yaw", 0.0)) for pos in poses]
        return self.navigator.goThroughPoses(goal_poses)

    def get_pose_stamped(self, x, y, yaw) -> geo_msgs.PoseStamped:
        """
        Transform position from x, y, and yaw to geometry_msgs.msg.PoseStamped
        :param x: Position along x-axis in meters
        :param y: Position along y-axis in meters
        :param yaw: Rotation around z-axis in radians
        :return: The corresponding PoseStamped
        """
        pose = geo_msgs.PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.global_frame_param.value
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0

        qvals = {k: v for k, v in zip("xyzw", tft.quaternion_from_euler(0.0, 0.0, float(yaw)))}
        pose.pose.orientation = geo_msgs.Quaternion(**qvals)

        return pose


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HeadNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
