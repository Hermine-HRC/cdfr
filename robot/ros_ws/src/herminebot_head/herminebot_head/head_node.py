import json
import os

import herminebot_head

from launch_ros.substitutions import FindPackageShare
import rclpy
from rclpy.node import Node
import tf_transformations as tft

import geometry_msgs.msg as geo_msgs
import rcl_interfaces.msg as rcl_msgs

import nav2_simple_commander.robot_navigator as nav2

PKG = FindPackageShare(package="herminebot_head").find("herminebot_head")


class HeadNode(Node):
    """
    Head node for managing the actions the herminebot has to realize.
    This node reads a sequence json file and manage to realize the actions until the end of the time.
    The node is called by ROS and depend on the nav2 stack functioning.
    """

    def __init__(self):
        super().__init__("head_node")

        # Create attributes
        self.actions = list()
        self.action_idx = 0
        self.setup = dict()
        self.go_to_end_pose_time = 0.0

        self.desired_speed = 0.0
        self.xy_tolerance = 0.0
        self.yaw_tolerance = 0.0

        self.global_frame = str()
        self.stop_time = float()

        self.start_action_time = self.start_time = 0.0

        self.is_first_manager_call = True
        self.is_going_to_end_pos = False
        self.wait_action_end_time = -1.0
        self.navigator = nav2.BasicNavigator("basic_navigator")

        # Init attributes
        self.init_parameters()
        self.init_sequence(self.get_parameter("sequence_default_filename").get_parameter_value().string_value)
        self.navigator.waitUntilNav2Active()
        self.controller_server = herminebot_head.ExternalParamInterface("controller_server")
        self.load_external_parameters()

        # Setup
        self.set_pose(**self.setup["initial_pose"])

        # Start actions
        self.actions_manager_timer = self.create_timer(
            self.get_parameter("action_manager_period").get_parameter_value().double_value,
            self.actions_manager_callback
        )

    def actions_manager_callback(self):
        """
        Callback for managing the sequence actions
        :return: None
        """
        time_sec, time_nano_sec = self.get_clock().now().seconds_nanoseconds()
        now_time = float(f"{time_sec}.{time_nano_sec}") - self.start_time
        if self.is_first_manager_call:
            self.is_first_manager_call = False
            self.start_action_time = self.start_time = now_time

        if not self.should_start_action(now_time):
            return

        def set_goal_params(action_: dict) -> None:
            self.controller_server.set_params({
                "FollowPath.desired_linear_vel": action_.get("desired_speed", self.desired_speed),
                "goal_checker.xy_goal_tolerance": action_.get("xy_tolerance", self.xy_tolerance),
                "goal_checker.yaw_goal_tolerance": action_.get("yaw_tolerance", self.yaw_tolerance)
            })

        if len(self.actions) <= self.action_idx:
            self.get_logger().info("No more actions left. Going to end position")
            set_goal_params(self.setup.get("end_pose_tolerance", dict()))
            self.goto(**self.setup["end_pose"])
            self.is_going_to_end_pos = True
            self.action_idx += 1
            return

        action: dict = self.actions[self.action_idx]
        if comment := action.get("comment", None):
            self.get_logger().info(f"Action {self.action_idx} comment: {comment}")

        if action.get("skip", False):
            self.get_logger().warn(f"Skipping action {self.action_idx}")
        else:
            action_type = action.get("type", "no_type_given")
            match action_type:
                case "goto":
                    set_goal_params(action)
                    self.goto(**action["pose"])

                case "gothrough":
                    set_goal_params(action)
                    self.gothrough(action["poses"])

                case "set_pose":
                    self.set_pose(**action["pose"])

                case "wait_for":
                    wait_duration = action.get("duration", 0.0)
                    self.get_logger().info(f"Waiting for {wait_duration} seconds")
                    self.wait_action_end_time = wait_duration + now_time

                case "wait_until":
                    wait_time = action.get("time", 0.0)
                    self.get_logger().info(f"Waiting until {wait_time} seconds")
                    self.wait_action_end_time = wait_time

                case _:
                    self.get_logger().error(
                        f"Unexpected action type '{action_type}'. Ignoring action {self.action_idx}")

        self.action_idx += 1

    def should_start_action(self, now_time: float) -> bool:
        """
        Whether a new action should be started
        :param now_time: The current time
        :return: True if the current state allow to start a new action
        """
        if now_time >= self.stop_time:
            self.get_logger().info("Stop time reached. Stopping action manager")
            self.navigator.cancelTask()
            self.actions_manager_timer.destroy()
            return False

        elif now_time >= self.go_to_end_pose_time and not self.is_going_to_end_pos:
            self.get_logger().info("Go to final position time reached. "
                                   f"Stopping current action: action {self.action_idx}")
            self.navigator.cancelTask()
            self.is_going_to_end_pos = True
            self.action_idx = len(self.actions)
            return False

        elif not self.are_actions_completed(now_time):
            return False

        elif self.navigator.getResult() == nav2.TaskResult.SUCCEEDED:
            self.get_logger().info(f"Action {self.action_idx - 1} took {now_time - self.start_action_time:.1f} seconds")
            self.start_action_time = now_time

            if self.is_going_to_end_pos:
                self.get_logger().info("All actions realized. Stopping action manager")
                self.get_logger().info(f"All actions took {now_time:.1f} seconds")
                self.actions_manager_timer.destroy()
                return False

        elif self.navigator.getResult() != nav2.TaskResult.UNKNOWN and not self.is_going_to_end_pos:
            self.get_logger().warn(f"Action {self.action_idx} failed")
            self.start_action_time = now_time

        return True

    def are_actions_completed(self, current_time_sec: float) -> bool:
        """
        Verify whether the actions are completed
        :param current_time_sec: Current time
        :return: 'False' if a task is still running else 'True'
        """
        return (
                self.navigator.isTaskComplete() and
                current_time_sec > self.wait_action_end_time
        )

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
        self.go_to_end_pose_time = self.setup["go_to_end_pose_time"]
        self.action_idx = 0

    def init_parameters(self) -> None:
        """
        Declare node parameters and assign them to some attributes
        :return: None
        """
        self.declare_parameter("action_manager_period", 0.5)
        self.declare_parameter("sequence_default_filename", "demo_seq.json")
        self.global_frame = self.declare_parameter("global_frame_id", "map").get_parameter_value().string_value
        self.stop_time = self.declare_parameter("stop_time", 99.0).get_parameter_value().double_value

        self.add_on_set_parameters_callback(self.dynamic_parameters_callback)

    def load_external_parameters(self) -> None:
        """
        Load the external parameters
        :return: None
        """
        controller_server_params = self.controller_server.get_params([
            "FollowPath.desired_linear_vel",
            "goal_checker.xy_goal_tolerance",
            "goal_checker.yaw_goal_tolerance"
        ])

        if controller_server_params is None:
            self.get_logger().fatal("Could not load 'controller server' params")
            raise Exception("Could not load 'controller server' params")

        self.desired_speed: float = controller_server_params.values[0].double_value
        self.xy_tolerance: float = controller_server_params.values[1].double_value
        self.yaw_tolerance: float = controller_server_params.values[2].double_value

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
                case rclpy.Parameter.Type.DOUBLE:
                    if param.name == "stop_time":
                        self.stop_time = param.get_parameter_value().double_value
                    else:
                        res = False
                        self.get_logger().warn(f"Unsupported runtime modification for param '{param.name}'")
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
        pose.header.frame_id = self.global_frame
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
