import json
import os

import std_msgs.msg

import herminebot_head

from launch_ros.substitutions import FindPackageShare
import rclpy
from rclpy.node import Node
import tf_transformations as tft

import geometry_msgs.msg as geo_msgs
import rcl_interfaces.msg as rcl_msgs

import hrc_interfaces.srv as hrc_srv

import nav2_simple_commander.robot_navigator as nav2


class HeadNode(Node):
    """
    Head node for managing the actions the herminebot has to realize.
    This node reads a sequence json file and manage to realize the actions until the end of the time.
    The node is called by ROS and depend on the nav2 stack functioning.
    The actions will start once a 'True' message will be received on the topic defined by 'start_actions_topic'.
    The node is able to restart. This can be done by publishing a 'True' value on the topic defined by 'restart_topic'.
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
        self.team_colors = list()
        self.stop_time = float()
        self.timeout_time = -1.0
        self.time_to_enable_laser_sensors = 0.0

        self.undone_actions_ids = list()
        self.start_action_time = self.start_time = 0.0

        self.is_first_manager_call = True
        self.is_going_to_end_pos = False
        self.wait_action_end_time = -1.0
        self.navigator = herminebot_head.HRCNavigator("hrc_navigator")
        self.score = 0

        self.actions_manager_timer: rclpy.executors.Timer = None

        self.init_parameters()

        # Subscriptions
        self.begin_actions_sub = self.create_subscription(
            std_msgs.msg.Bool,
            self.get_parameter("start_actions_topic").get_parameter_value().string_value,
            self.begin_actions_callback,
            1
        )
        self.restart_actions_sub = self.create_subscription(
            std_msgs.msg.Bool,
            self.get_parameter("restart_topic").get_parameter_value().string_value,
            self.restart,
            1
        )

        # Services
        self.color_team_client = self.create_client(hrc_srv.GetTeamColor, "get_team_color")
        while not self.color_team_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")

        self.navigator.waitUntilNav2Active()
        self.controller_server = herminebot_head.ExternalParamInterface("controller_server")
        self.collision_monitor_server = herminebot_head.ExternalParamInterface("collision_monitor")

        self.enable_laser_sensors(self.time_to_enable_laser_sensors == 0.0)

        self.load_external_parameters()

        self.init_sequence()

        # Setup
        self.execute_setup()

    def actions_manager_callback(self):
        """
        Callback for managing the sequence actions
        :return: None
        """
        time_nano_sec = self.get_clock().now().nanoseconds - self.start_time * 1e9
        now_time = time_nano_sec * 1e-9

        if self.is_first_manager_call:
            self.is_first_manager_call = False
            self.start_time = now_time
            now_time = 0.0
            self.start_action_time = 0.0

        if now_time > self.time_to_enable_laser_sensors > 0.0:
            self.enable_laser_sensors()
            self.time_to_enable_laser_sensors = -1.0

        if not self.should_start_action(now_time):
            return

        if len(self.actions) <= self.action_idx:
            self.get_logger().info("No more actions left. Going to end position")
            self.set_goal_params(self.setup.get("end_pose_tolerance", dict()))
            self.goto(**self.setup["end_pose"])
            self.is_going_to_end_pos = True
            self.action_idx += 1
            return

        self.realize_action(self.actions[self.action_idx], now_time)
        self.action_idx += 1

    def set_goal_params(self, action: dict) -> None:
        """
        Set the common goal params to the controller server
        :param action: Action that contains the params values
        :return: None
        """
        self.controller_server.set_params({
            "FollowPath.desired_linear_vel": action.get("desired_speed", self.desired_speed),
            "goal_checker.xy_goal_tolerance": action.get("xy_tolerance", self.xy_tolerance),
            "goal_checker.yaw_goal_tolerance": action.get("yaw_tolerance", self.yaw_tolerance)
        })

    def realize_action(self, action: dict, now_time: float) -> None:
        """
        Realize the input action
        :param action: Action to be realized
        :param now_time: Current time
        :return: None
        """
        if any(depend_id in self.undone_actions_ids for depend_id in action.get("depend", list())):
            self.get_logger().info(f"Ignoring action {action['id']} because the action depend on an undone action")
            self.undone_actions_ids.append(action["id"])
            return

        if action.get("skip", False):
            self.get_logger().warn(f"Skipping action {self.action_idx}")
            self.undone_actions_ids.append(action["id"])
            return

        if comment := action.get("comment", None):
            self.get_logger().info(f"Action {action['id']} comment: {comment}")

        if (timeout := action.get("timeout", -1.0)) > 0.0:
            self.timeout_time = timeout + now_time
        else:
            self.timeout_time = -1.0

        action_type = action.get("type", "no_type_given")
        match action_type:
            case "goto":
                self.set_goal_params(action)
                self.goto(**action["pose"])

            case "gothrough":
                self.set_goal_params(action)
                self.gothrough(action["poses"])

            case "preempt":
                self.navigator.preempt()

            case "set_pose":
                self.set_pose(**action["pose"])

            case "wait_for":
                self.navigator.wait(action.get("duration", 0.0))

            case "wait_until":
                wait_time = action.get("time", 0.0)
                self.get_logger().info(f"Waiting until {wait_time:.1f} seconds")
                self.navigator.wait(wait_time - now_time)

            case "spin":
                self.navigator.spin(action.get("angle", 0.0))

            case "drive":
                self.navigator.drive_on_heading(action.get("distance", 0.0), action.get("speed", 0.025))

            case _:
                self.get_logger().error(f"Unexpected action type '{action_type}'. Ignoring action {action['id']}")

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
            self.get_logger().info(f"Total score is {self.score}")
            return False

        elif now_time >= self.go_to_end_pose_time and not self.is_going_to_end_pos:
            self.get_logger().info("Go to final position time reached. Stopping action")
            self.navigator.cancelTask()
            self.is_going_to_end_pos = True
            self.action_idx = len(self.actions)
            self.timeout_time = -1.0
            return False

        elif now_time >= self.timeout_time > 0.0:
            self.get_logger().info("Timeout reached. Cancelling action")
            self.navigator.cancelTask()
            self.timeout_time = -1.0
            return False

        elif not self.navigator.isTaskComplete():
            return False

        elif self.navigator.getResult() == nav2.TaskResult.SUCCEEDED and self.action_idx > 0:
            self.get_logger().info(f"Action {self.action_idx - 1} took {now_time - self.start_action_time:.1f} seconds")
            if self.action_idx - 1 < len(self.actions):
                self.score += self.actions[self.action_idx - 1].get("score", 0)

            if self.is_going_to_end_pos:
                self.score += self.setup.get("end_pose_reached_score", 0)
                self.get_logger().info("All actions realized. Stopping action manager")
                self.get_logger().info(f"All actions took {now_time:.1f} seconds")
                self.get_logger().info(f"Total score is {self.score}")
                self.actions_manager_timer.destroy()
                return False

        elif (self.navigator.getResult() != nav2.TaskResult.UNKNOWN
              and self.action_idx > 0 and not self.is_going_to_end_pos):
            self.get_logger().warn(f"Action {self.actions[self.action_idx - 1]['id']} failed")
            self.undone_actions_ids.append(self.actions[self.action_idx - 1]['id'])

        self.navigator.clearGlobalCostmap()
        self.start_action_time = now_time

        return True

    def begin_actions_callback(self, msg: std_msgs.msg.Bool) -> None:
        """
        Callback method to activate or deactivate the actions_manager_timer
        :param msg: Whether the actions are enabled
        :return: None
        """
        if msg.data and not self.actions_manager_timer:
            self.get_logger().info("Starting actions")
            self.actions_manager_timer = self.create_timer(
                self.get_parameter("action_manager_period").get_parameter_value().double_value,
                self.actions_manager_callback
            )

        elif not msg.data and self.actions_manager_timer:
            self.get_logger().info("Stop asked by starting topic")
            self.navigator.cancelTask()
            self.action_idx -= 1
            self.actions_manager_timer.destroy()
            self.actions_manager_timer: rclpy.executors.Timer = None

    def get_team_color(self) -> str:
        """
        Get the team color from the service
        :return: The team color
        """
        req = hrc_srv.GetTeamColor.Request()
        future = self.color_team_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if future.result():
            return future.result().team_color
        self.get_logger().error("Could not get team color")
        return ""

    def restart(self, msg: std_msgs.msg.Bool | None = None) -> None:
        """
        Restart the node
        :param msg: Only exist to use the method in a subscriber callback. If message data is False: do nothing
        :return: None
        """
        if msg and not msg.data:
            return

        self.get_logger().info("Restarting head node")

        self.navigator.cancelTask()
        if self.actions_manager_timer:
            self.actions_manager_timer.destroy()
            self.actions_manager_timer: rclpy.executors.Timer = None
        self.action_idx = 0
        self.start_time = 0.0
        self.score = 0
        self.is_first_manager_call = True
        self.is_going_to_end_pos = False
        self.wait_action_end_time = -1.0

        while not self.color_team_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")

        self.enable_laser_sensors(self.time_to_enable_laser_sensors == 0.0)
        self.init_sequence()
        self.execute_setup()

    def init_sequence(self) -> None:
        """
        Load the json sequence file depending on the team
        :return: None
        """
        if (team_color := self.get_team_color()) in self.team_colors:
            seq_file = self.get_parameter(f"{team_color}_sequence_file").get_parameter_value().string_value
        else:
            pkg = FindPackageShare(package="herminebot_head").find("herminebot_head")
            seq_file = os.path.join(pkg, "sequences",
                                    self.get_parameter("sequence_default_filename").get_parameter_value().string_value)

        self.get_logger().info(f"Loading sequence {seq_file}")
        with open(seq_file, 'r') as file:
            data = json.load(file)

        self.actions: list = data["actions"]
        self.setup: dict = data["setup"]
        self.go_to_end_pose_time = self.setup["go_to_end_pose_time"]
        self.action_idx = 0

    def execute_setup(self) -> None:
        """
        Execute all that can be executed at setup
        :return: None
        """
        self.set_pose(**self.setup["initial_pose"])
        for action in self.setup.get("actions", list()):
            if action.get("type", "") == "wait_until":
                self.get_logger().error("Unauthorized action 'wait_until' for setup. Ignoring action")
                continue

            self.realize_action(action, 0.0)

            while not self.navigator.isTaskComplete():
                pass

        self.get_logger().info("Setup actions done")
        self.navigator.clearAllCostmaps()

    def init_parameters(self) -> None:
        """
        Declare node parameters and assign them to some attributes
        :return: None
        """
        self.declare_parameter("action_manager_period", 0.5)
        self.declare_parameter("sequence_default_filename", "demo_seq.json")
        self.team_colors = self.declare_parameter(
            "team_colors", rclpy.Parameter.Type.STRING_ARRAY
        ).get_parameter_value().string_array_value
        for color in self.team_colors:
            self.declare_parameter(f"{color}_sequence_file", "")
        self.declare_parameter("start_actions_topic", "/can_start_actions")
        self.declare_parameter("restart_topic", "/restart")
        self.global_frame = self.declare_parameter("global_frame_id", "map").get_parameter_value().string_value
        self.stop_time = self.declare_parameter("stop_time", 99.0).get_parameter_value().double_value
        self.time_to_enable_laser_sensors = self.declare_parameter("time_to_enable_laser_sensors",
                                                            0.0).get_parameter_value().double_value

        self.add_on_set_parameters_callback(self.dynamic_parameters_callback)

    def enable_laser_sensors(self, enable: bool = True) -> None:
        """
        Enable the laser sensors
        :param enable: Whether to enable the sensors
        :return: None
        """
        self.get_logger().info(f"{'En' if enable else 'Dis'}abling laser sensors for collision")
        self.collision_monitor_server.set_params({
            "laser_sensor_1.enabled": enable,
            "laser_sensor_2.enabled": enable,
            "laser_sensor_3.enabled": enable,
            "laser_sensor_4.enabled": enable
        })

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
