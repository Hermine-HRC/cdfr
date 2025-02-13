#!/usr/bin/env python3

import geometry_msgs.msg as geo_msgs

import numpy as np

import rclpy
from rclpy.node import Node

import sensor_msgs.msg as sensor_msg
import std_msgs.msg as std_msg


class OmnibotCmdConverter(Node):
    """
    Convert a Twist message to a set of wheel velocities and convert joint states to Twist.

    Equations based on "Kinematics and Control A Three-wheeled Mobile Robot with Omni-directional Wheels"
    https://www.google.com/url?sa=t&source=web&rct=j&opi=89978449&url=https://public.thinkonweb.com/sites/iccr2023/media%3Fkey%3Dsite/iccr2023/abs/A-4-5.pdf&ved=2ahUKEwiBsvisq6yLAxUKRKQEHdZ5O6wQFnoECCYQAQ&usg=AOvVaw3tyASdDPipwb4O-KGO57SR
    """

    def __init__(self):
        """Initialize the node."""
        super().__init__('omnibot_cmd_converter')

        self.cmd_in_topic = ''
        self.cmd_out_topics = []
        self.wheels_twist_topic = ''
        self.joint_states_topic = ''
        self.wheel_joints = []
        self.wheels_velocity_cov_mat = np.array([])
        self.wheel_radius = 0.
        self.robot_radius = 0.

        self.init_parameters()

        self.cmd_sub = self.create_subscription(geo_msgs.Twist, self.cmd_in_topic, self.cmd_callback, 10)

        self.cmd_pubs = []
        for topic in self.cmd_out_topics:
            self.cmd_pubs.append(self.create_publisher(std_msg.Float64, topic, 10))

        self.wheels_twist_pub = self.create_publisher(geo_msgs.TwistWithCovarianceStamped, self.wheels_twist_topic, 10)
        self.joint_states_sub = self.create_subscription(sensor_msg.JointState, self.joint_states_topic,
                                                         self.joint_states_callback, 10)

        self.cmd_to_twist = np.array([
            [0, -np.sqrt(3) / 3, np.sqrt(3) / 3],
            [2 / 3, -1 / 3, -1 / 3],
            [1 / (3 * self.robot_radius), 1 / (3 * self.robot_radius), 1 / (3 * self.robot_radius)],
        ]) * self.wheel_radius

        self.twist_to_cmd = np.linalg.inv(self.cmd_to_twist)

        self.get_logger().info(f'Will receive cmd in {self.cmd_in_topic} and publish cmds in {self.cmd_out_topics}')

    def init_parameters(self):
        """
        Declare node parameters and assign them to some attributes.

        :return: None
        """
        self.cmd_in_topic = self.declare_parameter('cmd_in_topic', '/cmd_vel').get_parameter_value().string_value
        self.cmd_out_topics = self.declare_parameter(
            'cmd_out_topics',
            ['/wheel_cmd/front', '/wheel_cmd/left', '/wheel_cmd/right']).get_parameter_value().string_array_value

        assert len(self.cmd_out_topics) == 3, \
            'cmd_out_topics must have 3 elements'

        self.wheel_radius = self.declare_parameter('wheel_radius', 0.1).get_parameter_value().double_value
        self.robot_radius = self.declare_parameter('robot_radius', 0.5).get_parameter_value().double_value
        self.wheels_twist_topic = self.declare_parameter('wheels_twist_topic',
                                                         '/wheels_twist').get_parameter_value().string_value
        self.joint_states_topic = self.declare_parameter('joint_states_topic',
                                                         '/joint_states').get_parameter_value().string_value
        self.wheel_joints = self.declare_parameter(
            'wheel_joints',
            ['front_wheel_joint', 'left_wheel_joint', 'right_wheel_joint']).get_parameter_value().string_array_value

        assert len(self.wheel_joints) == 3, \
            'wheel_joints must have 3 elements'

        self.wheels_velocity_cov_mat = np.array(self.declare_parameter(
            'wheels_velocity_covariance_matrix', [0.] * 36).get_parameter_value().double_array_value)

        assert len(self.wheels_velocity_cov_mat) == 36, \
            'wheels_velocity_covariance_matrix must have 36 elements'

    def cmd_callback(self, msg: geo_msgs.Twist):
        """
        Calculate the velocities of each wheel and publish them.

        :param msg: Command twist message
        :return: None
        """
        velocities = np.matmul(self.twist_to_cmd, np.array([msg.linear.x, msg.linear.y, msg.angular.z]))
        for pub, vel in zip(self.cmd_pubs, velocities):
            pub.publish(std_msg.Float64(data=vel))

    def joint_states_callback(self, msg: sensor_msg.JointState) -> None:
        """
        Calculate the velocities of each wheel and publish them.

        :param msg: JointState message
        """
        idx = []
        for joint in self.wheel_joints:
            if joint not in msg.name:
                return
            idx.append(msg.name.index(joint))

        velocities = np.matmul(self.cmd_to_twist,
                               np.array([msg.velocity[idx[0]], msg.velocity[idx[1]], msg.velocity[idx[2]]]))
        tw = geo_msgs.TwistWithCovarianceStamped()
        tw.header.stamp = msg.header.stamp
        tw.header.frame_id = 'base_footprint'
        tw.twist.covariance = np.array(self.wheels_velocity_cov_mat)
        tw.twist.twist.linear.x = velocities[0]
        tw.twist.twist.linear.y = velocities[1]
        tw.twist.twist.angular.z = velocities[2]
        self.wheels_twist_pub.publish(tw)


def main(args=None):
    rclpy.init(args=args)
    node = OmnibotCmdConverter()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
