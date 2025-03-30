#!/usr/bin/env python3

import argparse
import textwrap
from typing import Callable

import rclpy
from rclpy.node import Node
import nav_msgs.msg as nav_msgs
import geometry_msgs.msg as geo_msgs
import tf_transformations as tft
import numpy as np


def decorate_progress_bar(func: Callable) -> Callable:
    """
    Decorate a function to print a progress bar.
    For use in a method of CovarianceCalculator class only.

    :param func: function to decorate
    :return: decorated function
    """

    def progress_bar(self, msg):
        self.count += 1
        self.print_progress_bar(self.count, self.args.n, prefix='Progress:', suffix='Complete')
        func(self, msg)

    return progress_bar


class CovarianceCalculator(Node):
    """
    Calculate covariance matrix elements for the given target.
    Command line usage only!!!

    Examples:
        ./calculate_covariance.py triangulation
        ./calculate_covariance.py -h
    """

    def __init__(self):
        super().__init__('covariance_calculator')

        parser = argparse.ArgumentParser(
            description=textwrap.dedent(
                """
                Calculate covariance matrix elements for the given target.
                For calculating the covariance, some setup are required depending on the target.
                Available targets:
                - triangulation: Requires a robot running with a lidar and the 3 beacons well positioned. 
                The robot should be positioned in the worst position for the triangulation where he can be (eg. in a corner)
                Default topic: /triangulation
                - wheels_twist: Requires a robot that can move and to publish the velocity command for running indefinitely.
                Default topic: /wheels_twist
                Important: the robot must be running before launching the covariance calculator for not having false values.
                Example of command:
                    ros2 topic pub -t 1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.1}, angular: {z: 1.0}}"
                """),
            formatter_class=argparse.RawTextHelpFormatter)
        parser.add_argument('target', help='What to calculate the covariance for')
        parser.add_argument('-t', '--topic', help='Topic to listen to')
        parser.add_argument('-n', help='Number of samples to take', default=1000, type=int)
        self.args = parser.parse_args()

        match self.args.target.lower():
            case 'triangulation':
                self.create_subscription(
                    nav_msgs.Odometry, self.args.topic or 'triangulation', self.triangulation_callback, 10)
            case 'wheels_twist':
                self.create_subscription(geo_msgs.TwistWithCovarianceStamped, self.args.topic or 'wheels_twist',
                                         self.wheels_twist_callback, 10)
            case _:
                print('Unknown target')
                parser.print_help()
                exit(-1)

        self.x_vals = []
        self.y_vals = []
        self.yaw_vals = []
        self.count = 0
        self.print_progress_bar(0, self.args.n, prefix='Progress:', suffix='Complete')

    @decorate_progress_bar
    def triangulation_callback(self, msg: nav_msgs.Odometry) -> None:
        """
        Listen the odometry message and calculate the variance of x, y and yaw values of the triangulation

        :param msg: message containing the data
        :return: None
        """
        self.x_vals.append(msg.pose.pose.position.x)
        self.y_vals.append(msg.pose.pose.position.y)
        quat = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w]
        self.yaw_vals.append(tft.euler_from_quaternion(quat)[2])

        if self.count >= self.args.n:
            self.calculate_covariance({'x': self.x_vals, 'y': self.y_vals, 'yaw': self.yaw_vals})
            exit()

    @decorate_progress_bar
    def wheels_twist_callback(self, msg: geo_msgs.TwistWithCovarianceStamped) -> None:
        """
        Listen the odometry message and calculate the covariance matrix of x, y and yaw values of odometry.

        :param msg: message containing the data
        :return: None
        """
        self.x_vals.append(msg.twist.twist.linear.x)
        self.y_vals.append(msg.twist.twist.linear.y)
        self.yaw_vals.append(msg.twist.twist.angular.z)

        if self.count >= self.args.n:
            self.calculate_covariance({'vx': self.x_vals, 'vy': self.y_vals, 'vyaw': self.yaw_vals})
            exit()

    @staticmethod
    def calculate_covariance(vals: dict) -> None:
        """
        Calculate the covariance of the given values and print the result

        :param vals: dictionary of name and list of values to calculate the variance
        :return: None
        """
        variables = [v for v in vals.values()]
        print("Covariance matrix:")
        print(*vals.keys())
        print(np.cov(variables))

    @staticmethod
    def print_progress_bar(iteration: int, total: int, prefix='', suffix='', decimals=1, length=50, fill='#',
                           end_line="\r") -> None:
        """
        Create terminal progress bar

        :param iteration: current iteration
        :param total: total iterations
        :param prefix: prefix string
        :param suffix: suffix string
        :param decimals: positive number of decimals in percent complete
        :param length: character length of bar
        :param fill: bar fill character
        :param end_line: end character of the line
        :return: None
        """
        percent = ("{0:." + str(decimals) + "f}").format(100 * (iteration / total))
        filled_length = int(length * iteration // total)
        bar = fill * filled_length + '-' * (length - filled_length)
        print(f'{prefix} |{bar}| {percent}% {suffix}', end=end_line)
        # Print New Line on Complete
        if iteration >= total:
            print()


def main(args=None):
    rclpy.init(args=args)
    node = CovarianceCalculator()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
