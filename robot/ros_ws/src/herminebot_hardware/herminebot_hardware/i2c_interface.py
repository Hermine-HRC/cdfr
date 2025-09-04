from collections.abc import Callable
import math
from typing import Any

import board
import busio
import geometry_msgs.msg as geo_msgs
import icm20948  # IMU
import nav_msgs.msg as nav_msgs_
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
import sensor_msgs.msg as sensor_msgs_


class I2CInterface(Node):
    """
    This node is used to interface with the I2C bus to read and write values to devices using the I2C protocol.

    This node use I2C as master.
    """

    def __init__(self):
        super().__init__('i2c_interface')

        self.wheel_radius = 0.
        self.wheel_separation = 0.
        self.coder_addresses = []
        self.coder_subscription_rate = 0.
        self.odometry_topic = ''
        self.odometry_frame_id = ''
        self.odometry_child_frame_id = ''
        self.odometry_covariance_matrix = []
        self.cmd_vel_topic = ''
        self.coder_message_ratio = 0.
        self.i2c_frequency = 0
        self.imu_address = 0
        self.imu_topic = ''
        self.imu_rate = 0.
        self.imu_frame_id = ''
        self.imu_acceleration_covariance_matrix = []
        self.imu_orientation_covariance_matrix = []

        self.init_parameters()

        self.i2c = busio.I2C(board.SCL, board.SDA, frequency=self.i2c_frequency)
        self.imu = icm20948.ICM20948(self.imu_address)

        self.coder_timer = self.create_timer(self.coder_subscription_rate, self.coder_callback)
        self.odom_pub = self.create_publisher(nav_msgs_.Odometry, self.odometry_topic, 10)
        self.cmd_vel_sub = self.create_subscription(geo_msgs.Twist, self.cmd_vel_topic, self.cmd_vel_callback, 10)
        self.imu_pub = self.create_publisher(sensor_msgs_.Imu, self.imu_topic, 10)
        self.imu_timer = self.create_timer(self.imu_rate, self.imu_callback)

        self.get_logger().info('I2C interface ready')

    def init_parameters(self):
        """Declare node parameters and assign them to some attributes."""
        self.wheel_radius = self.declare_parameter('wheel_radius',
                                                   rclpy.Parameter.Type.DOUBLE).get_parameter_value().double_value
        self.wheel_separation = self.declare_parameter('wheel_separation',
                                                       rclpy.Parameter.Type.DOUBLE).get_parameter_value().double_value
        self.coder_addresses = self.declare_parameter(
            'coder_addresses',
            rclpy.Parameter.Type.INTEGER_ARRAY).get_parameter_value().integer_array_value.tolist()
        assert len(self.coder_addresses) == 2, f'Two coder addresses are needed, {len(self.coder_addresses)} given'

        self.odometry_covariance_matrix = self.declare_parameter(
            'odometry_covariance_matrix',
            rclpy.Parameter.Type.DOUBLE_ARRAY).get_parameter_value().double_array_value.tolist()
        assert len(self.odometry_covariance_matrix) == 36, \
            f'36 covariance values are needed, {len(self.odometry_covariance_matrix)} given'

        self.coder_subscription_rate = self.declare_parameter('coder_subscription_rate',
                                                              1e3).get_parameter_value().double_value
        self.odometry_topic = self.declare_parameter('odometry_topic',
                                                     '/wheel/odometry').get_parameter_value().string_value
        self.odometry_frame_id = self.declare_parameter('odometry_frame_id', 'odom').get_parameter_value().string_value
        self.odometry_child_frame_id = self.declare_parameter('odometry_child_frame_id',
                                                              'base_footprint').get_parameter_value().string_value
        self.cmd_vel_topic = self.declare_parameter('cmd_vel_topic', '/cmd_vel').get_parameter_value().string_value
        self.coder_message_ratio = self.declare_parameter('coder_message_ratio',
                                                          10.0).get_parameter_value().double_value

        self.i2c_frequency = self.declare_parameter('i2c_frequency', 100000).get_parameter_value().integer_value
        assert self.i2c_frequency in [100000, 400000, 1000000], \
            f'Invalid I2C frequency {self.i2c_frequency}, expected are 100kHz, 400kHz or 1MHz'

        self.imu_address = self.declare_parameter('imu_address', icm20948.I2C_ADDR).get_parameter_value().integer_value
        self.imu_topic = self.declare_parameter('imu_topic', '/imu/data').get_parameter_value().string_value
        self.imu_rate = self.declare_parameter('imu_rate', 1e3).get_parameter_value().double_value
        self.imu_frame_id = self.declare_parameter('imu_frame_id', 'imu_link').get_parameter_value().string_value

        self.imu_acceleration_covariance_matrix = self.declare_parameter(
            'imu_acceleration_covariance_matrix',
            rclpy.Parameter.Type.DOUBLE_ARRAY).get_parameter_value().double_array_value.tolist()
        assert len(self.imu_acceleration_covariance_matrix) == 9, \
            f'9 covariance values are needed, {len(self.imu_acceleration_covariance_matrix)} given'

        self.imu_orientation_covariance_matrix = self.declare_parameter(
            'imu_orientation_covariance_matrix',
            rclpy.Parameter.Type.DOUBLE_ARRAY).get_parameter_value().double_array_value.tolist()
        assert len(self.imu_orientation_covariance_matrix) == 9, \
            f'9 covariance values are needed, {len(self.imu_orientation_covariance_matrix)} given'

    def use_i2c(self, func: Callable[[], Any]) -> Any:
        """
        Use the I2C bus to execute a function.

        :param func: The function to execute using the I2C bus
        :return: Anything that is returned by the function
        """
        while not self.i2c.try_lock():
            self.get_clock().sleep_for(Duration(seconds=1e-3))

        ans = func()

        self.i2c.unlock()

        return ans

    def coder_callback(self) -> None:
        """Ask the coders the velocity and publish it."""
        coder_right_res = bytearray(1)
        coder_left_res = bytearray(1)

        def using_i2c():
            self.i2c.readfrom_into(self.coder_addresses[0], coder_right_res)
            self.i2c.readfrom_into(self.coder_addresses[1], coder_left_res)

        self.use_i2c(using_i2c)

        wheel_circumference = 2 * math.pi * self.wheel_radius

        # Calculation of the wheel velocity:
        # - coder_XXX_res & 0x7F: Get the 7 least significant bits, that represents the rotation velocity
        # - coder_XXX_res & 0x80: Get the 8th bit, if 1 the wheel is moving in the opposite direction
        # - (-1 if coder_XXX_res & 0x80 else 1): If the 8th bit is set, the wheel is moving in the opposite direction
        # - coder_message_ratio: The ratio between the message and the wheel rotation
        right_wheel_velocity = (coder_right_res & 0x7F) * wheel_circumference * (
            -1 if coder_right_res & 0x80 else 1) * self.coder_message_ratio
        left_wheel_velocity = (coder_left_res & 0x7F) * wheel_circumference * (
            -1 if coder_left_res & 0x80 else 1) * self.coder_message_ratio

        odom_msg = nav_msgs_.Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = self.odometry_frame_id
        odom_msg.child_frame_id = self.odometry_child_frame_id

        # Calculation of the linear and angular velocity (source: https://control.ros.org/rolling/doc/ros2_controllers/doc/mobile_robot_kinematics.html#differential-drive-robot): # noqa: E501
        odom_msg.twist.twist.linear.x = (
                (wheel_circumference * right_wheel_velocity - wheel_circumference * left_wheel_velocity) / 2)
        odom_msg.twist.twist.angular.z = (
                (wheel_circumference * right_wheel_velocity + wheel_circumference * left_wheel_velocity)
                / self.wheel_separation)

        odom_msg.twist.covariance = self.odometry_covariance_matrix

        self.odom_pub.publish(odom_msg)

    def cmd_vel_callback(self, msg: geo_msgs.Twist) -> None:
        """
        Set the velocity of the wheels.

        :param msg: The velocity to set
        :return: None
        """
        # Calculation of the wheels velocities (source: https://control.ros.org/rolling/doc/ros2_controllers/doc/mobile_robot_kinematics.html#differential-drive-robot): # noqa: E501
        velocity_left = int(msg.linear.x - msg.angular.z * self.wheel_separation / 2)
        velocity_right = int(msg.linear.x + msg.angular.z * self.wheel_separation / 2)

        wheel_circumference = 2 * math.pi * self.wheel_radius

        # Rotation speed calculation for the coders
        left_wheel_rotation = velocity_left / wheel_circumference / self.coder_message_ratio + 0x80 * (
                velocity_left < 0)
        right_wheel_rotation = velocity_right / wheel_circumference / self.coder_message_ratio + 0x80 * (
                velocity_right < 0)

        def using_i2c():
            self.i2c.writeto(self.coder_addresses[0], int(right_wheel_rotation).to_bytes(1))
            self.i2c.writeto(self.coder_addresses[1], int(left_wheel_rotation).to_bytes(1))

        self.use_i2c(using_i2c)

    def imu_callback(self) -> None:
        """Ask the IMU the acceleration and angular velocity and publish it."""

        def using_i2c():
            return self.imu.read_accelerometer_gyro_data()

        ax, ay, az, gx, gy, gz = self.use_i2c(using_i2c)

        imu_msg = sensor_msgs_.Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = self.imu_frame_id

        # Conversion from g to m/s^2
        g = 9.81
        imu_msg.linear_acceleration.x = ax * g
        imu_msg.linear_acceleration.y = ay * g
        imu_msg.linear_acceleration.z = az * g

        # Conversion from dps to rad/s
        dps_to_rad = math.pi / 180
        imu_msg.angular_velocity.x = gx * dps_to_rad
        imu_msg.angular_velocity.y = gy * dps_to_rad
        imu_msg.angular_velocity.z = gz * dps_to_rad

        imu_msg.linear_acceleration_covariance = self.imu_acceleration_covariance_matrix
        imu_msg.orientation_covariance = self.imu_orientation_covariance_matrix

        self.imu_pub.publish(imu_msg)


def main(args=None):
    rclpy.init(args=args)
    node = I2CInterface()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
