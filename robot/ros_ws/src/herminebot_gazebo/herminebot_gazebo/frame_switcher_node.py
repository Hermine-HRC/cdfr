#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import nav_msgs.msg as nav_msgs_
import sensor_msgs.msg as sensor_msgs


class FrameSwitcherNode(Node):
    """
    This node is a bridge between Gazebo and ROS to adapt the frames.
    Listen to a topic, change the frame and publishes to another topic.
    """

    def __init__(self):
        super().__init__('frame_switcher_node')

        self.pubs = {
            "lidar_link": self.create_publisher(sensor_msgs.LaserScan, "/scan", 10),
            "imu_link": self.create_publisher(sensor_msgs.Imu, "/imu/data", 10),
            "odom": self.create_publisher(nav_msgs_.Odometry, "/wheel/odometry", 10)
        }

        self.subs = {
            "lidar_link": self.create_subscription(sensor_msgs.LaserScan, "/gz/scan", self.cb, 10),
            "imu_link": self.create_subscription(sensor_msgs.Imu, "/gz/imu", self.cb, 10),
            "odom": self.create_subscription(nav_msgs_.Odometry, "/gz/wheel/odometry", self.cb, 10)
        }

        self.get_logger().info("FrameSwitcher node has been started.")

    @staticmethod
    def frame_converter(frame: str) -> str:
        match frame:
            case "herminebot/lidar_link/lidar":
                return "lidar_link"
            case "herminebot/imu_link/hrc_imu":
                return "imu_link"
            case "herminebot/odom":
                return "odom"
            case _:
                return ""

    def cb(self, msg) -> None:
        frame = self.frame_converter(msg.header.frame_id)
        if frame == "":
            self.get_logger().error(f"Invalid frame id '{msg.header.frame_id}'")
            return

        if isinstance(msg, nav_msgs_.Odometry):
            msg.child_frame_id = "base_footprint"

        msg.header.frame_id = frame
        self.pubs[frame].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FrameSwitcherNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
