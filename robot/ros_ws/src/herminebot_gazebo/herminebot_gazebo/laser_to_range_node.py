#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import sensor_msgs.msg as sensor_msgs


class LaserToRangeNode(Node):
    """
    Convert a laser msg to a range msg.

    Node parameters:
    source_topic: Topic to read
    output_topic: Topic to publish
    frame_id: The frame of the range message
    """

    def __init__(self):
        super().__init__('laser_to_range_node')

        source_topic = self.declare_parameter('source_topic', '/scan').get_parameter_value().string_value
        output_topic = self.declare_parameter('output_topic', '/range').get_parameter_value().string_value
        self.frame = self.declare_parameter('frame_id', '').get_parameter_value().string_value

        self.range_publisher = self.create_publisher(sensor_msgs.Range, output_topic, 10)
        self.laser_subscriber = self.create_subscription(sensor_msgs.LaserScan, source_topic, self.laser_callback, 10)

        self.get_logger().info("LaserToRange node has been started.")

    def laser_callback(self, msg: sensor_msgs.LaserScan) -> None:
        """
        Convert LaserScan to Range and publish it.

        :param msg: The LaserScan message
        :return: None
        """
        if not msg.ranges:
            self.get_logger().warn("Received an empty LaserScan message.")
            return

        range_msg = sensor_msgs.Range()
        range_msg.header = msg.header
        range_msg.header.frame_id = self.frame
        range_msg.radiation_type = sensor_msgs.Range.INFRARED
        range_msg.field_of_view = msg.angle_increment * len(msg.ranges)
        range_msg.min_range = msg.range_min
        range_msg.max_range = msg.range_max
        range_msg.range = min(msg.ranges)

        self.range_publisher.publish(range_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LaserToRangeNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
