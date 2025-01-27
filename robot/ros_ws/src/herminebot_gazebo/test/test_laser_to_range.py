import time

from herminebot_gazebo import LaserToRangeNode

import pytest
import rclpy
from rclpy.duration import Duration
import sensor_msgs.msg as sensor_msgs


def test_laser_to_range():
    rclpy.init()
    try:
        node = LaserToRangeNode()
        range_out: sensor_msgs.Range = None

        def range_callback(msg_):
            nonlocal range_out
            range_out = msg_

        _ = node.create_subscription(sensor_msgs.Range, '/range', range_callback, 1)

        msg = sensor_msgs.LaserScan()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = 'sensor_link_laser'
        msg.angle_min = -0.2
        msg.angle_max = 0.2
        msg.angle_increment = 0.04
        msg.range_min = 0.1
        msg.range_max = 4.0
        msg.ranges = [1.1, 2.9, 0.13, 0.87, 1.31, 0.14, 0.57, 3.8, 1.2]

        node.laser_callback(msg)
        start_time = node.get_clock().now()

        # Get result
        while rclpy.ok() and node.get_clock().now() - start_time < Duration(seconds=1):
            if range_out is not None:
                break
            rclpy.spin_once(node)
            time.sleep(0.01)

        assert range_out is not None

        # Check result
        assert range_out.header.frame_id == ''
        assert range_out.radiation_type == sensor_msgs.Range.INFRARED
        assert range_out.field_of_view == pytest.approx(msg.angle_increment * len(msg.ranges))
        assert range_out.min_range == pytest.approx(msg.range_min)
        assert range_out.max_range == pytest.approx(msg.range_max)
        assert range_out.range == pytest.approx(min(msg.ranges))

    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    pytest.main(['-v'])
