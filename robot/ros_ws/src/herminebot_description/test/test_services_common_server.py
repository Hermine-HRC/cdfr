import pytest
import rclpy
from rclpy.duration import Duration
import tf2_ros
import tf_transformations as tft
from herminebot_description import ServicesCommonServer
import geometry_msgs.msg as geo_msgs
import hrc_interfaces.srv as hrc_srvs


def test_get_robot_pose():
    rclpy.init()
    try:
        node = ServicesCommonServer()
        tf_broadcaster = tf2_ros.TransformBroadcaster(node)

        # Set transform
        transform = geo_msgs.TransformStamped()
        transform.header.frame_id = "odom"
        transform.transform.translation.x = 1.2
        transform.transform.translation.y = 2.3
        quat = tft.quaternion_from_euler(0, 0, 1.0)
        transform.transform.rotation = geo_msgs.Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        transform.child_frame_id = "base_link"

        # Publish the transform for 1 second each separated by 100 ms
        for i in range(10):
            transform.header.stamp = (node.get_clock().now() + Duration(nanoseconds=i * 100e6)).to_msg()
            tf_broadcaster.sendTransform(transform)

        rclpy.spin_once(node)  # Update the TF buffer

        req = hrc_srvs.GetRobotPose.Request()
        req.base_frame = "odom"
        req.robot_frame = "base_link"
        res = node.get_robot_pose_cb(req, hrc_srvs.GetRobotPose.Response())

        assert res.robot_pose.x == pytest.approx(1.2)
        assert res.robot_pose.y == pytest.approx(2.3)
        assert res.robot_pose.theta == pytest.approx(1.0)

    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    pytest.main(["-v"])
