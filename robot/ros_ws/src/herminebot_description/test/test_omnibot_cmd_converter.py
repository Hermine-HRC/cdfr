import time

import geometry_msgs.msg as geo_msgs
from herminebot_description import OmnibotCmdConverter
import pytest
import rclpy
import sensor_msgs.msg as sensor_msg
import std_msgs.msg as std_msg

FRONT = '/wheel_cmd/front'
LEFT = '/wheel_cmd/left'
RIGHT = '/wheel_cmd/right'


class OmnibotCmdConverterWrapper(OmnibotCmdConverter):
    def __init__(self):
        super().__init__()

        self.cmd_subs = []
        self.cmd_res = {}
        self.twist_res: geo_msgs.TwistWithCovarianceStamped = None

        for topic, cb in zip([FRONT, LEFT, RIGHT], [self.front_cb, self.left_cb, self.right_cb]):
            self.cmd_subs.append(self.create_subscription(std_msg.Float64, topic, cb, 1))

        self.twist_sub = self.create_subscription(geo_msgs.TwistWithCovarianceStamped, self.wheels_twist_topic,
                                                  self.wheels_twist_cb, 1)

    def get_speed(self, topic_name: str) -> float:
        return self.cmd_res[topic_name]

    def front_cb(self, msg):
        self.cmd_res[FRONT] = msg.data

    def left_cb(self, msg):
        self.cmd_res[LEFT] = msg.data

    def right_cb(self, msg):
        self.cmd_res[RIGHT] = msg.data

    def wheels_twist_cb(self, msg):
        self.twist_res = msg

    def wait_for_cmd(self, timeout: float = 0.2) -> bool:
        self.cmd_res.clear()
        start = time.time()
        while time.time() - start < timeout * 1e9:
            if len(self.cmd_res) == 3:
                return True
            rclpy.spin_once(self, timeout_sec=0.01)
        return False

    def wait_for_twist(self, timeout: float = 0.2) -> bool:
        self.twist_res = None
        start = time.time()
        while time.time() - start < timeout * 1e9:
            if self.twist_res:
                return True
            rclpy.spin_once(self, timeout_sec=0.01)
        return False


def test_omnibot_cmd_converter():
    rclpy.init()
    try:
        node = OmnibotCmdConverterWrapper()
        req = geo_msgs.Twist()
        joint_states = sensor_msg.JointState()
        joint_states.name = ['front_wheel_joint', 'right_wheel_joint', 'left_wheel_joint']

        # Pure x translation
        req.linear.x = 1.0
        req.linear.y = 0.0
        req.angular.z = 0.0
        node.cmd_callback(req)
        assert node.wait_for_cmd()
        assert node.get_speed(FRONT) == pytest.approx(0.0, abs=1e-3)
        assert node.get_speed(RIGHT) == pytest.approx(8.66, abs=1e-3)
        assert node.get_speed(LEFT) == pytest.approx(-8.66, abs=1e-3)
        joint_states.velocity = [0.0, 8.66, -8.66]
        node.joint_states_callback(joint_states)
        node.wait_for_twist()
        assert node.twist_res.twist.twist.linear.x == pytest.approx(1.0, abs=1e-3)
        assert node.twist_res.twist.twist.linear.y == pytest.approx(0.0, abs=1e-3)
        assert node.twist_res.twist.twist.angular.z == pytest.approx(0.0, abs=1e-3)

        # Pure y translation
        req.linear.x = 0.0
        req.linear.y = 1.0
        req.angular.z = 0.0
        node.cmd_callback(req)
        assert node.wait_for_cmd()
        assert node.get_speed(FRONT) == pytest.approx(10.0, abs=1e-3)
        assert node.get_speed(RIGHT) == pytest.approx(-5.0, abs=1e-3)
        assert node.get_speed(LEFT) == pytest.approx(-5.0, abs=1e-3)
        joint_states.velocity = [10.0, -5.0, -5.0]
        node.joint_states_callback(joint_states)
        node.wait_for_twist()
        assert node.twist_res.twist.twist.linear.x == pytest.approx(0.0, abs=1e-3)
        assert node.twist_res.twist.twist.linear.y == pytest.approx(1.0, abs=1e-3)
        assert node.twist_res.twist.twist.angular.z == pytest.approx(0.0, abs=1e-3)

        # Pure rotation
        req.linear.x = 0.0
        req.linear.y = 0.0
        req.angular.z = 1.0
        node.cmd_callback(req)
        assert node.wait_for_cmd()
        assert node.get_speed(FRONT) == pytest.approx(5.0, abs=1e-3)
        assert node.get_speed(RIGHT) == pytest.approx(5.0, abs=1e-3)
        assert node.get_speed(LEFT) == pytest.approx(5.0, abs=1e-3)
        joint_states.velocity = [5.0, 5.0, 5.0]
        node.joint_states_callback(joint_states)
        node.wait_for_twist()
        assert node.twist_res.twist.twist.linear.x == pytest.approx(0.0, abs=1e-3)
        assert node.twist_res.twist.twist.linear.y == pytest.approx(0.0, abs=1e-3)
        assert node.twist_res.twist.twist.angular.z == pytest.approx(1.0, abs=1e-3)

        # All
        req.linear.x = 1.0
        req.linear.y = 1.0
        req.angular.z = 1.0
        node.cmd_callback(req)
        assert node.wait_for_cmd()
        assert node.get_speed(FRONT) == pytest.approx(15.0, abs=1e-3)
        assert node.get_speed(RIGHT) == pytest.approx(8.66, abs=1e-3)
        assert node.get_speed(LEFT) == pytest.approx(-8.66, abs=1e-3)
        joint_states.velocity = [15.0, 8.66, -8.66]
        node.joint_states_callback(joint_states)
        node.wait_for_twist()
        assert node.twist_res.twist.twist.linear.x == pytest.approx(1.0, abs=1e-3)
        assert node.twist_res.twist.twist.linear.y == pytest.approx(1.0, abs=1e-3)
        assert node.twist_res.twist.twist.angular.z == pytest.approx(1.0, abs=1e-3)

    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    pytest.main(['-v'])
