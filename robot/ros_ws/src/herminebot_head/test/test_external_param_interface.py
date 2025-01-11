import pytest
import rclpy
import threading
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from herminebot_head import ExternalParamInterface


class TestNode(Node):
    __test__ = False

    def __init__(self):
        super().__init__("test_node")
        self.param = self.declare_parameter("param", "empty_param").get_parameter_value().string_value
        self.add_on_set_parameters_callback(self.dynamic_parameters_callback)

    def dynamic_parameters_callback(self, params: list[rclpy.Parameter]) -> SetParametersResult:
        self.param = params[0].get_parameter_value().string_value
        return SetParametersResult(successful=True)

    def get_param(self) -> str:
        return self.param


def test_external_param_interface():
    rclpy.init()
    thread = None
    try:
        test_node = TestNode()
        assert test_node.get_param() == "empty_param"

        node = ExternalParamInterface("test_node")

        # Spin server node
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(test_node)
        thread = threading.Thread(target=executor.spin, daemon=True)

        # Execute requests
        thread.start()
        assert node.get_params(["param"]).values[0].string_value == "empty_param"
        assert node.set_params({"param": "test_param"})

        assert test_node.get_param() == "test_param"
        assert node.get_params(["param"]).values[0].string_value == "test_param"

    finally:
        if thread is not None:
            thread.join(0.5)
        rclpy.shutdown()


if __name__ == "__main__":
    pytest.main(["-v"])
