import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import GetParameters, SetParameters


class ExternalParamInterface(Node):
    """
    Interface class to get/set parameters of other nodes.

    Example:
    -------
        controller_server = ExternalParamInterface("controller_server")
        params = controller_server.get_params(["FollowPath.desired_linear_vel", "goal_checker.xy_goal_tolerance"])
        vel = params.values[0].double_value
        xy_tol = params.values[1].double_value
        controller_server.set_params({"FollowPath.desired_linear_vel": 0.2, "goal_checker.yaw_goal_tolerance": 0.4})

    """

    def __init__(self, node_name: str):
        """
        Initialize the class for a specific node to get/set the parameters.

        :param node_name: Name of the node to get/set the parameters
        """
        super().__init__(node_name, namespace="external_parameters_interface")

        self.setter_client = self.create_client(SetParameters, "/" + node_name + "/set_parameters")
        self.getter_client = self.create_client(GetParameters, "/" + node_name + "/get_parameters")
        while (not self.setter_client.wait_for_service(timeout_sec=1.0)
               or not self.getter_client.wait_for_service(timeout_sec=1.0)):
            self.get_logger().info("service not available, waiting again...")
        self.setting_req = SetParameters.Request()
        self.getting_req = GetParameters.Request()

    def set_params(self, params: dict) -> bool:
        """
        Set parameters values of the node.

        :param params: Dictionary containing the name of the parameter and the value to set
        :return: Whether the parameters have been set
        """
        self.setting_req.parameters = [Parameter(name=name, value=value).to_parameter_msg() for name, value in
                                       params.items()]
        future = self.setter_client.call_async(self.setting_req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=0.5)
        if future.result() is None:
            self.get_logger().warn(f"Parameters {params.keys()} not set")
            return False
        return True

    def get_params(self, params_names: list[str]) -> GetParameters.Response:
        """
        Get the parameters values of the node.

        :param params_names: Name of the parameters to get the values
        :return: The parameters whose values are listed in the same order as the input order
        """
        self.getting_req.names = params_names
        future = self.getter_client.call_async(self.getting_req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=0.5)
        return future.result()
