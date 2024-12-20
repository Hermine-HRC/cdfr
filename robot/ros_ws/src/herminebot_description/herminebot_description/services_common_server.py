#! /usr/bin/env python3
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time

import tf2_ros
import tf_transformations as tft

import hrc_interfaces.srv as hrc_srv


class ServicesCommonServer(Node):
    """
    Server that serves services
    """
    def __init__(self):
        super().__init__("services_common_server")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_robot_pose_service = self.create_service(hrc_srv.GetRobotPose, "get_robot_pose",
                                                          self.get_robot_pose_cb)

    def get_robot_pose_cb(self, request: hrc_srv.GetRobotPose.Request,
                          response: hrc_srv.GetRobotPose.Response) -> hrc_srv.GetRobotPose.Response:
        """
        Callback to get the robot pose.
        :param request: The service request
        :param response: The service response
        :return: The service response with the robot pose
        """
        try:
            trans = self.tf_buffer.lookup_transform(request.base_frame, request.robot_frame, rclpy.time.Time(),
                                                    rclpy.duration.Duration(seconds=1))
            response.robot_pose.x = trans.transform.translation.x
            response.robot_pose.y = trans.transform.translation.y

            quat = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z,
                    trans.transform.rotation.w]
            response.robot_pose.theta = tft.euler_from_quaternion(quat)[2]

        except tf2_ros.TransformException as ex:
            self.get_logger().error(
                f"Could not get transform from '{request.robot_frame}' to '{request.base_frame}' : {ex}")

        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ServicesCommonServer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
