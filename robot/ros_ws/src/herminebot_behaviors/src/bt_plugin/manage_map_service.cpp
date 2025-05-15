#include "herminebot_behaviors/bt_plugin/manage_map_service.hpp"
#include "hrc_utils/utils.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <chrono>

namespace hrc_behavior_tree
{

ManageMapService::ManageMapService(
    const std::string& service_node_name,
    const BT::NodeConfiguration& config)
  : BtServiceNode<hrc_interfaces::srv::ManageObjectsMap>(service_node_name, config, "manage_object_map")
{
    get_robot_pose_client_ = node_->create_client<hrc_interfaces::srv::GetRobotPose>("get_robot_pose");
}

void ManageMapService::on_tick()
{
    bool is_robot_relative;
    std::vector<std::vector<std::vector<double>>> objects;
    std::vector<std::vector<double>> points_to_remove;

    getInput<bool>("is_robot_relative", is_robot_relative);
    getInput<std::vector<std::vector<std::vector<double>>>>("new_objects", objects);
    getInput<std::vector<std::vector<double>>>("points_objects_to_remove", points_to_remove);

    // Get robot pose
    geometry_msgs::msg::Pose2D robot_pose;
    if (is_robot_relative) {
        auto req = std::make_shared<hrc_interfaces::srv::GetRobotPose::Request>();
        while (!get_robot_pose_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(node_->get_logger(), "service not available, waiting again...");
        }

        auto result = get_robot_pose_client_->async_send_request(req);

        if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS) {
            robot_pose = result.get()->robot_pose;
        }
        else {
            RCLCPP_ERROR(node_->get_logger(), "Failed to call service get_robot_pose. Ending request");
            return;
        }
    }

    geometry_msgs::msg::Point32 p_robot, p;
    for (auto& object : objects) {
        geometry_msgs::msg::Polygon polygon;
        for (std::vector<double>& point : object) {
            p.x = point[0];
            p.y = point[1];
            if (is_robot_relative) {
                p_robot = p;
                hrc_utils::robotToMap(robot_pose, p_robot, p);
            }
            polygon.points.push_back(p);
        }
        request_->new_objects.push_back(polygon);
    }

    for (auto& point : points_to_remove) {
        p.x = point[0];
        p.y = point[1];
        if (is_robot_relative) {
            p_robot = p;
            hrc_utils::robotToMap(robot_pose, p_robot, p);
        }
        request_->points_objects_to_remove.push_back(p);
    }
}

} // namespace hrc_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<hrc_behavior_tree::ManageMapService>("ManageMap");
}
