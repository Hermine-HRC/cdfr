#include "herminebot_behaviors/bt_plugin/manage_map_service.hpp"
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

    // Transform robot-relative points to map frame
    auto robot_to_map = [robot_pose](const float px, const float py, float& mx, float& my) {
            mx = px * cos(robot_pose.theta) - py * sin(robot_pose.theta) + robot_pose.x;
            my = px * sin(robot_pose.theta) + py * cos(robot_pose.theta) + robot_pose.y;
        };

    for (auto& object : objects) {
        geometry_msgs::msg::Polygon polygon;
        for (std::vector<double>& point : object) {
            geometry_msgs::msg::Point32 p;
            p.x = point[0];
            p.y = point[1];
            if (is_robot_relative) {
                robot_to_map(p.x, p.y, p.x, p.y);
            }
            polygon.points.push_back(p);
        }
        request_->new_objects.push_back(polygon);
    }

    for (auto& point : points_to_remove) {
        geometry_msgs::msg::Point32 p;
        p.x = point[0];
        p.y = point[1];
        if (is_robot_relative) {
            robot_to_map(p.x, p.y, p.x, p.y);
        }
        request_->points_objects_to_remove.push_back(p);
    }
}

} // namespace hrc_behavior_tree

namespace BT
{

template<>
std::vector<std::vector<std::vector<double>>>
convertFromString<std::vector<std::vector<std::vector<double>>>>(StringView str)
{
    std::vector<std::vector<std::vector<double>>> result;
    for (StringView polygon : splitString(str, '|')) {
        auto points = convertFromString<std::vector<std::vector<double>>>(polygon);
        result.push_back(points);
    }
    return result;
}

template<>
std::vector<std::vector<double>>
convertFromString<std::vector<std::vector<double>>>(StringView str)
{
    std::vector<std::vector<double>> result;
    for (StringView point : splitString(str, ';')) {
        std::vector<double> inner_vector;
        for (StringView coord : splitString(point, ',')) {
            inner_vector.push_back(std::stod(coord.to_string()));
        }
        result.push_back(inner_vector);
    }

    return result;
}

}  // namespace BT

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<hrc_behavior_tree::ManageMapService>("ManageMap");
}
