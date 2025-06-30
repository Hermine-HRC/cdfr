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
    std::vector<std::vector<geometry_msgs::msg::Point>> objects;
    std::vector<geometry_msgs::msg::Point> points_to_remove;

    getInput<bool>("is_robot_relative", request_->is_robot_relative);
    getInput<std::vector<std::vector<geometry_msgs::msg::Point>>>("new_objects", objects);
    getInput<std::vector<geometry_msgs::msg::Point>>("points_objects_to_remove", points_to_remove);

    geometry_msgs::msg::Point32 p_robot, p;
    request_->new_objects.reserve(objects.size());
    for (auto& object : objects) {
        geometry_msgs::msg::Polygon polygon;
        for (geometry_msgs::msg::Point& point : object) {
            p.x = point.x;
            p.y = point.y;
            polygon.points.push_back(p);
        }
        request_->new_objects.push_back(polygon);
    }

    request_->points_objects_to_remove.reserve(points_to_remove.size());
    for (auto& point : points_to_remove) {
        p.x = point.x;
        p.y = point.y;
        request_->points_objects_to_remove.push_back(p);
    }
}

} // namespace hrc_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<hrc_behavior_tree::ManageMapService>("ManageMap");
}
