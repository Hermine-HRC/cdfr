#ifndef HRC_BEHAVIORS_BT_PLUGIN_MANAGE_MAP_SERVICE_HPP
#define HRC_BEHAVIORS_BT_PLUGIN_MANAGE_MAP_SERVICE_HPP

#include <string>
#include "nav2_behavior_tree/bt_service_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "hrc_interfaces/srv/manage_objects_map.hpp"
#include "hrc_interfaces/srv/get_robot_pose.hpp"


namespace hrc_behavior_tree
{

class ManageMapService : public nav2_behavior_tree::BtServiceNode<hrc_interfaces::srv::ManageObjectsMap>
{
public:
    /**
     * @brief A constructor for nav2_behavior_tree::ClearEntireCostmapService
     * @param service_node_name Service name this node creates a client for
     * @param config BT node configuration
     */
    ManageMapService(const std::string& service_node_name, const BT::NodeConfiguration& config);

    /**
     * @brief The main override required by a BT service
     * @return BT::NodeStatus Status of tick execution
     */
    void on_tick() override;

    /**
     * @brief Creates list of BT ports
     * @return BT::PortsList Containing basic ports along with node-specific ports
     */
    static BT::PortsList providedPorts()
    {
        return { 
            BT::InputPort<std::vector<std::vector<std::vector<double>>>>("new_objects"),
            BT::InputPort<std::vector<std::vector<double>>>("points_objects_to_remove"),
            BT::InputPort<bool>("is_robot_relative")
        };
    }

protected:
    rclcpp::Client<hrc_interfaces::srv::GetRobotPose>::SharedPtr get_robot_pose_client_;
};

}  // namespace hrc_behavior_tree

namespace BT
{

template <>
std::vector<std::vector<std::vector<double>>> convertFromString(StringView str);

template <>
std::vector<std::vector<double>> convertFromString(StringView str);

}

#endif  // HRC_BEHAVIORS_BT_PLUGIN_MANAGE_MAP_SERVICE_HPP
