#include "herminebot_behaviors/action/move_elevator_action.hpp"

namespace hrc_behavior_tree
{

MoveElevatorAction::MoveElevatorAction(
    const std::string& xml_tag_name,
    const std::string& action_name,
    const BT::NodeConfiguration& conf)
    : nav2_behavior_tree::BtActionNode<hrc_interfaces::action::MoveElevators>(xml_tag_name, action_name, conf)
{
    double time_allowance;
    getInput("time_allowance", time_allowance);
    getInput<std::vector<int>>("elevators_ids", goal_.elevators_ids);
    getInput<std::vector<double>>("elevators_poses", goal_.elevators_poses);
    goal_.time_allowance = rclcpp::Duration::from_seconds(time_allowance);
}

}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config) {
        return std::make_unique<hrc_behavior_tree::MoveElevatorAction>(name, "move_elevators", config);
    };

    factory.registerBuilder<hrc_behavior_tree::MoveElevatorAction>("MoveElevators", builder);
}
