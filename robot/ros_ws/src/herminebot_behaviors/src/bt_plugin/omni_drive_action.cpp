#include "herminebot_behaviors/bt_plugin/omni_drive_action.hpp"

namespace hrc_behavior_tree
{

OmniDriveAction::OmniDriveAction(
    const std::string& xml_tag_name,
    const std::string& action_name,
    const BT::NodeConfiguration& conf)
  : nav2_behavior_tree::BtActionNode<hrc_interfaces::action::OmniDrive>(xml_tag_name, action_name, conf)
{
    double time_allowance;
    double speed, x, y;
    getInput("time_allowance", time_allowance);
    getInput<double>("target_x", x);
    getInput<double>("target_y", y);
    getInput<double>("speed", speed);

    goal_.target.x = x;
    goal_.target.y = y;
    goal_.speed = (float) speed;
    goal_.time_allowance = rclcpp::Duration::from_seconds(time_allowance);
}

}

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config) {
        return std::make_unique<hrc_behavior_tree::OmniDriveAction>(name, "omni_drive", config);
    };

    factory.registerBuilder<hrc_behavior_tree::OmniDriveAction>("OmniDrive", builder);
}
