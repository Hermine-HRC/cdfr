#include "herminebot_behaviors/navigator_preemption.hpp"

namespace hrc_bt_navigator
{

std::string NavigatorPreemption::getDefaultBTFilepath(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node)
{
    std::string default_bt_xml_filename;
    auto node = parent_node.lock();

    if (!node->has_parameter("default_preemption_bt_xml")) {
        std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("herminebot_behaviors");
        node->declare_parameter<std::string>(
            "default_preemption_bt_xml",
            pkg_share_dir + "/behavior_trees/preemption.xml"
        );
    }

    node->get_parameter("default_preemption_bt_xml", default_bt_xml_filename);

    return default_bt_xml_filename;
}

bool NavigatorPreemption::goalReceived(ActionT::Goal::ConstSharedPtr goal)
{
    auto bt_xml_filename = goal->behavior_tree;

    if (!bt_action_server_->loadBehaviorTree(bt_xml_filename)) {
        RCLCPP_ERROR(logger_, "BT file not found: %s. Preemption canceled.", bt_xml_filename.c_str());
        return false;
    }

    return true;
}

void NavigatorPreemption::goalCompleted(
    typename ActionT::Result::SharedPtr /*result*/,
    const nav2_behavior_tree::BtStatus /*final_bt_status*/)
{
}

void NavigatorPreemption::onLoop() {}

void NavigatorPreemption::onPreempt(ActionT::Goal::ConstSharedPtr /*goal*/) 
{
    RCLCPP_WARN(logger_, 
        "The request is rejected because the navigator needs to be finished before going to a new goal."
        "\nCancel the current goal and send a new action request if you want to do another preemption."
    );
}

}
