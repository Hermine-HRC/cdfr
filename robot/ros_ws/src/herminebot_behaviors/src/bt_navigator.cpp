#include "herminebot_behaviors/bt_navigator.hpp"
#include "herminebot_behaviors/navigator/navigator_preemption.hpp"

namespace hrc_bt_navigator
{

BtNavigator::BtNavigator(rclcpp::NodeOptions options)
  : nav2_bt_navigator::BtNavigator(options)
{}

nav2_util::CallbackReturn BtNavigator::on_configure(const rclcpp_lifecycle::State& state)
{
    nav2_util::CallbackReturn res = nav2_bt_navigator::BtNavigator::on_configure(state);

    nav2_bt_navigator::FeedbackUtils feedback_utils;
    feedback_utils.tf = tf_;
    feedback_utils.global_frame = global_frame_;
    feedback_utils.robot_frame = robot_frame_;
    feedback_utils.transform_tolerance = transform_tolerance_;

    auto plugin_lib_names = get_parameter("plugin_lib_names").as_string_array();

    preempt_navigator_ = std::make_unique<hrc_bt_navigator::NavigatorPreemption>();

    if (!preempt_navigator_->on_configure(
            shared_from_this(), plugin_lib_names, feedback_utils, &plugin_muxer_,
            odom_smoother_))
    {
        return nav2_util::CallbackReturn::FAILURE;
    }

    return res;
}

nav2_util::CallbackReturn BtNavigator::on_activate(const rclcpp_lifecycle::State& state)
{
    if (!preempt_navigator_->on_activate()) {
        return nav2_util::CallbackReturn::FAILURE;
    }

    return nav2_bt_navigator::BtNavigator::on_activate(state);
}

nav2_util::CallbackReturn BtNavigator::on_deactivate(const rclcpp_lifecycle::State& state)
{
    if (!preempt_navigator_->on_deactivate()) {
        return nav2_util::CallbackReturn::FAILURE;
    }

    return nav2_bt_navigator::BtNavigator::on_deactivate(state);
}

nav2_util::CallbackReturn BtNavigator::on_cleanup(const rclcpp_lifecycle::State& state)
{
    if (!preempt_navigator_->on_cleanup()) {
        return nav2_util::CallbackReturn::FAILURE;
    }

    return nav2_bt_navigator::BtNavigator::on_cleanup(state);
}

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(hrc_bt_navigator::BtNavigator)
