#include "herminebot_navigation/hrc_regulated_rotation_controller.hpp"
#include "nav2_util/node_utils.hpp"

namespace hrc_regulated_rotation_controller
{

RegulatedRotationController::RegulatedRotationController()
  : lp_loader_("nav2_core", "nav2_core::Controller"),
    primary_controller_(nullptr), final_orientation_(0.0), integral_(0.0), previous_error_(0.0)
{
}

void RegulatedRotationController::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    plugin_name_ = name;
    node_ = parent;
    auto node = parent.lock();

    logger_ = node->get_logger();

    std::string primary_controller;
    nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + ".primary_controller", rclcpp::PARAMETER_STRING);
    nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + ".p_gain", rclcpp::ParameterValue(0.2));
    nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + ".i_gain", rclcpp::ParameterValue(0.02));
    nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + ".d_gain", rclcpp::ParameterValue(2.0));
    nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + ".max_rotation_vel", rclcpp::ParameterValue(1.5));

    node->get_parameter(plugin_name_ + ".p_gain", p_gain_);
    node->get_parameter(plugin_name_ + ".i_gain", i_gain_);
    node->get_parameter(plugin_name_ + ".d_gain", d_gain_);
    node->get_parameter(plugin_name_ + ".max_rotation_vel", max_rotation_vel_);
    node->get_parameter(plugin_name_ + ".primary_controller", primary_controller);

    try {
        primary_controller_ = lp_loader_.createUniqueInstance(primary_controller);
        RCLCPP_INFO(
            logger_, "Created internal controler for regulated rotation: %s of type %s",
            plugin_name_.c_str(), primary_controller.c_str());
    } catch (const pluginlib::PluginlibException & ex) {
        RCLCPP_ERROR(
            logger_, "Failed to create internal controller '%s' for regulated rotation: %s",
            primary_controller.c_str(), ex.what());
        throw;
    }

    primary_controller_->configure(parent, name, tf, costmap_ros);
}

void RegulatedRotationController::activate()
{
    RCLCPP_INFO(
        logger_,
        "Activating controller: %s of type "
        "regulated_rotation_controller::RegulatedRotationController",
        plugin_name_.c_str());
    primary_controller_->activate();
    auto node = node_.lock();
    dyn_params_handler_ = node->add_on_set_parameters_callback(
        std::bind(
            &RegulatedRotationController::dynamicParametersCallback,
            this, std::placeholders::_1));
}

void RegulatedRotationController::deactivate()
{
    RCLCPP_INFO(
        logger_,
        "Deactivating controller: %s of type "
        "regulated_rotation_controller::RegulatedRotationController",
        plugin_name_.c_str());
    primary_controller_->deactivate();

    if (auto node = node_.lock()) {
        node->remove_on_set_parameters_callback(dyn_params_handler_.get());
    }
    dyn_params_handler_.reset();
}

void RegulatedRotationController::cleanup()
{
    RCLCPP_INFO(
        logger_,
        "Cleaning up controller: %s of type "
        "regulated_rotation_controller::RegulatedRotationController",
        plugin_name_.c_str());
    primary_controller_->cleanup();
    primary_controller_.reset();
}

geometry_msgs::msg::TwistStamped RegulatedRotationController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker)
{
    auto cmd_vel = primary_controller_->computeVelocityCommands(pose, velocity, goal_checker);
    cmd_vel.twist.angular.z = getRotationVelocity(tf2::getYaw(pose.pose.orientation), final_orientation_);
    return cmd_vel;
}

double RegulatedRotationController::getRotationVelocity(
    const double& current_orientation, const double& target_orientation)
{
    double error = target_orientation - current_orientation;
    if (error > M_PI) { // Wrap around the circle
        error -= 2 * M_PI;
    }
    else if (error < -M_PI) {
        error += 2 * M_PI;
    }

    double proportional = p_gain_ * error;
    integral_ += error;
    double integral = integral_ * i_gain_;
    double derivative = d_gain_ * (error - previous_error_);

    previous_error_ = error;

    return std::clamp(
        (proportional + integral + derivative) * max_rotation_vel_, -max_rotation_vel_, max_rotation_vel_);
}

void RegulatedRotationController::setPlan(const nav_msgs::msg::Path & path)
{
    const double fo = tf2::getYaw(path.poses.back().pose.orientation);
    // Whether the final orientation is not the same as the current one
    if (round(fo * 1e3) != round(final_orientation_ * 1e3)) {
        final_orientation_ = fo;

        // Reset PID variables
        integral_ = 0.0;
        previous_error_ = 0.0;
    }

    primary_controller_->setPlan(path);
}

void RegulatedRotationController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
    primary_controller_->setSpeedLimit(speed_limit, percentage);
}

void RegulatedRotationController::reset()
{
    primary_controller_->reset();
}

rcl_interfaces::msg::SetParametersResult
RegulatedRotationController::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    std::lock_guard<std::mutex> lock_reinit(mutex_);

    for (auto parameter : parameters) {
        if (parameter.get_name() == plugin_name_ + ".p_gain") {
            p_gain_ = parameter.as_double();
        }
        else if (parameter.get_name() == plugin_name_ + ".i_gain") {
            i_gain_ = parameter.as_double();
        }
        else if (parameter.get_name() == plugin_name_ + ".d_gain") {
            d_gain_ = parameter.as_double();
        }
        else if (parameter.get_name() == plugin_name_ + ".max_rotation_vel") {
            max_rotation_vel_ = parameter.as_double();
        }
    }

    result.successful = true;
    return result;
}

} // namespace hrc_regulated_rotation_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    hrc_regulated_rotation_controller::RegulatedRotationController,
    nav2_core::Controller)
