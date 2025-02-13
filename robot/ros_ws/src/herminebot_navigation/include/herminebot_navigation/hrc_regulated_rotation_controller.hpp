#ifndef HRC_REGULATED_ROTATION_CONTROLLER_HPP
#define HRC_REGULATED_ROTATION_CONTROLLER_HPP

#include "nav2_core/controller.hpp"

namespace hrc_regulated_rotation_controller
{

/**
 * @class hrc_regulated_rotation_controller::RegulatedRotationController
 * @brief Rotate to final path heading using a PID controller
 */
class RegulatedRotationController : public nav2_core::Controller
{
public:
    RegulatedRotationController();

    /**
     * @brief Configure controller state machine
     * @param parent WeakPtr to node
     * @param name Name of plugin
     * @param tf TF buffer
     * @param costmap_ros Costmap2DROS object of environment
     */
    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
        std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

    /**
     * @brief Cleanup controller state machine
     */
    void cleanup() override;

    /**
     * @brief Activate controller state machine
     */
    void activate() override;

    /**
     * @brief Deactivate controller state machine
     */
    void deactivate() override;

    /**
     * @brief Compute the best command given the current pose and velocity
     * @param pose      Current robot pose
     * @param velocity  Current robot velocity
     * @param goal_checker Ptr to the goal checker for this task in case useful in computing commands
     * @return          Best command
     */
    geometry_msgs::msg::TwistStamped computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped & pose,
        const geometry_msgs::msg::Twist & velocity,
        nav2_core::GoalChecker * /*goal_checker*/) override;

    /**
     * @brief nav2_core setPlan - Sets the global plan
     * @param path The global plan
     */
    void setPlan(const nav_msgs::msg::Path & path) override;

    /**
     * @brief Limits the maximum linear speed of the robot.
     * @param speed_limit expressed in absolute value (in m/s)
     * or in percentage from maximum robot speed.
     * @param percentage Setting speed limit in percentage if true
     * or in absolute values in false case.
     */
    void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

    /**
     * @brief Reset the state of the controller
     */
    void reset() override;

protected:
    /**
     * @brief Callback executed when a parameter change is detected
     * @param event ParameterEvent message
     */
    rcl_interfaces::msg::SetParametersResult
    dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

    /**
     * @brief Calculate the rotating velocity of the robot
     * @param current_orientation Current orientation of the robot
     * @param target_orientation Target orientation of the robot
     */
    double getRotationVelocity(const double& current_orientation, const double& target_orientation);

    rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
    std::string plugin_name_;
    rclcpp::Logger logger_ {rclcpp::get_logger("RegulatedRotationController")};

    pluginlib::ClassLoader<nav2_core::Controller> lp_loader_;
    nav2_core::Controller::Ptr primary_controller_;

    double final_orientation_;
    double integral_;
    double previous_error_;

    // Parameters
    double p_gain_;
    double i_gain_;
    double d_gain_;
    double max_rotation_vel_;

    // Dynamic parameters handler
    std::mutex mutex_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
};

}  // namespace hrc_regulated_rotation_controller

#endif  // HRC_REGULATED_ROTATION_CONTROLLER_HPP
