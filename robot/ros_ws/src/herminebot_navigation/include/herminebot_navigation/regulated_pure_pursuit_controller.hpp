#ifndef HRC_REGULATED_PURE_PURSUIT_CONTROLLER_HPP
#define HRC_REGULATED_PURE_PURSUIT_CONTROLLER_HPP

#include "nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"

namespace hrc_rpp_controller
{
    
/**
 * @brief Extend nav2 RPP controller to allow reversing AND rotation to heading
 */
class RegulatedPurePursuitController : public nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
{
public:
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
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros
    );

    /**
     * @brief Compute the best command given the current pose and velocity, with possible debug information
     *
     * Same as above computeVelocityCommands, but with debug results.
     * If the results pointer is not null, additional information about the twists
     * evaluated will be in results after the call.
     *
     * @param pose      Current robot pose
     * @param velocity  Current robot velocity
     * @param goal_checker   Ptr to the goal checker for this task in case useful in computing commands
     * @return          Best command
     */
    geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * /*goal_checker*/) override;

protected:
    /**
     * @brief Whether robot should rotate to rough path heading
     * @param carrot_pose current lookahead point
     * @param angle_to_path Angle of robot output relatie to carrot marker
     * @return Whether should rotate to path heading
     */
    bool shouldRotateToPath(
        const geometry_msgs::msg::PoseStamped & carrot_pose, 
        double & angle_to_path
    );

    /**
     * @brief Whether the robot should move backward
     * @return True if the robot should move backward
     */
    bool useReverse() const;

    /**
     * @brief Callback executed when a parameter change is detected
     * @param event ParameterEvent message
     */
    rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

protected:
    double max_rotation_before_reverse_;
};

}

#endif // HRC_REGULATED_PURE_PURSUIT_CONTROLLER_HPP
