#ifndef OMNI_DRIVE_HPP
#define OMNI_DRIVE_HPP

#include "nav2_behaviors/timed_behavior.hpp"
#include "hrc_interfaces/action/omni_drive.hpp"

namespace hrc_behaviors
{

using OmniDriveAction = hrc_interfaces::action::OmniDrive;

class OmniDrive : public nav2_behaviors::TimedBehavior<OmniDriveAction>
{
public:
    OmniDrive();

    /**
     * @brief Initialization to run behavior
     * @param command Goal to execute
     * @return Status of behavior
     */
    nav2_behaviors::ResultStatus onRun(const std::shared_ptr<const OmniDriveAction::Goal> command) override;

    /**
     * @brief Configuration of behavior action
     */
    void onConfigure() override;

    /**
     * @brief Loop function to run behavior
     * @return Status of behavior
     */
    nav2_behaviors::ResultStatus onCycleUpdate() override;

    /**
     * @brief Method to determine the required costmap info
     * @return costmap resources needed
     */
    nav2_core::CostmapInfoType getResourceInfo() override {return nav2_core::CostmapInfoType::LOCAL;}

protected:
    geometry_msgs::msg::Point target_;
    geometry_msgs::msg::PoseStamped initial_pose_;
    double speed_;
    double angle_;
    double distance_to_travel_;
    rclcpp::Time end_time_;
    rclcpp::Duration time_allowance_{0, 0};
    OmniDriveAction::Feedback::SharedPtr feedback_;
};

} // namespace hrc_behaviors

#endif  // OMNI_DRIVE_HPP
