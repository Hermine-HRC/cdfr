#ifndef MOVE_ELEVATOR_HPP
#define MOVE_ELEVATOR_HPP

#include "nav2_behaviors/timed_behavior.hpp"
#include "hrc_interfaces/action/move_elevators.hpp"
#include "std_msgs/msg/float64.hpp"
#include <vector>
#include <map>

namespace hrc_behaviors
{
using MoveElevatorAction = hrc_interfaces::action::MoveElevators;

class MoveElevator : public nav2_behaviors::TimedBehavior<MoveElevatorAction>
{
public:
    MoveElevator();
    ~MoveElevator();

    /**
     * @brief Initialization to run behavior
     * @param command Goal to execute
     * @return Status of behavior
     */
    nav2_behaviors::ResultStatus onRun(const std::shared_ptr<const MoveElevatorAction::Goal> command) override;

    /**
     * @brief Configuration of behavior action
     */
    void onConfigure() override;

    void onCleanup() override;

    /**
     * @brief Loop function to run behavior
     * @return Status of behavior
     */
    nav2_behaviors::ResultStatus onCycleUpdate() override;

    /**
     * @brief Method to determine the required costmap info
     * @return costmap resources needed
     */
    nav2_core::CostmapInfoType getResourceInfo() override {return nav2_core::CostmapInfoType::NONE;}

protected:
    std::map<int, rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr> pose_pubs_;
    MoveElevatorAction::Feedback::SharedPtr feedback_;
    std::map<int, double> elevators_;
    rclcpp::Time end_time_;
    double position_accuracy_;
};

}

#endif // MOVE_ELEVATOR_HPP
