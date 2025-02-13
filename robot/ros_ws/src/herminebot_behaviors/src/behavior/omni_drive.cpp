#include "herminebot_behaviors/behavior/omni_drive.hpp"
#include "nav2_util/node_utils.hpp"

using Result = nav2_behaviors::ResultStatus;
using Status = nav2_behaviors::Status;

namespace hrc_behaviors
{

OmniDrive::OmniDrive() : nav2_behaviors::TimedBehavior<OmniDriveAction>()
{
    feedback_ = std::make_shared<OmniDriveAction::Feedback>();
}

void OmniDrive::onConfigure() {}

Result OmniDrive::onRun(const std::shared_ptr<const OmniDriveAction::Goal> command)
{
    auto node = node_.lock();
    if (!node) {
        RCLCPP_ERROR(logger_, "Failed to lock node");
        return Result{Status::FAILED, OmniDriveAction::Result::UNKNOWN};
    }

    if (command->target.z != 0.0) {
        RCLCPP_ERROR(logger_, "OmniDrive only supports XY movement");
        return Result{Status::FAILED, OmniDriveAction::Result::INVALID_INPUT};
    }

    target_ = command->target;
    speed_ = command->speed;
    time_allowance_ = command->time_allowance;

    if (command->time_allowance.sec > 0 || command->time_allowance.nanosec > 0) {
        end_time_ = this->clock_->now() + command->time_allowance;
    }
    else {
        end_time_ = this->clock_->now() + rclcpp::Duration(100, 0); // No time limit
    }

    if (!nav2_util::getCurrentPose(
            initial_pose_, *this->tf_, this->local_frame_, this->robot_base_frame_,
            this->transform_tolerance_))
    {
        RCLCPP_ERROR(this->logger_, "Initial robot pose is not available.");
        return Result{Status::FAILED, OmniDriveAction::Result::TF_ERROR};
    }

    distance_to_travel_ = hypot(target_.x, target_.y);
    angle_ = atan2(target_.y, target_.x);

    RCLCPP_INFO(logger_, "Robot will move %.2f m at %.2f m/s", distance_to_travel_, speed_);

    return Result{Status::SUCCEEDED, OmniDriveAction::Result::NONE};
}

Result OmniDrive::onCycleUpdate()
{
    rclcpp::Duration time_remaining = end_time_ - this->clock_->now();
    if (time_remaining.seconds() < 0.0 && end_time_.seconds() > 0) {
        stopRobot();
        RCLCPP_WARN(
            logger_,
            "Exceeded time allowance before reaching the target - Exiting OmniDrive"
        );
        return Result{Status::FAILED, OmniDriveAction::Result::TIMEOUT};
    }

    geometry_msgs::msg::PoseStamped current_pose;
    if (!nav2_util::getCurrentPose(
            current_pose, *this->tf_, this->local_frame_, this->robot_base_frame_,
            this->transform_tolerance_))
    {
        RCLCPP_ERROR(logger_, "Current robot pose is not available.");
        return Result{Status::FAILED, OmniDriveAction::Result::TF_ERROR};
    }

    double dx = initial_pose_.pose.position.x - current_pose.pose.position.x;
    double dy = initial_pose_.pose.position.y - current_pose.pose.position.y;
    double distance = hypot(dx, dy);

    feedback_->distance_traveled = distance;

    auto cmd_vel = std::make_unique<geometry_msgs::msg::TwistStamped>();
    cmd_vel->header.stamp = this->clock_->now();
    cmd_vel->header.frame_id = this->robot_base_frame_;
    cmd_vel->twist.linear.x = speed_ * cos(angle_);
    cmd_vel->twist.linear.y = speed_ * sin(angle_);
    cmd_vel->twist.angular.z = 0.0;

    action_server_->publish_feedback(feedback_);

    if (distance >= distance_to_travel_) {
        stopRobot();
        RCLCPP_INFO(logger_, "OmniDrive reached target");
        return Result{Status::SUCCEEDED, OmniDriveAction::Result::NONE};
    }

    this->vel_pub_->publish(std::move(cmd_vel));

    return Result{Status::RUNNING, OmniDriveAction::Result::NONE};
}

} // namespace hrc_behaviors

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(hrc_behaviors::OmniDrive, nav2_core::Behavior)
