#include "herminebot_behaviors/behavior/move_elevator.hpp"
#include "nav2_util/node_utils.hpp"
#include <string>

namespace hrc_behaviors
{

MoveElevator::MoveElevator() : nav2_behaviors::TimedBehavior<MoveElevatorAction>()
{
    feedback_ = std::make_shared<MoveElevatorAction::Feedback>();
    position_accuracy_ = 0.0;
}

MoveElevator::~MoveElevator() = default;

void MoveElevator::onConfigure()
{
    auto node = node_.lock();
    if (!node) {
        throw std::runtime_error{"Failed to lock node"};
    }

    nav2_util::declare_parameter_if_not_declared(node, "position_accuracy", rclcpp::ParameterValue(0.01));
    node->get_parameter("position_accuracy", position_accuracy_);
}

nav2_behaviors::Status MoveElevator::onRun(const std::shared_ptr<const MoveElevatorAction::Goal> command)
{
    auto node = node_.lock();
    if (!node) {
        RCLCPP_ERROR(logger_, "Failed to lock node");
        return nav2_behaviors::Status::FAILED;
    }

    if (command->elevators_ids.size() != command->elevators_poses.size()) {
        RCLCPP_ERROR(logger_, "The size of elevator_ids and elevator_poses doesn't match");
        return nav2_behaviors::Status::FAILED;
    }

    tf2::Transform tf_transform;
    for (const int id : command->elevators_ids) {
        if (!nav2_util::getTransform(
                "elevator_" + std::to_string(id) + "_link",
                "elevator_support_" + std::to_string(id) + "_link",
                tf2::Duration(this->clock_->now().nanoseconds()),
                tf_, tf_transform))
        {
            RCLCPP_ERROR(logger_, "Cannot get transform of elevator with id '%d'", id);
            return nav2_behaviors::Status::FAILED;
        }
    }

    elevators_.clear();

    int id;
    double pose;
    for (uint8_t i = 0 ; i < command->elevators_ids.size() ; i ++) {
        id = command->elevators_ids.at(i);
        pose = command->elevators_poses.at(i);

        elevators_.insert({id, pose});
        if (pose_pubs_.count(id) == 0) {
            pose_pubs_.insert(
                {
                    id,
                    node->template create_publisher<std_msgs::msg::Float64>(
                        "elevator_pose/id_" + std::to_string(id), 1)
                });
            pose_pubs_.at(id)->on_activate();
        }

        RCLCPP_INFO(logger_, "Moving elevator with id '%d' to %.3f m", id, pose);
    }

    if (command->time_allowance.sec > 0 || command->time_allowance.nanosec > 0) {
        end_time_ = this->clock_->now() + command->time_allowance;
    }
    else {
        end_time_ = this->clock_->now() + rclcpp::Duration(100, 0); // No time limit
    }

    return nav2_behaviors::Status::SUCCEEDED;
}

nav2_behaviors::Status MoveElevator::onCycleUpdate()
{
    rclcpp::Duration time_remaining = end_time_ - this->clock_->now();
    if (time_remaining.seconds() < 0.0 && end_time_.seconds() > 0) {
        RCLCPP_WARN(
            logger_,
            "Exceeded time allowance before reaching the elevators goals - Exiting MoveElevator"
        );
        return nav2_behaviors::Status::FAILED;
    }

    feedback_->current_poses.clear();

    nav2_behaviors::Status status = nav2_behaviors::Status::SUCCEEDED;

    tf2::Transform tf_transform;
    tf2::Vector3 p_v3_s(0.0, 0.0, 0.0);
    tf2::Vector3 p_v3_b;
    double current_pose;
    std_msgs::msg::Float64 msg;
    for (const auto& [id, pose]: elevators_) {
        if (!nav2_util::getTransform(
                "elevator_" + std::to_string(id) + "_link",
                "elevator_support_" + std::to_string(id) + "_link",
                tf2::Duration(this->clock_->now().nanoseconds()),
                tf_, tf_transform))
        {
            RCLCPP_ERROR(logger_, "Cannot get transform of elevator with id '%d'", id);
            status = nav2_behaviors::Status::FAILED;
            continue;
        }

        p_v3_b = tf_transform * p_v3_s;
        current_pose = p_v3_b.z();
        feedback_->current_poses.push_back(current_pose);

        if (fabs(current_pose - pose) > position_accuracy_) { // Publish elevator target position
            status = nav2_behaviors::Status::RUNNING;
            msg.set__data(pose);
            pose_pubs_.at(id)->publish(msg);
        }
    }

    action_server_->publish_feedback(feedback_);

    if (status == nav2_behaviors::Status::SUCCEEDED) {
        RCLCPP_INFO(logger_, "All elevators positions reached");
    }

    return status;
}

void MoveElevator::onCleanup()
{
    elevators_.clear();

    for (auto& [id, pub] : pose_pubs_) {
        pub->on_deactivate();
    }
    pose_pubs_.clear();
}

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(hrc_behaviors::MoveElevator, nav2_core::Behavior)
