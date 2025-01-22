#include "gtest/gtest.h"
#include "herminebot_behaviors/behavior/move_elevator.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <iostream>
#include <functional>

using namespace std::chrono_literals;

class MoveElevatorTest : public ::testing::Test
{
protected:
    MoveElevatorTest() {}

    void SetUp() override
    {
        node_lifecycle_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
            "test_move_elevator_node", rclcpp::NodeOptions());

        node_lifecycle_->declare_parameter(
            "costmap_topic",
            rclcpp::ParameterValue(std::string("local_costmap/costmap_raw")));
        node_lifecycle_->declare_parameter(
            "footprint_topic",
            rclcpp::ParameterValue(std::string("local_costmap/published_footprint")));

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_lifecycle_->get_clock());
        auto time_interface = std::make_shared<tf2_ros::CreateTimerROS>(
            node_lifecycle_->get_node_base_interface(), node_lifecycle_->get_node_timers_interface());
        tf_buffer_->setCreateTimerInterface(time_interface);
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        std::string costmap_topic, footprint_topic;
        node_lifecycle_->get_parameter("costmap_topic", costmap_topic);
        node_lifecycle_->get_parameter("footprint_topic", footprint_topic);
        std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_ =
            std::make_shared<nav2_costmap_2d::CostmapSubscriber>(
            node_lifecycle_, costmap_topic);
        std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub_ =
            std::make_shared<nav2_costmap_2d::FootprintSubscriber>(
            node_lifecycle_, footprint_topic, *tf_buffer_);
        std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> collision_checker_ =
            std::make_shared<nav2_costmap_2d::CostmapTopicCollisionChecker>(
            *costmap_sub_, *footprint_sub_,
            node_lifecycle_->get_name());

        behavior_ = std::make_shared<hrc_behaviors::MoveElevator>();
        behavior_->configure(node_lifecycle_, "test_move_elevator_behavior", tf_buffer_, collision_checker_, collision_checker_);
        behavior_->activate();

        client_ = rclcpp_action::create_client<hrc_interfaces::action::MoveElevators>(
            node_lifecycle_->get_node_base_interface(),
            node_lifecycle_->get_node_graph_interface(),
            node_lifecycle_->get_node_logging_interface(),
            node_lifecycle_->get_node_waitables_interface(),
            "test_move_elevator_behavior");

        goal_options_ = rclcpp_action::Client<hrc_interfaces::action::MoveElevators>::SendGoalOptions();
        goal_options_.feedback_callback = std::bind(
            &MoveElevatorTest::handleFeedback, this, std::placeholders::_1, std::placeholders::_2);

        std::cout << "Setup complete" << std::endl;
    }

    void TearDown() override {}

    bool sendCommand(
        const std::vector<int>& elevators_ids,
        const std::vector<double>& elevators_poses,
        const double time_allowance = 0.0)
    {
        if (!client_->wait_for_action_server(1s)) {
            std::cout << "Server not up" << std::endl;
            return false;
        }

        auto goal = hrc_interfaces::action::MoveElevators::Goal();
        goal.elevators_ids = elevators_ids;
        goal.elevators_poses = elevators_poses;
        goal.time_allowance = rclcpp::Duration::from_seconds(time_allowance);
        auto future = client_->async_send_goal(goal, goal_options_);

        if (rclcpp::spin_until_future_complete(node_lifecycle_, future, 1s) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            std::cout << "Send goal failed" << std::endl;
            return false;
        }

        goal_handle_ = future.get();

        if (!goal_handle_) {
            std::cout << "goal was rejected" << std::endl;
            return false;
        }

        elevator_poses_.clear();

        return true;
    }

    nav2_behaviors::Status getOutcome()
    {
        if (getResult().code == rclcpp_action::ResultCode::SUCCEEDED) {
            return nav2_behaviors::Status::SUCCEEDED;
        }

        return nav2_behaviors::Status::FAILED;
    }

    rclcpp_action::ClientGoalHandle<hrc_interfaces::action::MoveElevators>::WrappedResult getResult()
    {
        std::cout << "Getting async result" << std::endl;
        auto future = client_->async_get_result(goal_handle_);
        rclcpp::spin_until_future_complete(node_lifecycle_, future, 1000ms);
        return future.get();
    }

    void sendTransform(const std::string id, const double position) const
    {
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster =
            std::make_shared<tf2_ros::TransformBroadcaster>(node_lifecycle_);
        geometry_msgs::msg::TransformStamped transform;
        transform.transform.rotation.x = 0.0;
        transform.transform.rotation.y = 0.0;
        transform.transform.rotation.z = 0.0;
        transform.transform.rotation.w = 1.0;

        transform.transform.translation.x = 0.0;
        transform.transform.translation.y = 0.0;
        transform.transform.translation.z = position;

        const rclcpp::Duration ahead = 0ms;
        const rclcpp::Time stamp = node_lifecycle_->now();
        // Fill TF buffer ahead for future transform usage
        for (rclcpp::Time t = stamp ; t <= stamp + ahead ;
            t += rclcpp::Duration(50ms))
        {
            transform.header.stamp = t;
            transform.header.frame_id = "elevator_support_" + id + "_link";
            transform.child_frame_id = "elevator_" + id + "_link";
            tf_broadcaster->sendTransform(transform);
        }
    }

    void handleFeedback(
        rclcpp_action::ClientGoalHandle<hrc_interfaces::action::MoveElevators>::SharedPtr,
        const std::shared_ptr<const hrc_interfaces::action::MoveElevators::Feedback> feedback)
    {
        elevator_poses_ = feedback->current_poses;
    }

    bool waitFeedback(const std::chrono::nanoseconds & timeout)
    {
        rclcpp::Time start_time = node_lifecycle_->now();
        while (rclcpp::ok() && node_lifecycle_->now() - start_time <= rclcpp::Duration(timeout)) {
            if (elevator_poses_.size()) {
                return true;
            }
            rclcpp::spin_some(node_lifecycle_->get_node_base_interface());
            std::this_thread::sleep_for(10ms);
        }
        return false;
    }

    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_lifecycle_;
    std::shared_ptr<hrc_behaviors::MoveElevator> behavior_;
    std::shared_ptr<rclcpp_action::Client<hrc_interfaces::action::MoveElevators>> client_;
    std::shared_ptr<rclcpp_action::ClientGoalHandle<hrc_interfaces::action::MoveElevators>> goal_handle_;
    rclcpp_action::Client<hrc_interfaces::action::MoveElevators>::SendGoalOptions goal_options_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::vector<double> elevator_poses_;
};

TEST_F(MoveElevatorTest, test_move_elevator)
{
    std::vector<int> elevators_ids = {0, 1, 2};
    std::vector<double> elevators_poses = {0.0, 0.0, 0.0};

    sendTransform("0", 0.0);
    sendTransform("1", 0.0);
    sendTransform("2", 0.0);

    ASSERT_TRUE(sendCommand(elevators_ids, elevators_poses));
    ASSERT_EQ(getOutcome(), nav2_behaviors::Status::SUCCEEDED);
    ASSERT_TRUE(waitFeedback(500ms));
    ASSERT_NEAR(elevator_poses_.at(0), 0.0, 0.0001);
    ASSERT_NEAR(elevator_poses_.at(1), 0.0, 0.0001);
    ASSERT_NEAR(elevator_poses_.at(2), 0.0, 0.0001);

    elevators_poses = {10.0, 10.0, 10.0};
    ASSERT_TRUE(sendCommand(elevators_ids, elevators_poses));
    ASSERT_TRUE(waitFeedback(500ms));
    ASSERT_NEAR(elevator_poses_.at(0), 0.0, 0.0001);
    ASSERT_NEAR(elevator_poses_.at(1), 0.0, 0.0001);
    ASSERT_NEAR(elevator_poses_.at(2), 0.0, 0.0001);

    sendTransform("0", 10.0);
    sendTransform("1", 10.0);
    sendTransform("2", 10.0);
    elevator_poses_.clear();
    ASSERT_TRUE(waitFeedback(500ms));
    ASSERT_NEAR(elevator_poses_.at(0), 10.0, 0.0001);
    ASSERT_NEAR(elevator_poses_.at(1), 10.0, 0.0001);
    ASSERT_NEAR(elevator_poses_.at(2), 10.0, 0.0001);
    ASSERT_EQ(getOutcome(), nav2_behaviors::Status::SUCCEEDED);
}

TEST_F(MoveElevatorTest, test_timeout)
{
    std::vector<int> elevators_ids = {0};
    std::vector<double> elevators_poses = {0.0};

    sendTransform("0", 1.0);

    ASSERT_TRUE(sendCommand(elevators_ids, elevators_poses, 0.2));
    ASSERT_TRUE(waitFeedback(500ms));
    ASSERT_NEAR(elevator_poses_.at(0), 1.0, 0.0001);
    ASSERT_EQ(getOutcome(), nav2_behaviors::Status::FAILED);
}

TEST_F(MoveElevatorTest, testOneElevatorFail)
{
    std::vector<int> elevators_ids = {0, 1};
    std::vector<double> elevators_poses = {0.0, 0.0};

    sendTransform("0", 0.0);
    sendTransform("1", 1.0);

    ASSERT_TRUE(sendCommand(elevators_ids, elevators_poses, 0.2));
    ASSERT_TRUE(waitFeedback(500ms));
    ASSERT_NEAR(elevator_poses_.at(0), 0.0, 0.0001);
    ASSERT_NEAR(elevator_poses_.at(1), 1.0, 0.0001);
    ASSERT_EQ(getOutcome(), nav2_behaviors::Status::FAILED);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    // initialize ROS
    rclcpp::init(argc, argv);

    bool all_successful = RUN_ALL_TESTS();

    // shutdown ROS
    rclcpp::shutdown();

    return all_successful;
}
