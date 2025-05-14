#include "gtest/gtest.h"
#include "herminebot_behaviors/behavior/omni_drive.hpp"
#include "hrc_utils/testing_utils.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

class OmniDriveTest : public ::testing::Test
{
protected:
    OmniDriveTest() {}
    void SetUp() override
    {
        node_lifecycle_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
            "test_omni_drive_node", rclcpp::NodeOptions());

        node_lifecycle_->declare_parameter(
            "costmap_topic",
            rclcpp::ParameterValue(std::string("local_costmap/costmap_raw")));
        node_lifecycle_->declare_parameter(
            "footprint_topic",
            rclcpp::ParameterValue(std::string("local_costmap/published_footprint")));
        node_lifecycle_->declare_parameter("local_frame", rclcpp::ParameterValue("odom"));
        node_lifecycle_->declare_parameter("global_frame", rclcpp::ParameterValue("odom"));
        node_lifecycle_->declare_parameter("robot_base_frame", rclcpp::ParameterValue("base_link"));

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

        behavior_ = std::make_shared<hrc_behaviors::OmniDrive>();
        behavior_->configure(
            node_lifecycle_, "test_omni_drive_behavior", tf_buffer_, collision_checker_, collision_checker_);
        behavior_->activate();

        client_ = rclcpp_action::create_client<hrc_interfaces::action::OmniDrive>(
            node_lifecycle_->get_node_base_interface(),
            node_lifecycle_->get_node_graph_interface(),
            node_lifecycle_->get_node_logging_interface(),
            node_lifecycle_->get_node_waitables_interface(),
            "test_omni_drive_behavior");

        goal_options_ = rclcpp_action::Client<hrc_interfaces::action::OmniDrive>::SendGoalOptions();
        goal_options_.feedback_callback = std::bind(
            &OmniDriveTest::handleFeedback, this, std::placeholders::_1, std::placeholders::_2);

        velocity_sub_ = node_lifecycle_->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 1, std::bind(&OmniDriveTest::handleVelocity, this, std::placeholders::_1));

        std::cout << "Setup complete" << std::endl;
    }

    void TearDown() override {}

    bool sendCommand(
        const geometry_msgs::msg::Point& target,
        const double speed,
        const double time_allowance = 0.0)
    {
        if (!client_->wait_for_action_server(1s)) {
            std::cout << "Server not up" << std::endl;
            return false;
        }

        auto goal = hrc_interfaces::action::OmniDrive::Goal();
        goal.target = target;
        goal.speed = speed;
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

        travelled_distance_ = -1.0;
        velocity_received_ = false;

        return true;
    }

    nav2_behaviors::Status getOutcome()
    {
        if (getResult().code == rclcpp_action::ResultCode::SUCCEEDED) {
            return nav2_behaviors::Status::SUCCEEDED;
        }

        return nav2_behaviors::Status::FAILED;
    }

    rclcpp_action::ClientGoalHandle<hrc_interfaces::action::OmniDrive>::WrappedResult getResult()
    {
        std::cout << "Getting async result" << std::endl;
        auto future = client_->async_get_result(goal_handle_);
        rclcpp::spin_until_future_complete(node_lifecycle_, future, 1000ms);
        return future.get();
    }

    void sendTransform(const geometry_msgs::msg::Pose& robot_pose) const
    {
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster =
            std::make_shared<tf2_ros::TransformBroadcaster>(node_lifecycle_);
        geometry_msgs::msg::TransformStamped transform;
        transform.transform.rotation.z = robot_pose.orientation.z;
        transform.transform.rotation.w = robot_pose.orientation.w;

        transform.transform.translation.x = robot_pose.position.x;
        transform.transform.translation.y = robot_pose.position.y;

        transform.header.frame_id = "odom";
        transform.child_frame_id = "base_link";

        const rclcpp::Duration ahead = 100ms;
        const rclcpp::Time stamp = node_lifecycle_->now();
        // Fill TF buffer ahead for future transform usage
        for (rclcpp::Time t = stamp ; t <= stamp + ahead ; t += rclcpp::Duration(50ms)) {
            transform.header.stamp = t;
            tf_broadcaster->sendTransform(transform);
        }
    }

    void handleFeedback(
        rclcpp_action::ClientGoalHandle<hrc_interfaces::action::OmniDrive>::SharedPtr,
        const std::shared_ptr<const hrc_interfaces::action::OmniDrive::Feedback> feedback)
    {
        travelled_distance_ = feedback->distance_traveled;
    }

    void handleVelocity(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        velocity_ = *msg;
        velocity_received_ = true;
    }

    bool waitFeedback(const std::chrono::nanoseconds & timeout, double expected_distance)
    {
        rclcpp::Time start_time = node_lifecycle_->now();
        while (rclcpp::ok() && node_lifecycle_->now() - start_time <= rclcpp::Duration(timeout)) {
            if (fabs(travelled_distance_ - expected_distance) < 1e-4) {
                return true;
            }
            rclcpp::spin_some(node_lifecycle_->get_node_base_interface());
            std::this_thread::sleep_for(10ms);
        }
        return false;
    }

    bool waitVelocity(const std::chrono::nanoseconds & timeout)
    {
        rclcpp::Time start_time = node_lifecycle_->now();
        while (rclcpp::ok() && node_lifecycle_->now() - start_time <= rclcpp::Duration(timeout)) {
            if (velocity_received_) {
                return true;
            }
            rclcpp::spin_some(node_lifecycle_->get_node_base_interface());
            std::this_thread::sleep_for(10ms);
        }
        return false;
    }

    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_lifecycle_;
    std::shared_ptr<hrc_behaviors::OmniDrive> behavior_;
    std::shared_ptr<rclcpp_action::Client<hrc_interfaces::action::OmniDrive>> client_;
    std::shared_ptr<rclcpp_action::ClientGoalHandle<hrc_interfaces::action::OmniDrive>> goal_handle_;
    rclcpp_action::Client<hrc_interfaces::action::OmniDrive>::SendGoalOptions goal_options_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    double travelled_distance_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_sub_;
    geometry_msgs::msg::Twist velocity_;
    bool velocity_received_;
};

TEST_F(OmniDriveTest, test_omni_drive)
{
    // No movement
    geometry_msgs::msg::Point target;
    target.x = 0.0;
    target.y = 0.0;

    geometry_msgs::msg::Pose robot_pose;
    robot_pose.position.x = 0.0;
    robot_pose.position.y = 0.0;

    sendTransform(robot_pose);

    ASSERT_TRUE(sendCommand(target, 1.0));
    ASSERT_TRUE(waitFeedback(500ms, 0.0));
    ASSERT_TRUE(waitVelocity(500ms));
    ASSERT_EQ(getOutcome(), nav2_behaviors::Status::SUCCEEDED);
    ASSERT_NEAR(velocity_.linear.x, 0.0, HRC_UTILS__TESTING_FLOAT_ASSERTION_PRECISION);
    ASSERT_NEAR(velocity_.linear.y, 0.0, HRC_UTILS__TESTING_FLOAT_ASSERTION_PRECISION);

    // Move diagoanlly
    target.x = 1.0;
    target.y = 1.0;

    ASSERT_TRUE(sendCommand(target, 1.0));
    ASSERT_TRUE(waitFeedback(500ms, 0.0));
    ASSERT_TRUE(waitVelocity(500ms));
    ASSERT_NEAR(velocity_.linear.x, cos(M_PI_4), HRC_UTILS__TESTING_FLOAT_ASSERTION_PRECISION);
    ASSERT_NEAR(velocity_.linear.y, sin(M_PI_4), HRC_UTILS__TESTING_FLOAT_ASSERTION_PRECISION);

    robot_pose.position.x = 1.0;
    robot_pose.position.y = 1.0;
    sendTransform(robot_pose);
    velocity_received_ = false;
    ASSERT_TRUE(waitFeedback(500ms, hypot(1.0, 1.0)));
    ASSERT_EQ(getOutcome(), nav2_behaviors::Status::SUCCEEDED);
    ASSERT_TRUE(waitVelocity(500ms));
    ASSERT_NEAR(velocity_.linear.x, 0.0, HRC_UTILS__TESTING_FLOAT_ASSERTION_PRECISION);
    ASSERT_NEAR(velocity_.linear.y, 0.0, HRC_UTILS__TESTING_FLOAT_ASSERTION_PRECISION);
}

TEST_F(OmniDriveTest, test_timeout)
{
    geometry_msgs::msg::Point target;
    target.x = 1.0;
    target.y = 1.0;

    geometry_msgs::msg::Pose robot_pose;
    robot_pose.position.x = 0.0;
    robot_pose.position.y = 0.0;

    sendTransform(robot_pose);

    ASSERT_TRUE(sendCommand(target, 1.0, 0.2));
    ASSERT_TRUE(waitFeedback(500ms, 0.0));
    ASSERT_EQ(getOutcome(), nav2_behaviors::Status::FAILED);
    ASSERT_TRUE(waitVelocity(500ms));
    ASSERT_NEAR(velocity_.linear.x, 0.0, HRC_UTILS__TESTING_FLOAT_ASSERTION_PRECISION);
    ASSERT_NEAR(velocity_.linear.y, 0.0, HRC_UTILS__TESTING_FLOAT_ASSERTION_PRECISION);
}

TEST_F(OmniDriveTest, testFails)
{
    // test fail for target.z != 0.0
    geometry_msgs::msg::Point target;
    target.x = 1.0;
    target.y = 1.0;
    target.z = 1.0;

    ASSERT_TRUE(sendCommand(target, 1.0));
    ASSERT_EQ(getOutcome(), nav2_behaviors::Status::FAILED);

    // test fail for no transform
    target.z = 0.0;
    ASSERT_TRUE(sendCommand(target, 1.0));
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
