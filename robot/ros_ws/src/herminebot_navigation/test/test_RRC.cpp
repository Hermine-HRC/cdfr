#include "herminebot_navigation/hrc_regulated_rotation_controller.hpp"
#include "gtest/gtest.h"

const double EPSILON = 1e-4;
const double MAX_ROTATION_VEL = 1.5;

class RclCppFixture
{
public:
    RclCppFixture() {rclcpp::init(0, nullptr);}
    ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

class RegulatedRotationControllerTest : public hrc_regulated_rotation_controller::RegulatedRotationController
{
public:
    void RegulatedRotationController() {}

    double getRotationVelocityWrapper(const double & current_orientation, const double & target_orientation)
    {
        return getRotationVelocity(current_orientation, target_orientation);
    }

    nav2_core::Controller::Ptr getPrimaryController()
    {
        return primary_controller_;
    }
};

TEST(RegulatedRotationControllerTest, lifecycleTransition)
{
    auto ctrl = std::make_shared<RegulatedRotationControllerTest>();
    auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_node");
    auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("fake_costmap");
    std::string name = "PathFollower";
    rclcpp_lifecycle::State state;

    costmap->on_configure(state);
    // Should not populate primary controller, does not exist
    ASSERT_THROW(ctrl->configure(node, name, tf, costmap), std::runtime_error);
    ASSERT_EQ(ctrl->getPrimaryController(), nullptr);

    // Add a controller to the setup
    auto rec_param = std::make_shared<rclcpp::AsyncParametersClient>(
        node->get_node_base_interface(), node->get_node_topics_interface(),
        node->get_node_graph_interface(),
        node->get_node_services_interface());
    auto results = rec_param->set_parameters_atomically(
        {rclcpp::Parameter(
                "PathFollower.primary_controller",
                std::string("nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"))});
    rclcpp::spin_until_future_complete(
        node->get_node_base_interface(),
        results);

    ctrl->configure(node, name, tf, costmap);
    ASSERT_NE(ctrl->getPrimaryController(), nullptr);

    ctrl->activate();

    ctrl->setSpeedLimit(50.0, true);

    ctrl->deactivate();
    ctrl->cleanup();
}

TEST(RegulatedRotationControllerTest, testDynamicParameter)
{
    auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_node");
    auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("fake_costmap");
    std::string name = "PathFollower";
    auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    rclcpp_lifecycle::State state;

    costmap->on_configure(state);
    node->declare_parameter(
        "PathFollower.primary_controller",
        "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController");

    auto ctrl = std::make_shared<RegulatedRotationControllerTest>();
    ctrl->configure(node, name, tf, costmap);
    ctrl->activate();

    auto rec_param = std::make_shared<rclcpp::AsyncParametersClient>(
        node->get_node_base_interface(), node->get_node_topics_interface(),
        node->get_node_graph_interface(),
        node->get_node_services_interface());

    auto results = rec_param->set_parameters_atomically(
    {
        rclcpp::Parameter("PathFollower.p_gain", 1.0),
        rclcpp::Parameter("PathFollower.i_gain", 1.0),
        rclcpp::Parameter("PathFollower.d_gain", 1.0),
        rclcpp::Parameter("PathFollower.max_rotation_vel", 1.0)
    });

    rclcpp::spin_until_future_complete(node->get_node_base_interface(), results);
    ASSERT_EQ(node->get_parameter("PathFollower.p_gain").as_double(), 1.0);
    ASSERT_EQ(node->get_parameter("PathFollower.i_gain").as_double(), 1.0);
    ASSERT_EQ(node->get_parameter("PathFollower.d_gain").as_double(), 1.0);
    ASSERT_EQ(node->get_parameter("PathFollower.max_rotation_vel").as_double(), 1.0);
}

TEST(RegulatedRotationControllerTest, testPID)
{
    auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_node");
    auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("fake_costmap");
    std::string name = "PathFollower";
    auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    rclcpp_lifecycle::State state;

    costmap->on_configure(state);
    node->declare_parameter(
        "PathFollower.primary_controller",
        "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController");

    auto ctrl = std::make_shared<RegulatedRotationControllerTest>();
    ctrl->configure(node, name, tf, costmap);
    ctrl->activate();

    nav_msgs::msg::Path path;
    auto last_pose = geometry_msgs::msg::PoseStamped();
    // yaw = PI
    last_pose.pose.orientation.z = 1.0;
    last_pose.pose.orientation.w = 0.0;
    path.poses.push_back(last_pose);
    ctrl->setPlan(path);

    auto robot_pose = geometry_msgs::msg::PoseStamped();
    // yaw = 0
    robot_pose.pose.orientation.z = 0.0;
    robot_pose.pose.orientation.w = 1.0;

    ASSERT_NEAR(
        ctrl->getRotationVelocityWrapper(
            tf2::getYaw(robot_pose.pose.orientation),
            tf2::getYaw(last_pose.pose.orientation)),
        MAX_ROTATION_VEL, EPSILON);

    // Switch positions
    robot_pose.pose.orientation.z = 1.0;
    robot_pose.pose.orientation.w = 0.0;
    last_pose.pose.orientation.z = 0.0;
    last_pose.pose.orientation.w = 1.0;
    path.poses.push_back(last_pose);
    ctrl->setPlan(path);
    ASSERT_NEAR(
        ctrl->getRotationVelocityWrapper(
            tf2::getYaw(robot_pose.pose.orientation),
            tf2::getYaw(last_pose.pose.orientation)),
        -MAX_ROTATION_VEL, EPSILON);

    // Around PI
    robot_pose.pose.orientation.z = 0.997495; // 3 rad
    robot_pose.pose.orientation.w = 0.0707372;
    last_pose.pose.orientation.z = -0.997495; // -3 rad
    last_pose.pose.orientation.w = 0.0707372;
    path.poses.push_back(last_pose);
    ctrl->setPlan(path);
    double rotation_vel = ctrl->getRotationVelocityWrapper(
        tf2::getYaw(robot_pose.pose.orientation),
        tf2::getYaw(last_pose.pose.orientation));
    ASSERT_TRUE(0.2 < rotation_vel && rotation_vel < MAX_ROTATION_VEL);

    // Around PI switched
    robot_pose.pose.orientation.z = -0.997495; // -3 rad
    robot_pose.pose.orientation.w = 0.0707372;
    last_pose.pose.orientation.z = 0.997495; // 3 rad
    last_pose.pose.orientation.w = 0.0707372;
    path.poses.push_back(last_pose);
    ctrl->setPlan(path);
    rotation_vel = ctrl->getRotationVelocityWrapper(
        tf2::getYaw(robot_pose.pose.orientation),
        tf2::getYaw(last_pose.pose.orientation));
    ASSERT_TRUE(-MAX_ROTATION_VEL < rotation_vel && rotation_vel < -0.2);


    // Already at the final orientation
    robot_pose.pose.orientation.z = 0.0;
    robot_pose.pose.orientation.w = 1.0;
    last_pose.pose.orientation.z = 0.0;
    last_pose.pose.orientation.w = 1.0;
    path.poses.push_back(last_pose);
    ctrl->setPlan(path);
    ASSERT_NEAR(
        ctrl->getRotationVelocityWrapper(
            tf2::getYaw(robot_pose.pose.orientation),
            tf2::getYaw(last_pose.pose.orientation)),
        0.0, EPSILON);
}
