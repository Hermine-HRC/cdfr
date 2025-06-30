#include "gtest/gtest.h"
#include <memory>
#include "herminebot_behaviors/bt_plugin/manage_map_service.hpp"
#include "hrc_utils/utils.hpp"
#include "hrc_utils/testing_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "hrc_utils/test_service.hpp"
#include "behaviortree_cpp/bt_factory.h"

#define ROBOT_POSE_X 1.0
#define ROBOT_POSE_Y 0.5
#define ROBOT_POSE_THETA M_PI

class ManageMapService : public hrc_utils::TestService<hrc_interfaces::srv::ManageObjectsMap>
{
public:
    ManageMapService() : TestService("manage_object_map") {}
};

class ManageMapTestFixture : public ::testing::Test
{
public:
    static void SetUpTestCase()
    {
        node_ = std::make_shared<rclcpp::Node>("manage_map_test_fixture");
        factory_ = std::make_shared<BT::BehaviorTreeFactory>();
        config_ = new BT::NodeConfiguration();

        config_->blackboard = BT::Blackboard::create();
        config_->blackboard->set("node", node_);
        config_->blackboard->set<std::chrono::milliseconds>(
            "server_timeout",
            std::chrono::milliseconds(20));
        config_->blackboard->set<std::chrono::milliseconds>(
            "bt_loop_duration",
            std::chrono::milliseconds(10));
        config_->blackboard->set<std::chrono::milliseconds>(
            "wait_for_service_timeout",
            std::chrono::milliseconds(1000));

        factory_->registerNodeType<hrc_behavior_tree::ManageMapService>("ManageMap");
    }

    static void TearDownTestCase()
    {
        delete config_;
        config_ = nullptr;
        node_.reset();
        server_.reset();
        factory_.reset();
    }

    void TearDown() override
    {
        tree_.reset();
    }

    static std::shared_ptr<ManageMapService> server_;

protected:
    static rclcpp::Node::SharedPtr node_;
    static BT::NodeConfiguration* config_;
    static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
    static std::shared_ptr<BT::Tree> tree_;
};


rclcpp::Node::SharedPtr ManageMapTestFixture::node_ = nullptr;
std::shared_ptr<ManageMapService> ManageMapTestFixture::server_ = nullptr;
BT::NodeConfiguration* ManageMapTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> ManageMapTestFixture::factory_ = nullptr;
std::shared_ptr<BT::Tree> ManageMapTestFixture::tree_ = nullptr;

TEST_F(ManageMapTestFixture, test_ports)
{
    // Default values
    std::string xml_txt =
        R"(
        <root main_tree_to_execute = "MainTree" BTCPP_format="4" >
            <BehaviorTree ID="MainTree">
                <ManageMap />
            </BehaviorTree>
        </root>)";

    tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));
    EXPECT_EQ(tree_->rootNode()->getInput<bool>("is_robot_relative").value(), false);
    std::vector<geometry_msgs::msg::Point> points_objects_to_remove;
    tree_->rootNode()->getInput<std::vector<geometry_msgs::msg::Point>>(
        "points_objects_to_remove", points_objects_to_remove);
    EXPECT_EQ(points_objects_to_remove.size(), 0);
    std::vector<std::vector<geometry_msgs::msg::Point>> new_objects;
    tree_->rootNode()->getInput<std::vector<std::vector<geometry_msgs::msg::Point>>>("new_objects", new_objects);
    EXPECT_EQ(new_objects.size(), 0);

    // Custom values
    xml_txt =
        R"(
        <root main_tree_to_execute = "MainTree" BTCPP_format="4" >
            <BehaviorTree ID="MainTree">
                <ManageMap is_robot_relative="true" 
                points_objects_to_remove="[[0.15, 0.0], [0.1, 0.1]]" 
                new_objects="
                    [[-0.5, -0.2], [-0.5, 0.2], [-0.4, 0.2], [-0.4, -0.2]];
                    [[-0.9, -0.2], [-0.9, 0.2], [-0.7, 0.2], [-0.7, -0.2]]"/>
            </BehaviorTree>
        </root>)";

    tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));
    EXPECT_EQ(tree_->rootNode()->getInput<bool>("is_robot_relative").value(), true);

    tree_->rootNode()->getInput<std::vector<geometry_msgs::msg::Point>>(
        "points_objects_to_remove", points_objects_to_remove);
    EXPECT_EQ(points_objects_to_remove.size(), 2);
    EXPECT_NEAR(points_objects_to_remove[0].x, 0.15, HRC_UTILS__TESTING_FLOAT_ASSERTION_PRECISION);
    EXPECT_NEAR(points_objects_to_remove[0].y, 0.0, HRC_UTILS__TESTING_FLOAT_ASSERTION_PRECISION);
    EXPECT_NEAR(points_objects_to_remove[1].x, 0.1, HRC_UTILS__TESTING_FLOAT_ASSERTION_PRECISION);
    EXPECT_NEAR(points_objects_to_remove[1].y, 0.1, HRC_UTILS__TESTING_FLOAT_ASSERTION_PRECISION);

    tree_->rootNode()->getInput<std::vector<std::vector<geometry_msgs::msg::Point>>>("new_objects", new_objects);
    EXPECT_EQ(new_objects.size(), 2);
    EXPECT_NEAR(new_objects[0].at(0).x, -0.5, HRC_UTILS__TESTING_FLOAT_ASSERTION_PRECISION);
    EXPECT_NEAR(new_objects[0].at(0).y, -0.2, HRC_UTILS__TESTING_FLOAT_ASSERTION_PRECISION);
    EXPECT_NEAR(new_objects[0].at(1).x, -0.5, HRC_UTILS__TESTING_FLOAT_ASSERTION_PRECISION);
    EXPECT_NEAR(new_objects[0].at(1).y, 0.2, HRC_UTILS__TESTING_FLOAT_ASSERTION_PRECISION);
    EXPECT_NEAR(new_objects[0].at(2).x, -0.4, HRC_UTILS__TESTING_FLOAT_ASSERTION_PRECISION);
    EXPECT_NEAR(new_objects[0].at(2).y, 0.2, HRC_UTILS__TESTING_FLOAT_ASSERTION_PRECISION);
    EXPECT_NEAR(new_objects[0].at(3).x, -0.4, HRC_UTILS__TESTING_FLOAT_ASSERTION_PRECISION);
    EXPECT_NEAR(new_objects[0].at(3).y, -0.2, HRC_UTILS__TESTING_FLOAT_ASSERTION_PRECISION);
    EXPECT_NEAR(new_objects[1].at(0).x, -0.9, HRC_UTILS__TESTING_FLOAT_ASSERTION_PRECISION);
    EXPECT_NEAR(new_objects[1].at(0).y, -0.2, HRC_UTILS__TESTING_FLOAT_ASSERTION_PRECISION);
    EXPECT_NEAR(new_objects[1].at(1).x, -0.9, HRC_UTILS__TESTING_FLOAT_ASSERTION_PRECISION);
    EXPECT_NEAR(new_objects[1].at(1).y, 0.2, HRC_UTILS__TESTING_FLOAT_ASSERTION_PRECISION);
    EXPECT_NEAR(new_objects[1].at(2).x, -0.7, HRC_UTILS__TESTING_FLOAT_ASSERTION_PRECISION);
    EXPECT_NEAR(new_objects[1].at(2).y, 0.2, HRC_UTILS__TESTING_FLOAT_ASSERTION_PRECISION);
    EXPECT_NEAR(new_objects[1].at(3).x, -0.7, HRC_UTILS__TESTING_FLOAT_ASSERTION_PRECISION);
    EXPECT_NEAR(new_objects[1].at(3).y, -0.2, HRC_UTILS__TESTING_FLOAT_ASSERTION_PRECISION);
}

TEST_F(ManageMapTestFixture, test_running)
{
    std::string xml_txt =
        R"(
        <root main_tree_to_execute = "MainTree" BTCPP_format="4" >
            <BehaviorTree ID="MainTree">
                <ManageMap is_robot_relative="false"
                points_objects_to_remove="[[0.15, 0.0], [0.1, 0.1]]"
                new_objects="
                    [[-0.5, -0.2], [-0.5, 0.2], [-0.4, 0.2], [-0.4, -0.2]];
                    [[-0.9, -0.2], [-0.9, 0.2], [-0.7, 0.2], [-0.7, -0.2]]"/>
            </BehaviorTree>
        </root>)";

    tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));
    while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS) {
        tree_->rootNode()->executeTick();
    }

    EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::SUCCESS);

    auto req = server_->getCurrentRequest();

    EXPECT_EQ(tree_->rootNode()->getInput<bool>("is_robot_relative").value(), false);

    EXPECT_EQ(req->points_objects_to_remove.size(), 2);
    ASSERT_NEAR(req->points_objects_to_remove[0].x, 0.15, HRC_UTILS__TESTING_FLOAT_ASSERTION_PRECISION);
    ASSERT_NEAR(req->points_objects_to_remove[0].y, 0.0, HRC_UTILS__TESTING_FLOAT_ASSERTION_PRECISION);
    ASSERT_NEAR(req->points_objects_to_remove[1].x, 0.1, HRC_UTILS__TESTING_FLOAT_ASSERTION_PRECISION);
    ASSERT_NEAR(req->points_objects_to_remove[1].y, 0.1, HRC_UTILS__TESTING_FLOAT_ASSERTION_PRECISION);

    EXPECT_EQ(req->new_objects.size(), 2);
    ASSERT_NEAR(req->new_objects[0].points.at(0).x, -0.5, HRC_UTILS__TESTING_FLOAT_ASSERTION_PRECISION);
    ASSERT_NEAR(req->new_objects[0].points.at(0).y, -0.2, HRC_UTILS__TESTING_FLOAT_ASSERTION_PRECISION);
    ASSERT_NEAR(req->new_objects[0].points.at(1).x, -0.5, HRC_UTILS__TESTING_FLOAT_ASSERTION_PRECISION);
    ASSERT_NEAR(req->new_objects[0].points.at(1).y, 0.2, HRC_UTILS__TESTING_FLOAT_ASSERTION_PRECISION);
    ASSERT_NEAR(req->new_objects[0].points.at(2).x, -0.4, HRC_UTILS__TESTING_FLOAT_ASSERTION_PRECISION);
    ASSERT_NEAR(req->new_objects[0].points.at(2).y, 0.2, HRC_UTILS__TESTING_FLOAT_ASSERTION_PRECISION);
    ASSERT_NEAR(req->new_objects[0].points.at(3).x, -0.4, HRC_UTILS__TESTING_FLOAT_ASSERTION_PRECISION);
    ASSERT_NEAR(req->new_objects[0].points.at(3).y, -0.2, HRC_UTILS__TESTING_FLOAT_ASSERTION_PRECISION);
    ASSERT_NEAR(req->new_objects[1].points.at(0).x, -0.9, HRC_UTILS__TESTING_FLOAT_ASSERTION_PRECISION);
    ASSERT_NEAR(req->new_objects[1].points.at(0).y, -0.2, HRC_UTILS__TESTING_FLOAT_ASSERTION_PRECISION);
    ASSERT_NEAR(req->new_objects[1].points.at(1).x, -0.9, HRC_UTILS__TESTING_FLOAT_ASSERTION_PRECISION);
    ASSERT_NEAR(req->new_objects[1].points.at(1).y, 0.2, HRC_UTILS__TESTING_FLOAT_ASSERTION_PRECISION);
    ASSERT_NEAR(req->new_objects[1].points.at(2).x, -0.7, HRC_UTILS__TESTING_FLOAT_ASSERTION_PRECISION);
    ASSERT_NEAR(req->new_objects[1].points.at(2).y, 0.2, HRC_UTILS__TESTING_FLOAT_ASSERTION_PRECISION);
    ASSERT_NEAR(req->new_objects[1].points.at(3).x, -0.7, HRC_UTILS__TESTING_FLOAT_ASSERTION_PRECISION);
    ASSERT_NEAR(req->new_objects[1].points.at(3).y, -0.2, HRC_UTILS__TESTING_FLOAT_ASSERTION_PRECISION);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    // initialize ROS
    rclcpp::init(argc, argv);

    // initialize action server and spin on new thread
    ManageMapTestFixture::server_ = std::make_shared<ManageMapService>();
    std::thread server_thread([]() {
            rclcpp::spin(ManageMapTestFixture::server_);
        });

    int all_successful = RUN_ALL_TESTS();

    // shutdown ROS
    rclcpp::shutdown();
    server_thread.join();

    return all_successful;
}
