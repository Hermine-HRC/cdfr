#include "gtest/gtest.h"
#include <memory>
#include "herminebot_behaviors/bt_plugin/move_elevator_action.hpp"
#include "rclcpp/rclcpp.hpp"
#include "test_action_server.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"

class MoveElevatorActionServer : public TestActionServer<hrc_interfaces::action::MoveElevators>
{
public:
    MoveElevatorActionServer() : TestActionServer("move_elevators") {}

protected:
    void execute(
        const typename std::shared_ptr<rclcpp_action::ServerGoalHandle<hrc_interfaces::action::MoveElevators>> goal_handle)
    {
        auto result =
            std::make_shared<rclcpp_action::ClientGoalHandle<hrc_interfaces::action::MoveElevators>::Result>();
        bool return_success = getReturnSuccess();
        if (return_success) {
            goal_handle->succeed(result);
        }
        else {
            goal_handle->abort(result);
        }
    }
};

class MoveElevatorActionTestFixture : public ::testing::Test
{
public:
    static void SetUpTestCase()
    {
        node_ = std::make_shared<rclcpp::Node>("move_elevator_action_test_fixture");
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

        BT::NodeBuilder builder =
            [](const std::string & name, const BT::NodeConfiguration & config)
            {
                return std::make_unique<hrc_behavior_tree::MoveElevatorAction>(
                    name, "move_elevators", config);
            };

        factory_->registerBuilder<hrc_behavior_tree::MoveElevatorAction>("MoveElevators", builder);
    }

    static void TearDownTestCase()
    {
        delete config_;
        config_ = nullptr;
        node_.reset();
        action_server_.reset();
        factory_.reset();
    }

    void TearDown() override
    {
        tree_.reset();
    }

    static std::shared_ptr<MoveElevatorActionServer> action_server_;

protected:
    static rclcpp::Node::SharedPtr node_;
    static BT::NodeConfiguration * config_;
    static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
    static std::shared_ptr<BT::Tree> tree_;
};


rclcpp::Node::SharedPtr MoveElevatorActionTestFixture::node_ = nullptr;
std::shared_ptr<MoveElevatorActionServer> MoveElevatorActionTestFixture::action_server_ = nullptr;
BT::NodeConfiguration* MoveElevatorActionTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> MoveElevatorActionTestFixture::factory_ = nullptr;
std::shared_ptr<BT::Tree> MoveElevatorActionTestFixture::tree_ = nullptr;

TEST_F(MoveElevatorActionTestFixture, test_ports)
{
    // Default values
    std::string xml_txt =
        R"(
        <root main_tree_to_execute = "MainTree" >
            <BehaviorTree ID="MainTree">
                <MoveElevators />
            </BehaviorTree>
        </root>)";

    tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));
    EXPECT_EQ(tree_->rootNode()->getInput<double>("time_allowance").value(), 4.0);
    std::vector<int> ids;
    tree_->rootNode()->getInput<std::vector<int>>("elevators_ids", ids);
    EXPECT_EQ(ids.size(), 0);
    std::vector<double> poses;
    tree_->rootNode()->getInput<std::vector<double>>("elevators_poses", poses);
    EXPECT_EQ(poses.size(), 0);

    // Custom values
    xml_txt =
        R"(
        <root main_tree_to_execute = "MainTree" >
            <BehaviorTree ID="MainTree">
                <MoveElevators time_allowance="2.5" elevators_ids="0; 1" elevators_poses="0.02; 0.03"/>
            </BehaviorTree>
        </root>)";

    tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));
    EXPECT_EQ(tree_->rootNode()->getInput<double>("time_allowance").value(), 2.5);

    tree_->rootNode()->getInput<std::vector<int>>("elevators_ids", ids);
    EXPECT_EQ(ids.size(), 2);
    EXPECT_EQ(ids[0], 0);
    EXPECT_EQ(ids[1], 1);

    tree_->rootNode()->getInput<std::vector<double>>("elevators_poses", poses);
    EXPECT_EQ(poses.size(), 2);
    EXPECT_EQ(poses[0], 0.02);
    EXPECT_EQ(poses[1], 0.03);
}

TEST_F(MoveElevatorActionTestFixture, test_running)
{
    std::string xml_txt =
        R"(
        <root main_tree_to_execute = "MainTree" >
            <BehaviorTree ID="MainTree">
                <MoveElevators time_allowance="2.5" elevators_ids="0; 1" elevators_poses="0.02; 0.03"/>
            </BehaviorTree>
        </root>)";

    tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

    while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS) {
        tree_->rootNode()->executeTick();
    }

    EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::SUCCESS);

    auto goal = action_server_->getCurrentGoal();
    EXPECT_EQ(goal->elevators_ids.size(), 2);
    EXPECT_EQ(goal->elevators_poses.at(0), 0.02);
    EXPECT_EQ(goal->elevators_poses.at(1), 0.03);

    EXPECT_EQ(goal->elevators_poses.size(), 2);
    EXPECT_EQ(goal->elevators_ids.at(0), 0);
    EXPECT_EQ(goal->elevators_ids.at(1), 1);

    EXPECT_EQ(goal->time_allowance.sec, 2);
    EXPECT_EQ(goal->time_allowance.nanosec, 500000000);
}

TEST_F(MoveElevatorActionTestFixture, test_failure)
{
    std::string xml_txt =
        R"(
        <root main_tree_to_execute = "MainTree" >
            <BehaviorTree ID="MainTree">
                <MoveElevators time_allowance="2.5" elevators_ids="0; 1" elevators_poses="0.02; 0.03"/>
            </BehaviorTree>
        </root>)";

    tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));
    action_server_->setReturnSuccess(false);

    while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS &&
        tree_->rootNode()->status() != BT::NodeStatus::FAILURE)
    {
        tree_->rootNode()->executeTick();
    }

    EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::FAILURE);

    auto goal = action_server_->getCurrentGoal();
    EXPECT_EQ(goal->elevators_ids.size(), 2);
    EXPECT_EQ(goal->elevators_poses.at(0), 0.02);
    EXPECT_EQ(goal->elevators_poses.at(1), 0.03);

    EXPECT_EQ(goal->elevators_poses.size(), 2);
    EXPECT_EQ(goal->elevators_ids.at(0), 0);
    EXPECT_EQ(goal->elevators_ids.at(1), 1);

    EXPECT_EQ(goal->time_allowance.sec, 2);
    EXPECT_EQ(goal->time_allowance.nanosec, 500000000);
}

int main(int argc, char ** argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    // initialize ROS
    rclcpp::init(argc, argv);

    // initialize action server and spin on new thread
    MoveElevatorActionTestFixture::action_server_ = std::make_shared<MoveElevatorActionServer>();
    std::thread server_thread([]() {
            rclcpp::spin(MoveElevatorActionTestFixture::action_server_);
        });

    int all_successful = RUN_ALL_TESTS();

    // shutdown ROS
    rclcpp::shutdown();
    server_thread.join();

    return all_successful;
}
