#ifndef NAVIGATOR_PREEMPTION_HPP
#define NAVIGATOR_PREEMPTION_HPP

#include "nav2_core/behavior_tree_navigator.hpp"
#include "hrc_interfaces/action/preempt.hpp"

namespace hrc_bt_navigator
{

class NavigatorPreemption : public nav2_core::BehaviorTreeNavigator<hrc_interfaces::action::Preempt>
{
public:
    using ActionT = hrc_interfaces::action::Preempt;

    NavigatorPreemption() : BehaviorTreeNavigator() {}

    /**
     * @brief Get action name for this navigator
     * @return string Name of action server
     */
    std::string getName() override {return std::string("preemption_navigator");}

    /**
     * @brief Get navigator's default BT
     * @param node WeakPtr to the lifecycle node
     * @return string Filepath to default XML
     */
    std::string getDefaultBTFilepath(rclcpp_lifecycle::LifecycleNode::WeakPtr node) override;

protected:
    /**
     * @brief A callback to be called when a new goal is received by the BT action server
     * Can be used to check if goal is valid and put values on
     * the blackboard which depend on the received goal
     * @param goal Action template's goal message
     * @return bool if goal was received successfully to be processed
     */
    bool goalReceived(ActionT::Goal::ConstSharedPtr goal) override;

    /**
     * @brief A callback that defines execution that happens on one iteration through the BT
     * Can be used to publish action feedback
     */
    void onLoop() override;

    /**
     * @brief A callback that is called when a preempt is requested
     */
    void onPreempt(ActionT::Goal::ConstSharedPtr goal) override;

    /**
     * @brief A callback that is called when a the action is completed, can fill in
     * action result message or indicate that this action is done.
     * @param result Action template result message to populate
     * @param final_bt_status Resulting status of the behavior tree execution that may be
     * referenced while populating the result.
     */
    void goalCompleted(
        typename ActionT::Result::SharedPtr result,
        const nav2_behavior_tree::BtStatus final_bt_status
    ) override;

};

}

#endif // NAVIGATOR_PREEMPTION_HPP
