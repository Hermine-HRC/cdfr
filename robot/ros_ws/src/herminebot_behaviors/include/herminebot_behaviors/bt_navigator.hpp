#ifndef HRC_BT_NAVIGATOR
#define HRC_BT_NAVIGATOR

#include "nav2_bt_navigator/bt_navigator.hpp"
#include "hrc_interfaces/action/preempt.hpp"

namespace hrc_bt_navigator
{

class BtNavigator : public nav2_bt_navigator::BtNavigator
{
public:
    explicit BtNavigator(rclcpp::NodeOptions options = rclcpp::NodeOptions());

protected:
    /**
     * @brief Configures member variables
     *
     * Initializes action server for "NavigationToPose"; subscription to
     * "goal_sub"; and builds behavior tree from xml file.
     * @param state Reference to LifeCycle node state
     * @return SUCCESS or FAILURE
     */
    nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
    /**
     * @brief Activates action server
     * @param state Reference to LifeCycle node state
     * @return SUCCESS or FAILURE
     */
    nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
    /**
     * @brief Deactivates action server
     * @param state Reference to LifeCycle node state
     * @return SUCCESS or FAILURE
     */
    nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
    /**
     * @brief Resets member variables
     * @param state Reference to LifeCycle node state
     * @return SUCCESS or FAILURE
     */
    nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

    std::unique_ptr<nav2_bt_navigator::Navigator<hrc_interfaces::action::Preempt>> preempt_navigator_;
};

}

#endif // HRC_BT_NAVIGATOR
