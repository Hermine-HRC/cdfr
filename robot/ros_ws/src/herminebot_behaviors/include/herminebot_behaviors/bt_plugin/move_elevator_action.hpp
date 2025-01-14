#ifndef MOVE_ELEVATOR_ACTION_HPP
#define MOVE_ELEVATOR_ACTION_HPP

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "hrc_interfaces/action/move_elevators.hpp"
#include <vector>

namespace hrc_behavior_tree
{

class MoveElevatorAction : public nav2_behavior_tree::BtActionNode<hrc_interfaces::action::MoveElevators>
{
public:
    /**
     * @brief A constructor for hrc_behavior_tree::MoveElevatorAction
     * @param xml_tag_name Name for the XML tag for this node
     * @param action_name Action name this node creates a client for
     * @param conf BT node configuration
     */
    MoveElevatorAction(
        const std::string& xml_tag_name,
        const std::string& action_name,
        const BT::NodeConfiguration& conf
    );

    /**
     * @brief Creates list of BT ports
     * @return BT::PortsList Containing basic ports along with node-specific ports
     */
    static BT::PortsList providedPorts()
    {
        return providedBasicPorts(
            {
                BT::InputPort<double>("time_allowance", 4.0, "Allowed time for moving the elevators"),
                BT::InputPort<std::vector<int>>("elevators_ids", "Id of the elevators to move"),
                BT::InputPort<std::vector<double>>("elevators_poses", "Wanted position of the elevators")
            });
    }

};

}

#endif // MOVE_ELEVATOR_ACTION_HPP
