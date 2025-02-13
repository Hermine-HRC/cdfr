#ifndef OMNI_DRIVE_ACTION_HPP
#define OMNI_DRIVE_ACTION_HPP

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "hrc_interfaces/action/omni_drive.hpp"

namespace hrc_behavior_tree
{

class OmniDriveAction : public nav2_behavior_tree::BtActionNode<hrc_interfaces::action::OmniDrive>
{
public:
    /**
     * @brief A constructor for hrc_behavior_tree::OmniDriveAction
     * @param xml_tag_name Name for the XML tag for this node
     * @param action_name Action name this node creates a client for
     * @param conf BT node configuration
     */
    OmniDriveAction(
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
                BT::InputPort<double>("target_x", 0.0, "Target position in X-axis"),
                BT::InputPort<double>("target_y", 0.0, "Target position in Y-axis"),
                BT::InputPort<double>("speed", 0.1, "Speed of the movement"),
                BT::InputPort<double>("time_allowance", 4.0, "Allowed time for moving")
            });
    }
};

} // namespace hrc_behavior_tree

#endif  // OMNI_DRIVE_ACTION_HPP
