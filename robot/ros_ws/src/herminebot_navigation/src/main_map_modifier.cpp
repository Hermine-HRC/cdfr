#include "herminebot_navigation/map_modifier.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) 
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<hrc_map::MapModifier>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
