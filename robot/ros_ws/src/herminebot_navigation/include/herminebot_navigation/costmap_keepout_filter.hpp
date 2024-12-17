#ifndef HRC_COSTMAP_KEEPOUT_FILTER_HPP
#define HRC_COSTMAP_KEEPOUT_FILTER_HPP

#include "nav2_costmap_2d/costmap_filters/costmap_filter.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/msg/costmap_filter_info.hpp"

namespace hrc_costmap_2d
{

/**
 * @class KeepoutFilter
 * @brief Reads in a keepout mask and marks keepout regions in the map
 * to prevent planning or control in restricted areas
 */
class KeepoutFilter : public nav2_costmap_2d::CostmapFilter
{
public:
    /**
     * @brief A constructor
     */
    KeepoutFilter();

    /**
     * @brief Initialize the filter and subscribe to the info topic
     */
    void initializeFilter(
        const std::string & filter_info_topic);

    /**
     * @brief Process the keepout layer at the current pose / bounds / grid
     */
    void process(
        nav2_costmap_2d::Costmap2D & master_grid,
        int min_i, int min_j, int max_i, int max_j,
        const geometry_msgs::msg::Pose2D & pose);

    /**
     * @brief Reset the costmap filter / topic / info
     */
    void resetFilter();

    /**
     * @brief If this filter is active
     */
    bool isActive();

protected:
    /**
     * @brief Callback for the filter information
     */
    void filterInfoCallback(const nav2_msgs::msg::CostmapFilterInfo::SharedPtr msg);
    /**
     * @brief Callback for the filter mask
     */
    void maskCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    /**
     * @brief apply an inflation area around the obstacles of the size of the robot radius
     */
    void applyRobotRadius(nav2_costmap_2d::Costmap2D& master_grid, unsigned char* master_array, const unsigned int x, const unsigned int y);

    rclcpp::Subscription<nav2_msgs::msg::CostmapFilterInfo>::SharedPtr filter_info_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr mask_sub_;

    std::unique_ptr<nav2_costmap_2d::Costmap2D> mask_costmap_;

    std::string mask_frame_;  // Frame where mask located in
    std::string global_frame_;  // Frame of currnet layer (master_grid)

    double robot_radius_;
};

}

#endif // HRC_COSTMAP_KEEPOUT_FILTER_HPP
