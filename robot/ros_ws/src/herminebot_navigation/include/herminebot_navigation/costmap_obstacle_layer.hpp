#ifndef HRC_COSTMAP_OBSTACLE_LAYER
#define HRC_COSTMAP_OBSTACLE_LAYER

#include "nav2_costmap_2d/obstacle_layer.hpp"

namespace hrc_costmap_2d
{

class ObstacleLayer : public nav2_costmap_2d::ObstacleLayer
{
public:
    /**
     * @brief Initialization process of layer on startup
     */
    virtual void onInitialize();

    /**
     * @brief Update the bounds of the master costmap by this layer's update dimensions
     * @param robot_x X pose of robot
     * @param robot_y Y pose of robot
     * @param robot_yaw Robot orientation
     * @param min_x X min map coord of the window to update
     * @param min_y Y min map coord of the window to update
     * @param max_x X max map coord of the window to update
     * @param max_y Y max map coord of the window to update
     */
    virtual void updateBounds(
        double robot_x, double robot_y, double robot_yaw, double * min_x,
        double * min_y, double * max_x, double * max_y);
    
    /**
     * @brief Callback executed when a parameter change is detected
     * @param event ParameterEvent message
     */
    rcl_interfaces::msg::SetParametersResult
    dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

protected:
    double inflation_radius_;
    double layer_width_;
    double layer_height_;

    /**
     * @brief Apply inflation radius around center point
     * @param x_center x position of the center to apply inflation
     * @param y_center y position of the center to apply inflation
     */
    void applyInflation(const unsigned int x_center, const unsigned int y_center);
    bool isPointInside(std::vector<std::pair<double, double>>& poly, const int x, const int y) const;
    std::vector<std::pair<double, double>> poly_;
};

} // namespace hrc_costmap_2d

#endif // HRC_COSTMAP_OBSTACLE_LAYER
