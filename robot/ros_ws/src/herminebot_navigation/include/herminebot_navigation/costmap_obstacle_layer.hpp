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
     * @brief  A callback to handle buffering LaserScan messages
     * @param message The message returned from a message notifier
     * @param buffer A pointer to the observation buffer to update
     */
    void laserScanCallback(
        sensor_msgs::msg::LaserScan::ConstSharedPtr message,
        const std::shared_ptr<nav2_costmap_2d::ObservationBuffer> & buffer);
};

} // namespace hrc_costmap_2d

#endif // HRC_COSTMAP_OBSTACLE_LAYER
