#ifndef HRC_UTILS__UTILS_HPP
#define HRC_UTILS__UTILS_HPP

#include <cmath>

#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

namespace hrc_utils
{

/**
 * Convert a point from the robot coordinates to the map coordinates.
 * @param robot_pose Pose of the robot
 * @param robot_point Point in the robot coordinates
 * @param map_point Point in the map coordinates
 */
void robotToMap(
    const geometry_msgs::msg::Pose2D& robot_pose,
    const geometry_msgs::msg::Point32& robot_point,
    geometry_msgs::msg::Point32& map_point);

} // namespace hrc_utils

#endif // HRC_UTILS__UTILS_HPP
