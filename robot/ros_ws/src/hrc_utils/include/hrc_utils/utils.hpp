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

/** Check if two floats are equal
 * @param a First float
 * @param b Second float
 * @param epsilon Error margin
 * @return True if the two floats are equal
 */
bool floatEqual(const float a, const float b, const float epsilon = 1e-5f);
bool floatEqual(const double a, const double b, const double epsilon = 1e-5);

} // namespace hrc_utils

#endif // HRC_UTILS__UTILS_HPP
