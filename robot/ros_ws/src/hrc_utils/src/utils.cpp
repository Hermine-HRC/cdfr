#include "hrc_utils/utils.hpp"

namespace hrc_utils
{

void robotToMap(
    const geometry_msgs::msg::Pose2D& robot_pose,
    const geometry_msgs::msg::Point32& robot_point,
    geometry_msgs::msg::Point32& map_point)
{
    map_point.x = robot_point.x * cos(robot_pose.theta) - robot_point.y * sin(robot_pose.theta) + robot_pose.x;
    map_point.y = robot_point.x * sin(robot_pose.theta) + robot_point.y * cos(robot_pose.theta) + robot_pose.y;
}

bool floatEqual(const float a, const float b, const float epsilon)
{
    return fabs(a - b) < epsilon;
}

bool floatEqual(const double a, const double b, const double epsilon)
{
    return fabs(a - b) < epsilon;
}

} // namespace hrc_utils
