#include "hrc_utils/utils.hpp"
#include "hrc_utils/testing_utils.hpp"

#include <gtest/gtest.h>

// Test the robotToMap function
TEST(TestUtils, RobotToMap)
{
    geometry_msgs::msg::Pose2D robot_pose;
    geometry_msgs::msg::Point32 robot_point;
    geometry_msgs::msg::Point32 map_point;

    // Check robot at the origin
    robot_pose.x = 0.0;
    robot_pose.y = 0.0;
    robot_pose.theta = 0.0;

    robot_point.x = 1.0;
    robot_point.y = 1.0;

    hrc_utils::robotToMap(robot_pose, robot_point, map_point);
    ASSERT_NEAR(map_point.x, 1.0, HRC_UTILS__TESTING_FLOAT_ASSERTION_PRECISION);
    ASSERT_NEAR(map_point.y, 1.0, HRC_UTILS__TESTING_FLOAT_ASSERTION_PRECISION);

    // Check robot still at the origin but rotated 90 degrees
    robot_pose.theta = M_PI_2;

    hrc_utils::robotToMap(robot_pose, robot_point, map_point);
    ASSERT_NEAR(map_point.x, -1.0, HRC_UTILS__TESTING_FLOAT_ASSERTION_PRECISION);
    ASSERT_NEAR(map_point.y, 1.0, HRC_UTILS__TESTING_FLOAT_ASSERTION_PRECISION);

    // Check robot not at the origin and still at 90 degrees
    robot_pose.x = 0.5;
    robot_pose.y = 0.5;

    hrc_utils::robotToMap(robot_pose, robot_point, map_point);
    ASSERT_NEAR(map_point.x, -0.5, HRC_UTILS__TESTING_FLOAT_ASSERTION_PRECISION);
    ASSERT_NEAR(map_point.y, 1.5, HRC_UTILS__TESTING_FLOAT_ASSERTION_PRECISION);
}

TEST(TestUtils, FloatComparison)
{
    // Double
    ASSERT_TRUE(hrc_utils::floatEqual(1.0, 1.0, HRC_UTILS__TESTING_FLOAT_ASSERTION_PRECISION));
    ASSERT_FALSE(hrc_utils::floatEqual(1.0, 1.1, HRC_UTILS__TESTING_FLOAT_ASSERTION_PRECISION));

    // Float
    ASSERT_TRUE(hrc_utils::floatEqual(1.0f, 1.0f, (float) HRC_UTILS__TESTING_FLOAT_ASSERTION_PRECISION));
    ASSERT_FALSE(hrc_utils::floatEqual(1.0f, 1.1f, (float) HRC_UTILS__TESTING_FLOAT_ASSERTION_PRECISION));
}
