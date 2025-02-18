#include "gtest/gtest.h"
#include "herminebot_navigation/robot_triangulation.hpp"
#include "test_service.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

class ColorTeamService : public TestService<hrc_interfaces::srv::GetTeamColor>
{
public:
    ColorTeamService() : TestService("get_team_color") {}

protected:
    void handle_service(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<hrc_interfaces::srv::GetTeamColor::Request> request,
        const std::shared_ptr<hrc_interfaces::srv::GetTeamColor::Response> response) override
    {
        (void) request_header;
        (void) request;
        response->team_color = "test";
    }
};

class RobotTriangulationWrapper : public hrc_localization::RobotTriangulation
{
public:
    RobotTriangulationWrapper(rclcpp::NodeOptions options = rclcpp::NodeOptions()) : RobotTriangulation(options) {}
    std::vector<double> getVisualizationColor() const
    {
        return visualization_color_;
    }
    bool getVisualize() const
    {
        return visualize_;
    }
    double getBeaconsPosTolerance() const
    {
        return sqrt(beacons_pos_tolerance_sq_);
    }
    double getTransformTolerance() const
    {
        return tf2::durationToSec(transform_tolerance_);
    }
};

struct ScanPoint
{
    double dist;
    double angle;
};

class Tester : public ::testing::Test
{
public:
    Tester();
    void setRobotTriangulation(std::shared_ptr<RobotTriangulationWrapper>& rt);
    sensor_msgs::msg::LaserScan::SharedPtr getScan(const std::vector<ScanPoint>& scan_points);
    bool waitPosition(const std::chrono::nanoseconds& timeout);
    bool waitBeacons(const std::chrono::nanoseconds& timeout);
    void publishPose(float x, float y, float yaw);

protected:
    void positionCb(const nav_msgs::msg::Odometry::SharedPtr msg);
    void beaconsCb(const visualization_msgs::msg::MarkerArray msg);

    geometry_msgs::msg::Pose pose_;
    visualization_msgs::msg::MarkerArray beacons_marker_;
    bool pose_received_;
    bool beacons_received_;
    std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Odometry>> pose_sub_;
    std::shared_ptr<rclcpp::Subscription<visualization_msgs::msg::MarkerArray>> beacons_sub_;

private:
    std::shared_ptr<RobotTriangulationWrapper> rt_;
};

Tester::Tester()
{
    rt_ = nullptr;
    pose_received_ = false;
    beacons_received_ = false;
}

void Tester::setRobotTriangulation(std::shared_ptr<RobotTriangulationWrapper>& rt)
{
    rt_ = rt;
    pose_sub_ = rt_->create_subscription<nav_msgs::msg::Odometry>(
        "triangulation", 1, std::bind(&Tester::positionCb, this, std::placeholders::_1));
    beacons_sub_ = rt_->create_subscription<visualization_msgs::msg::MarkerArray>(
        "triangulation_visualization", 1, std::bind(&Tester::beaconsCb, this, std::placeholders::_1));
}

void Tester::positionCb(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    pose_ = msg->pose.pose;
    pose_received_ = true;
}

void Tester::beaconsCb(const visualization_msgs::msg::MarkerArray msg)
{
    beacons_marker_ = msg;
    beacons_received_ = true;
}

sensor_msgs::msg::LaserScan::SharedPtr Tester::getScan(const std::vector<ScanPoint>& scan_points)
{
    sensor_msgs::msg::LaserScan::SharedPtr scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();
    scan_msg->header.frame_id = "lidar_link";
    scan_msg->range_min = 0.1f;
    scan_msg->range_max = 10.0f;
    scan_msg->angle_min = 0.0f;
    scan_msg->angle_max = 2.0f * M_PI;
    scan_msg->angle_increment = M_PI / 180.0f; // 1° in rad
    for (float i = scan_msg->angle_min ; i < scan_msg->angle_max ; i += scan_msg->angle_increment) {
        bool added = false;
        for (ScanPoint point : scan_points) {
            if (i <= point.angle && point.angle <= i + scan_msg->angle_increment) {
                scan_msg->ranges.push_back(point.dist);
                if (i + scan_msg->angle_increment <= scan_msg->angle_max) {
                    scan_msg->ranges.push_back(point.dist); // Duplicate so there is two points in the scan
                    i += scan_msg->angle_increment;
                }
                else {
                    scan_msg->ranges.at(0) = point.dist;
                }
                added = true;
                break;
            }
        }
        if (!added) {
            scan_msg->ranges.push_back(0.0f);
        }
    }
    pose_received_ = false;
    beacons_received_ = false;
    return scan_msg;
}

bool Tester::waitPosition(const std::chrono::nanoseconds& timeout)
{
    rclcpp::Time start_time = rt_->now();
    while (rclcpp::ok() && rt_->now() - start_time <= rclcpp::Duration(timeout)) {
        if (pose_received_) {
            return true;
        }
        rclcpp::spin_some(rt_->get_node_base_interface());
        std::this_thread::sleep_for(10ms);
    }
    return false;
}

bool Tester::waitBeacons(const std::chrono::nanoseconds& timeout)
{
    rclcpp::Time start_time = rt_->now();
    while (rclcpp::ok() && rt_->now() - start_time <= rclcpp::Duration(timeout)) {
        if (beacons_received_) {
            return true;
        }
        rclcpp::spin_some(rt_->get_node_base_interface());
        std::this_thread::sleep_for(10ms);
    }
    return false;
}

void Tester::publishPose(float x, float y, float yaw)
{
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster =
        std::make_shared<tf2_ros::TransformBroadcaster>(rt_);
    geometry_msgs::msg::TransformStamped transform;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    transform.transform.rotation.x = q.getX();
    transform.transform.rotation.y = q.getY();
    transform.transform.rotation.z = q.getZ();
    transform.transform.rotation.w = q.getW();

    transform.transform.translation.x = x;
    transform.transform.translation.y = y;
    transform.transform.translation.z = 0.0;

    const rclcpp::Duration ahead = 100ms;
    const rclcpp::Time stamp = rt_->now();
    // Fill TF buffer ahead for future transform usage
    for (rclcpp::Time t = stamp ; t <= stamp + ahead ;
        t += rclcpp::Duration(50ms))
    {
        transform.header.stamp = t;
        transform.header.frame_id = "map";
        transform.child_frame_id = "lidar_link";
        tf_broadcaster->sendTransform(transform);
    }
}

TEST_F(Tester, testRobotTriangulation)
{
    rclcpp::NodeOptions options;
    std::vector<rclcpp::Parameter> parameters = {
        rclcpp::Parameter("team_colors", std::vector<std::string>{"test"}),
        rclcpp::Parameter("test_beacons_position", "[[1.0, 1.0], [0.0, 0.0], [1.0, -1.0]]")
    };
    options.parameter_overrides(parameters);
    auto rt = std::make_shared<RobotTriangulationWrapper>(options);
    setRobotTriangulation(rt);

    std::vector<ScanPoint> scan_points = {
        {1.12, 1.107},
        {0.5, 3.1416},
        {1.12, 5.176}
    };
    sensor_msgs::msg::LaserScan::SharedPtr scan;
    scan = getScan(scan_points);
    publishPose(0.5, 0.0, 0.0);
    rt->scanCallback(scan);
    ASSERT_TRUE(waitPosition(100ms));
    ASSERT_NEAR(pose_.position.x, 0.5, 0.05);
    ASSERT_NEAR(pose_.position.y, 0.0, 0.05);
    ASSERT_NEAR(tf2::getYaw(pose_.orientation), 0.0, 0.1);

    ASSERT_TRUE(waitBeacons(100ms));
    ASSERT_EQ(beacons_marker_.markers.size(), 3);
    ASSERT_NEAR(beacons_marker_.markers.at(0).pose.position.x, 0.5, 0.05);
    ASSERT_NEAR(beacons_marker_.markers.at(0).pose.position.y, 1.0, 0.05);
    ASSERT_NEAR(beacons_marker_.markers.at(1).pose.position.x, -0.5, 0.05);
    ASSERT_NEAR(beacons_marker_.markers.at(1).pose.position.y, 0.0, 0.05);
    ASSERT_NEAR(beacons_marker_.markers.at(2).pose.position.x, 0.5, 0.05);
    ASSERT_NEAR(beacons_marker_.markers.at(2).pose.position.y, -1.0, 0.05);
}

TEST_F(Tester, testDynamicParameters)
{
    rclcpp::NodeOptions options;
    std::vector<rclcpp::Parameter> parameters = {
        rclcpp::Parameter("team_colors", std::vector<std::string>{"test"}),
        rclcpp::Parameter("test_beacons_position", "[[1.0, 1.0], [0.0, 0.0], [1.0, -1.0]]")
    };
    options.parameter_overrides(parameters);
    auto rt = std::make_shared<RobotTriangulationWrapper>(options);
    setRobotTriangulation(rt);

    parameters = {
        rclcpp::Parameter("visualize", false),
        rclcpp::Parameter("visualization_color", std::vector<double>{1.0, 1.0, 0.0, 0.4}),
        rclcpp::Parameter("transform_tolerance", 0.6),
        rclcpp::Parameter("beacons_pos_tolerance", 0.9)
    };

    rt->dynamicParametersCallback(parameters);

    ASSERT_EQ(rt->getVisualize(), false);
    ASSERT_NEAR(rt->getBeaconsPosTolerance(), 0.9, 0.0001);
    ASSERT_NEAR(rt->getTransformTolerance(), 0.6, 0.0001);
    std::vector<double> visualization_color = rt->getVisualizationColor();
    ASSERT_EQ(visualization_color.size(), 4);
    ASSERT_NEAR(visualization_color.at(0), 1.0, 0.0001);
    ASSERT_NEAR(visualization_color.at(1), 1.0, 0.0001);
    ASSERT_NEAR(visualization_color.at(2), 0.0, 0.0001);
    ASSERT_NEAR(visualization_color.at(3), 0.4, 0.0001);

    parameters = {
        rclcpp::Parameter("visualization_color", std::vector<double>())
    };

    rt->dynamicParametersCallback(parameters);

    visualization_color = rt->getVisualizationColor();
    ASSERT_EQ(visualization_color.size(), 4);
    ASSERT_NEAR(visualization_color.at(0), 0.0, 0.0001);
    ASSERT_NEAR(visualization_color.at(1), 0.0, 0.0001);
    ASSERT_NEAR(visualization_color.at(2), 1.0, 0.0001);
    ASSERT_NEAR(visualization_color.at(3), 0.7, 0.0001);

    parameters = {
        rclcpp::Parameter("visualization_color", std::vector<double>{0.3, 0.5, 0.4})
    };

    rt->dynamicParametersCallback(parameters);

    visualization_color = rt->getVisualizationColor();
    ASSERT_EQ(visualization_color.size(), 4);
    ASSERT_NEAR(visualization_color.at(0), 0.3, 0.0001);
    ASSERT_NEAR(visualization_color.at(1), 0.5, 0.0001);
    ASSERT_NEAR(visualization_color.at(2), 0.4, 0.0001);
    ASSERT_NEAR(visualization_color.at(3), 1.0, 0.0001);

    parameters = {
        rclcpp::Parameter("visualization_color", std::vector<double>{0.0, 0., 0.0, 0.0, 0.0, 0.0})
    };

    rt->dynamicParametersCallback(parameters);

    visualization_color = rt->getVisualizationColor();
    ASSERT_EQ(visualization_color.size(), 4);
    ASSERT_NEAR(visualization_color.at(0), 0.0, 0.0001);
    ASSERT_NEAR(visualization_color.at(1), 0.0, 0.0001);
    ASSERT_NEAR(visualization_color.at(2), 1.0, 0.0001);
    ASSERT_NEAR(visualization_color.at(3), 0.7, 0.0001);
}

TEST_F(Tester, test_initialization)
{
    rclcpp::NodeOptions options;
    ASSERT_THROW(std::make_shared<RobotTriangulationWrapper>(options), std::runtime_error); // No team colors

    std::vector<rclcpp::Parameter> parameters = {
        rclcpp::Parameter("team_colors", std::vector<std::string>{"test"}),
        rclcpp::Parameter("visualization_color", "[0.1]")
    };
    options.parameter_overrides(parameters);
    // Invalid visualization color size
    ASSERT_THROW(std::make_shared<RobotTriangulationWrapper>(options), std::runtime_error);

    parameters = {
        rclcpp::Parameter("team_colors", std::vector<std::string>{"test"}),
        rclcpp::Parameter("test_beacons_position", "[[1.0, 1.0], [0.0, 0.0]]")
    };
    options.parameter_overrides(parameters);
    // Only two beacons
    ASSERT_THROW(std::make_shared<RobotTriangulationWrapper>(options), std::runtime_error);

    parameters = {
        rclcpp::Parameter("team_colors", std::vector<std::string>{"test"}),
        rclcpp::Parameter("test_beacons_position", "[[1.0, 1.0], [0.0, 0.0], 1.0, -1.0]]")
    };
    options.parameter_overrides(parameters);
    // Invalid array
    ASSERT_THROW(std::make_shared<RobotTriangulationWrapper>(options), std::runtime_error);

    parameters = {
        rclcpp::Parameter("team_colors", std::vector<std::string>{"test"}),
        rclcpp::Parameter("test_beacons_position", "[[1.0, 1.0], [0.0, 0.0], [1.0, -1.0, 0.0]]")
    };
    options.parameter_overrides(parameters);
    // Not a point
    ASSERT_THROW(std::make_shared<RobotTriangulationWrapper>(options), std::runtime_error);

    parameters = {
        rclcpp::Parameter("team_colors", std::vector<std::string>{"test"}),
        rclcpp::Parameter("test_beacons_position", "[[1.0, 1.0], [0.0, 0.0], [1.0, -1.0]]")
    };
    options.parameter_overrides(parameters);
    // Correct config
    ASSERT_NO_THROW(std::make_shared<RobotTriangulationWrapper>(options));

    auto rt = std::make_shared<RobotTriangulationWrapper>(options);
    std::vector<double> visualization_color = rt->getVisualizationColor();
    ASSERT_EQ(visualization_color.size(), 4);
    ASSERT_NEAR(visualization_color.at(0), 0.0, 0.0001);
    ASSERT_NEAR(visualization_color.at(1), 0.0, 0.0001);
    ASSERT_NEAR(visualization_color.at(2), 1.0, 0.0001);
    ASSERT_NEAR(visualization_color.at(3), 0.7, 0.0001);

    parameters = {
        rclcpp::Parameter("team_colors", std::vector<std::string>{"test"}),
        rclcpp::Parameter("test_beacons_position", "[[1.0, 1.0], [0.0, 0.0], [1.0, -1.0]]"),
        rclcpp::Parameter("visualization_color", std::vector<double>{0.0, 0.0, 1.0})
    };
    options.parameter_overrides(parameters);
    rt = std::make_shared<RobotTriangulationWrapper>(options);
    visualization_color = rt->getVisualizationColor();
    ASSERT_EQ(visualization_color.size(), 4);
    ASSERT_NEAR(visualization_color.at(0), 0.0, 0.0001);
    ASSERT_NEAR(visualization_color.at(1), 0.0, 0.0001);
    ASSERT_NEAR(visualization_color.at(2), 1.0, 0.0001);
    ASSERT_NEAR(visualization_color.at(3), 1.0, 0.0001);

    parameters = {
        rclcpp::Parameter("team_colors", std::vector<std::string>{"test"}),
        rclcpp::Parameter("test_beacons_position", "[[1.0, 1.0], [0.0, 0.0], [1.0, -1.0]]"),
        rclcpp::Parameter("visualization_color", std::vector<double>{0.3, 0.5})
    };
    options.parameter_overrides(parameters);
    rt = std::make_shared<RobotTriangulationWrapper>(options);
    visualization_color = rt->getVisualizationColor();
    ASSERT_EQ(visualization_color.size(), 4);
    ASSERT_NEAR(visualization_color.at(0), 0.0, 0.0001);
    ASSERT_NEAR(visualization_color.at(1), 0.0, 0.0001);
    ASSERT_NEAR(visualization_color.at(2), 1.0, 0.0001);
    ASSERT_NEAR(visualization_color.at(3), 0.7, 0.0001);
}

TEST_F(Tester, test_not_enough_beacons_detected)
{
    rclcpp::NodeOptions options;
    std::vector<rclcpp::Parameter> parameters = {
        rclcpp::Parameter("team_colors", std::vector<std::string>{"test"}),
        rclcpp::Parameter("test_beacons_position", "[[1.0, 1.0], [0.0, 0.0], [1.0, -1.0]]")
    };
    options.parameter_overrides(parameters);
    auto rt = std::make_shared<RobotTriangulationWrapper>(options);
    setRobotTriangulation(rt);

    std::vector<ScanPoint> scan_points = {
        {1.12, 1.107},
        {0.5, 3.1416}
    };
    sensor_msgs::msg::LaserScan::SharedPtr scan;
    scan = getScan(scan_points);
    scan->ranges.at(0) = 1.1f; // Set point alone
    publishPose(0.5, 0.0, 0.0);
    rt->scanCallback(scan);
    ASSERT_FALSE(waitPosition(50ms));

    ASSERT_TRUE(waitBeacons(100ms));
    ASSERT_EQ(beacons_marker_.markers.size(), 2);
    ASSERT_NEAR(beacons_marker_.markers.at(0).pose.position.x, 0.5, 0.05);
    ASSERT_NEAR(beacons_marker_.markers.at(0).pose.position.y, 1.0, 0.05);
    ASSERT_NEAR(beacons_marker_.markers.at(1).pose.position.x, -0.5, 0.05);
    ASSERT_NEAR(beacons_marker_.markers.at(1).pose.position.y, 0.0, 0.05);
}

TEST_F(Tester, test_wrap_around_circle)
{
    rclcpp::NodeOptions options;
    std::vector<rclcpp::Parameter> parameters = {
        rclcpp::Parameter("team_colors", std::vector<std::string>{"test"}),
        rclcpp::Parameter("test_beacons_position", "[[1.0, 1.0], [0.0, 0.0], [1.0, -1.0]]")
    };
    options.parameter_overrides(parameters);
    auto rt = std::make_shared<RobotTriangulationWrapper>(options);
    setRobotTriangulation(rt);

    std::vector<ScanPoint> scan_points = {
        {0.5, 0.0},
        {0.5, 2 * M_PI}
    };
    sensor_msgs::msg::LaserScan::SharedPtr scan;
    scan = getScan(scan_points);
    ASSERT_NEAR(scan->ranges.at(0), 0.5, 0.0001);
    ASSERT_NEAR(scan->ranges.back(), 0.5, 0.0001);
    publishPose(0.5, 0.0, M_PI);
    rt->scanCallback(scan);
    ASSERT_FALSE(waitPosition(50ms));

    ASSERT_TRUE(waitBeacons(100ms));
    ASSERT_EQ(beacons_marker_.markers.size(), 1);
    ASSERT_NEAR(beacons_marker_.markers.at(0).pose.position.x, 0.5, 0.05);
    ASSERT_NEAR(beacons_marker_.markers.at(0).pose.position.y, 0.0, 0.05);
}

TEST_F(Tester, not_a_beacon_detected)
{
    rclcpp::NodeOptions options;
    std::vector<rclcpp::Parameter> parameters = {
        rclcpp::Parameter("team_colors", std::vector<std::string>{"test"}),
        rclcpp::Parameter("test_beacons_position", "[[1.0, 1.0], [0.0, 0.0], [1.0, -1.0]]")
    };
    options.parameter_overrides(parameters);
    auto rt = std::make_shared<RobotTriangulationWrapper>(options);
    setRobotTriangulation(rt);

    std::vector<ScanPoint> scan_points = {
        {1.12, 1.107},
        {0.5, 3.1416},
        {0.5, 5.176} // too far from the expected position of the beacon
    };
    sensor_msgs::msg::LaserScan::SharedPtr scan;
    scan = getScan(scan_points);
    publishPose(0.5, 0.0, 0.0);
    rt->scanCallback(scan);
    ASSERT_FALSE(waitPosition(50ms));

    ASSERT_TRUE(waitBeacons(100ms));
    ASSERT_EQ(beacons_marker_.markers.size(), 2);
    ASSERT_NEAR(beacons_marker_.markers.at(0).pose.position.x, 0.5, 0.05);
    ASSERT_NEAR(beacons_marker_.markers.at(0).pose.position.y, 1.0, 0.05);
    ASSERT_NEAR(beacons_marker_.markers.at(1).pose.position.x, -0.5, 0.05);
    ASSERT_NEAR(beacons_marker_.markers.at(1).pose.position.y, 0.0, 0.05);
}

int main(int argc, char** argv)
{
    // Initialize the system
    testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);

    auto team_color_server = std::make_shared<ColorTeamService>();
    std::thread server_thread([team_color_server]() {
            rclcpp::spin(team_color_server);
        });

    // Actual testing
    bool test_result = RUN_ALL_TESTS();

    // Shutdown
    rclcpp::shutdown();
    server_thread.join();

    return test_result;
}
