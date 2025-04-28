#ifndef ROBOT_TRIANGULATION_HPP
#define ROBOT_TRIANGULATION_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <vector>
#include <map>
#include "hrc_interfaces/srv/get_team_color.hpp"
#include "hrc_interfaces/msg/restart.hpp"

namespace hrc_localization
{

struct Point
{
    float x;
    float y;
};

/**
 * @brief Node to triangulate the robot position from the laser scan.
 * There must be 3 beacons in known positions in the map frame.
 * The algorithm needs tf transform between the lidar frame and the map frame so it cannot calculate a position
 * if the initial position of the robot is not known.
 * The algorithm needs only 2 measured points from each beacon to calculate a position.
 * The triangulation is realized using the ToTal algorithm: https://www.telecom.uliege.be/publi/publications/pierlot/Pierlot2014ANewThree/
 */
class RobotTriangulation : public rclcpp::Node
{
public:
    RobotTriangulation(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    /**
     * @brief Process the laser scan to triangulate the robot position.
     * @param msg Laser scan message
     */
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    /**
     * @brief Callback executed when a parameter change is detected
     * @param parameters ParameterEvent message
     */
    rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

    /**
     * @brief Receive a position where the robot is and publish the odometry relating this position
     * @param msg The Pose message
     */
    void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

    /**
     * @brief Callback executed when a restart message is received
     * @param msg Restart message (unused)
     */
    void restartCallback(const hrc_interfaces::msg::Restart::SharedPtr);

protected:
    /**
     * @brief Calculate the potential positions of the beacons from the laser scan.cot
     * @param msg Laser scan message
     * @return Vector of potential positions of the beacons
     */
    std::vector<Point> calculateBeaconsPotentialPositions(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    /**
     * @brief Calculate the center of the circle that passes through the two points, based on the fact that the center
     * is the farthest point from the robot with a known radius.
     * @param p1 First point
     * @param p2 Second point
     * @return Center of the circle
     */
    Point calculateCircleCenter(const Point p1, const Point p2) const;

    /**
     * @brief Calculate the average point of a set of points.
     * @param points Set of points
     * @return Average point
     */
    Point calculateAveragePoint(const std::vector<Point>& points) const;

    /**
     * @brief Assign the beacons positions to the robot positions, based on the distance between the knwon position
     * of the beacons in the map frame and the beacons positions in the lidar frame.
     * @param beacons_positions Vector of beacons positions
     * @return Vector of robot positions
     */
    std::vector<Point> assignBeaconsPositions(const std::vector<Point>& beacons_positions) const;

    /**
     * @brief Publish the visualization of the mesured beacons.
     * @param beacons_positions Vector of beacons positions
     */
    void publishVisualization(const std::vector<Point>& beacons_positions) const;

    /**
     * @brief Triangulate the robot position from the measured beacons positions using the ToTal algorithm.
     * @param p1 First measured beacon position in lidar frame
     * @param p2 Second measured beacon position in lidar frame
     * @param p3 Third measured beacon position in lidar frame
     * @return Robot position in map frame
     */
    Point triangulateRobotPosition(const Point p1, const Point p2, const Point p3) const;

    /**
     * @brief Load the current team color from the server.
     */
    void loadTeamColor();

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr triangulation_pub_;
    double beacon_radius_;
    bool visualize_;
    tf2::Duration transform_tolerance_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visualization_pub_;
    std::string lidar_frame_, global_frame_;
    std::vector<double> visualization_color_;
    std::map<std::string, std::vector<Point>> beacons_position_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    double beacons_pos_tolerance_sq_;
    std::string team_color_;
    std::vector<std::string> team_colors_;
    rclcpp::Client<hrc_interfaces::srv::GetTeamColor>::SharedPtr get_team_color_client_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameters_handler_;
    std::array<double, 36> triangulation_covariance_;
    rclcpp::Subscription<hrc_interfaces::msg::Restart>::SharedPtr restart_sub_;

    static constexpr double triangulation_covariance_default_[36] = {
        //  x,  y,  z, roll, pitch, yaw
        0.01, 0.0, 0.0, 0.0, 0.0, 0.0, // x
        0.0, 0.01, 0.0, 0.0, 0.0, 0.0, // y
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, // z
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, // roll
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, // pitch
        0.0, 0.0, 0.0, 0.0, 0.0, 0.01 // yaw
    };
};

}  // namespace hrc_localization

#endif  // ROBOT_TRIANGULATION_HPP
