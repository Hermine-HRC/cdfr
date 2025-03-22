#include "herminebot_navigation/robot_triangulation.hpp"
#include "nav2_util/array_parser.hpp"
#include "tf2_ros/create_timer_ros.h"
#include "tf2/LinearMath/Quaternion.h"
#include "nav2_util/robot_utils.hpp"
#include "tf2/utils.h"
#include <cmath>

namespace hrc_localization
{

/**
 * @brief Calculates the cotangent of an angle
 * @param alpha Angle in radians
 * @return Cotangent of the angle
 */
double cot(const double alpha) {return 1 / tan(alpha);}

RobotTriangulation::RobotTriangulation(const rclcpp::NodeOptions& options) : Node("robot_triangulation", options)
{
    // Transform buffer and listener initialization
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(),
        this->get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    declare_parameter("scan_topic", rclcpp::ParameterValue("scan"));
    declare_parameter("triangulation_topic", rclcpp::ParameterValue("triangulation"));
    declare_parameter("beacon_radius", rclcpp::ParameterValue(0.05));
    declare_parameter("visualize", rclcpp::ParameterValue(true));
    declare_parameter("visualization_topic", rclcpp::ParameterValue("triangulation_visualization"));
    declare_parameter("visualization_color", rclcpp::PARAMETER_DOUBLE_ARRAY);
    declare_parameter("global_frame", rclcpp::ParameterValue("map"));
    declare_parameter("transform_tolerance", rclcpp::ParameterValue(0.1));
    declare_parameter("beacons_pos_tolerance", rclcpp::ParameterValue(0.5));
    declare_parameter("triangulation_covariance", rclcpp::PARAMETER_DOUBLE_ARRAY);
    declare_parameter("team_colors", rclcpp::PARAMETER_STRING_ARRAY);
    get_parameter("team_colors", team_colors_);
    if (team_colors_.empty()) {
        RCLCPP_FATAL(get_logger(), "The team_colors parameter must contain at least one team color");
        throw std::runtime_error("The team_colors parameter must contain at least one team color");
    }
    for (const std::string& color : team_colors_) {
        declare_parameter(color + "_beacons_position", rclcpp::ParameterValue(""));
    }

    std::string scan_topic, triangulation_topic, visualization_topic;
    get_parameter("scan_topic", scan_topic);
    get_parameter("triangulation_topic", triangulation_topic);
    get_parameter("beacon_radius", beacon_radius_);
    get_parameter("visualize", visualize_);
    get_parameter("visualization_topic", visualization_topic);
    get_parameter("global_frame", global_frame_);
    transform_tolerance_ = tf2::durationFromSec(get_parameter("transform_tolerance").as_double());
    get_parameter("visualization_color", visualization_color_);

    if (visualization_color_.size() == 0) {
        visualization_color_ = {0.0, 0.0, 1.0, 0.7};
    }
    else if (visualization_color_.size() == 3) {
        visualization_color_.push_back(1.0);
    }
    else if (visualization_color_.size() != 4) {
        RCLCPP_ERROR(get_logger(), "Invalid visualization_color parameter. Setting to default.");
        visualization_color_ = {0.0, 0.0, 1.0, 0.7};
    }

    double beacons_pos_tolerance;
    get_parameter("beacons_pos_tolerance", beacons_pos_tolerance);
    beacons_pos_tolerance_sq_ = beacons_pos_tolerance * beacons_pos_tolerance;

    for (const std::string& color : team_colors_) {
        std::string beacons_position;
        get_parameter(color + "_beacons_position", beacons_position);
        std::string error;
        const std::vector<std::vector<float>> vvf = nav2_util::parseVVF(beacons_position, error);
        if (error != "") {
            RCLCPP_FATAL(get_logger(), "Error parsing beacons_position parameter: '%s'", error.c_str());
            throw std::runtime_error("Error parsing beacons_position parameter: '" + error + "'");
        }
        for (const auto& vv : vvf) {
            if (vv.size() == 2) {
                beacons_position_.insert({color, std::vector<Point>()});
                beacons_position_.at(color).push_back({vv[0], vv[1]});
            }
            else {
                RCLCPP_FATAL(get_logger(), "Points in the beacons_position specification must be pairs of numbers");
                throw std::runtime_error("Points in the beacons_position specification must be pairs of numbers");
            }
        }
        if (beacons_position_.at(color).size() != 3) {
            RCLCPP_FATAL(get_logger(), "The beacons_position parameter must contain exactly 3 points");
            throw std::runtime_error("The beacons_position parameter must contain exactly 3 points");
        }
    }

    std::vector<double> triangulation_covariance;
    get_parameter("triangulation_covariance", triangulation_covariance);
    if (triangulation_covariance.empty()) {
        triangulation_covariance =
            std::vector<double>(triangulation_covariance_default_, triangulation_covariance_default_ + 36);
    }
    else if (triangulation_covariance.size() != 36) {
        RCLCPP_FATAL(get_logger(), "The triangulation_covariance parameter must contain exactly 36 values");
        throw std::runtime_error("The triangulation_covariance parameter must contain exactly 36 values");
    }

    for (size_t i = 0 ; i < 36 ; i ++) {
        triangulation_covariance_[i] = triangulation_covariance.at(i);
    }

    parameters_handler_ = add_on_set_parameters_callback(
        std::bind(&RobotTriangulation::dynamicParametersCallback, this, std::placeholders::_1));

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic, 10, std::bind(&RobotTriangulation::scanCallback, this, std::placeholders::_1)
    );
    triangulation_pub_ = create_publisher<nav_msgs::msg::Odometry>(triangulation_topic, 10);
    visualization_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(visualization_topic, 10);
    initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 10, std::bind(&RobotTriangulation::initialPoseCallback, this, std::placeholders::_1)
    );

    get_team_color_client_ = create_client<hrc_interfaces::srv::GetTeamColor>(
        "get_team_color", rclcpp::QoS(1).reliable());
    loadTeamColor();

    RCLCPP_INFO(
        get_logger(), "Robot triangulation will listen to '%s' and publish to '%s' topics",
        scan_topic.c_str(), triangulation_topic.c_str());
}

rcl_interfaces::msg::SetParametersResult RobotTriangulation::dynamicParametersCallback(
    std::vector<rclcpp::Parameter> parameters)
{
    for (const rclcpp::Parameter& param : parameters) {
        if (param.get_name() == "visualize") {
            visualize_ = param.as_bool();
        }
        else if (param.get_name() == "visualization_color") {
            visualization_color_.clear();
            for (double val : param.as_double_array()) {
                visualization_color_.push_back(val);
            }
            if (visualization_color_.size() == 0) {
                visualization_color_ = {0.0, 0.0, 1.0, 0.7};
            }
            else if (visualization_color_.size() == 3) {
                visualization_color_.push_back(1.0);
            }
            else if (visualization_color_.size() != 4) {
                RCLCPP_ERROR(get_logger(), "Invalid visualization_color parameter. Setting to default.");
                visualization_color_ = {0.0, 0.0, 1.0, 0.7};
            }
        }
        else if (param.get_name() == "transform_tolerance") {
            transform_tolerance_ = tf2::durationFromSec(param.as_double());
        }
        else if (param.get_name() == "beacons_pos_tolerance") {
            beacons_pos_tolerance_sq_ = param.as_double() * param.as_double();
        }
        else if (param.get_name() == "triangulation_covariance") {
            std::vector<double> triangulation_covariance = param.as_double_array();
            if (triangulation_covariance.empty()) {
                triangulation_covariance =
                    std::vector<double>(triangulation_covariance_default_, triangulation_covariance_default_ + 36);
            }
            else if (triangulation_covariance.size() != 36) {
                RCLCPP_FATAL(get_logger(), "The triangulation_covariance parameter must contain exactly 36 values");
                throw std::runtime_error("The triangulation_covariance parameter must contain exactly 36 values");
            }

            for (size_t i = 0 ; i < 36 ; i ++) {
                triangulation_covariance_[i] = triangulation_covariance.at(i);
            }
        }
    }

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
}

void RobotTriangulation::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    lidar_frame_ = msg->header.frame_id;

    std::vector<Point> beacons_positions = calculateBeaconsPotentialPositions(msg);
    beacons_positions = assignBeaconsPositions(beacons_positions);
    if (beacons_positions.size() == 3) {
        const std::vector<Point> bp = beacons_position_.at(team_color_);
        const Point robot_position = triangulateRobotPosition(
            beacons_positions.at(0), beacons_positions.at(1), beacons_positions.at(2));

        // Calculate the robot yaw
        // For the demonstration of Leon, see https://drive.google.com/file/d/18svqZY04eZZcAPdmf5_HyuK7bsSWW7Pq/view?usp=drive_link
        float robot_yaw = 0.0f;
        for (uint8_t i = 0 ; i < 3 ; i ++) {
            float cos_yaw = (bp.at(i).x + beacons_positions.at(i).y / beacons_positions.at(i).x * bp.at(i).y -
                robot_position.x - beacons_positions.at(i).y / beacons_positions.at(i).x * robot_position.y) /
                (beacons_positions.at(i).x + pow(beacons_positions.at(i).y, 2) / beacons_positions.at(i).x);
            cos_yaw = std::min(1.0f, std::max(-1.0f, cos_yaw)); // Restrict the cos yaw in [-1, 1]
            float r_yaw = std::acos(cos_yaw); //  Angle positive

            float sin_yaw = (bp.at(i).y - cos(r_yaw) * beacons_positions.at(i).y - robot_position.y) /
                beacons_positions.at(i).x;
            sin_yaw = std::min(1.0f, std::max(-1.0f, sin_yaw)); // Restrict the sin yaw in [-1, 1]
            r_yaw = copysign(r_yaw, std::asin(sin_yaw)); // Angle signed
            robot_yaw += r_yaw;
        }
        robot_yaw /= 3.0f;

        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.frame_id = global_frame_;
        odom_msg.header.stamp = msg->header.stamp;
        odom_msg.child_frame_id = lidar_frame_;
        odom_msg.pose.pose.position.x = robot_position.x;
        odom_msg.pose.pose.position.y = robot_position.y;
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, robot_yaw);
        odom_msg.pose.pose.orientation = tf2::toMsg(q);
        odom_msg.pose.covariance = triangulation_covariance_;
        triangulation_pub_->publish(odom_msg);
    }
    else {
        RCLCPP_WARN(
            get_logger(), "Could not triangulate robot position, only %ld beacons measured", beacons_positions.size());
    }
    publishVisualization(beacons_positions);
}

std::vector<Point> RobotTriangulation::calculateBeaconsPotentialPositions(
    const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    // Detect circles
    std::vector<std::vector<Point>> circles;
    circles.push_back(std::vector<Point>());

    // Check the back for looping
    size_t j;
    for (j = scan->ranges.size() - 1 ; j > 0 ; j --) {
        const float range = scan->ranges.at(j);
        if (scan->range_min < range && range < scan->range_max) {
            const float angle = scan->angle_increment * j + scan->angle_min;
            circles.back().insert(circles.back().begin(), {range * std::cos(angle), range * std::sin(angle)});
        }
        else {
            break;
        }
    }

    for (size_t i = 0 ; i < j ; i ++) {
        const float range = scan->ranges.at(i);
        if (scan->range_min < range && range < scan->range_max) {
            const float angle = scan->angle_increment * i + scan->angle_min;
            circles.back().push_back({range * std::cos(angle), range * std::sin(angle)});
        }
        else if (circles.back().size() == 1) {
            // Point alone detected, we remove it be cause it can't be a beacon
            circles.back().clear();
        }
        else if (circles.back().size() > 1) {
            circles.push_back(std::vector<Point>());
        }
    }

    if (circles.back().size() < 2) { // If it is a point alone => can't be a beacon
        circles.pop_back();
    }

    // Calculate circles center
    std::vector<Point> centers;
    for (const std::vector<Point>& circle : circles) {
        std::vector<Point> circle_centers;
        for (size_t i = 0 ; i < circle.size() - 1 ; i ++) {
            circle_centers.push_back(calculateCircleCenter(circle.at(i), circle.at(i + 1)));
        }
        centers.push_back(calculateAveragePoint(circle_centers));
    }
    return centers;
}

Point RobotTriangulation::calculateCircleCenter(const Point p1, const Point p2) const
{
    // Based on the equations from: https://en.wikipedia.org/wiki/Sagitta_(geometry)
    const float chord_length = sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
    const float circle_center_to_chord_center = sqrt(pow(beacon_radius_, 2) - pow(chord_length, 2) / 4);

    const float chord_angle = std::atan2(p1.y - p2.y, p1.x - p2.x);
    const float circle_center_x = p2.x + circle_center_to_chord_center * std::cos(chord_angle);
    const float circle_center_y = p2.y + circle_center_to_chord_center * std::sin(chord_angle);
    return {circle_center_x, circle_center_y};
}

Point RobotTriangulation::calculateAveragePoint(const std::vector<Point>& points) const
{
    float x = 0.0;
    float y = 0.0;
    for (const Point& point : points) {
        x += point.x;
        y += point.y;
    }
    return {x / points.size(), y / points.size()};
}

std::vector<Point> RobotTriangulation::assignBeaconsPositions(const std::vector<Point>& beacons_positions) const
{
    // Transform beacon fixed positions to lidar frame
    tf2::Transform tf_transform;
    if (!nav2_util::getTransform(global_frame_, lidar_frame_, transform_tolerance_, tf_buffer_, tf_transform)) {
        RCLCPP_ERROR(
            get_logger(), "Failed to get transform from %s to %s", global_frame_.c_str(), lidar_frame_.c_str());
        return std::vector<Point>();
    }

    std::vector<Point> lidar_beacons_positions;
    for (const Point& pos : beacons_position_.at(team_color_)) {
        tf2::Vector3 p_v3_b(pos.x, pos.y, 0.0);
        tf2::Vector3 p_v3_s = tf_transform * p_v3_b;
        lidar_beacons_positions.push_back({(float) p_v3_s.x(), (float) p_v3_s.y()});
    }

    // Find the position that most likely corresponds to the real beacon
    std::vector<Point> output_beacons_positions;
    for (const Point& pos : lidar_beacons_positions) {
        double shortest_sq_gap = beacons_pos_tolerance_sq_;
        Point closest_beacon;
        bool found_beacon = false;
        for (const Point& beacon_pos : beacons_positions) {
            double sq_gap = pow(pos.x - beacon_pos.x, 2) + pow(pos.y - beacon_pos.y, 2);
            if (sq_gap < shortest_sq_gap) {
                shortest_sq_gap = sq_gap;
                closest_beacon = beacon_pos;
                found_beacon = true;
            }
        }
        if (found_beacon) {
            output_beacons_positions.push_back(closest_beacon);
        }
    }

    return output_beacons_positions;
}

Point RobotTriangulation::triangulateRobotPosition(const Point p1, const Point p2, const Point p3) const
{
    // Application of the ToTal algorithm: https://www.telecom.uliege.be/publi/publications/pierlot/Pierlot2014ANewThree/
    // Compute the modified beacon coordinates
    const std::vector<Point> bp = beacons_position_.at(team_color_);
    const Point p1_ = {bp.at(0).x - bp.at(1).x, bp.at(0).y - bp.at(1).y};
    const Point p3_ = {bp.at(2).x - bp.at(1).x, bp.at(2).y - bp.at(1).y};

    // Compute the three cotangents
    const float alpha1 = atan2(p1.y, p1.x);
    const float alpha2 = atan2(p2.y, p2.x);
    const float alpha3 = atan2(p3.y, p3.x);
    const float cot_12 = cot(alpha2 - alpha1);
    const float cot_23 = cot(alpha3 - alpha2);
    const float cot_31 = (1 - cot_12 * cot_23) / (cot_12 + cot_23);

    // Compute the modified circle center coordinates
    const Point p_12 = {p1_.x + cot_12 * p1_.y, p1_.y - cot_12 * p1_.x};
    const Point p_23 = {p3_.x - cot_23 * p3_.y, p3_.y + cot_23 * p3_.x};
    const Point p_31 = {p3_.x + p1_.x + cot_31 * (p3_.y - p1_.y), p3_.y + p1_.y - cot_31 * (p3_.x - p1_.x)};

    // Compute k_31
    const float k_31 = p1_.x * p3_.x + p1_.y * p3_.y + cot_31 * (p1_.x * p3_.y - p3_.x * p1_.y);

    // Compute D
    const float D = (p_12.x - p_23.x) * (p_23.y - p_31.y) - (p_12.y - p_23.y) * (p_23.x - p_31.x);
    const float k = k_31 / D;

    // Compute the robot position
    const float x = bp.at(1).x + k * (p_12.y - p_23.y);
    const float y = bp.at(1).y + k * (p_23.x - p_12.x);

    return {x, y};
}

void RobotTriangulation::publishVisualization(const std::vector<Point>& beacons_positions) const
{
    if (!visualize_) {
        return;
    }

    visualization_msgs::msg::MarkerArray msg;
    for (size_t i = 0; i < beacons_positions.size(); i ++) {
        msg.markers.push_back(visualization_msgs::msg::Marker());
        msg.markers.back().type = visualization_msgs::msg::Marker::CYLINDER;
        msg.markers.back().header.frame_id = lidar_frame_;
        msg.markers.back().id = i;
        msg.markers.back().pose.position.x = beacons_positions[i].x;
        msg.markers.back().pose.position.y = beacons_positions[i].y;
        msg.markers.back().pose.position.z = 0.25;
        msg.markers.back().scale.x = 2 * beacon_radius_;
        msg.markers.back().scale.y = 2 * beacon_radius_;
        msg.markers.back().scale.z = 0.5;
        msg.markers.back().color.r = visualization_color_[0];
        msg.markers.back().color.g = visualization_color_[1];
        msg.markers.back().color.b = visualization_color_[2];
        msg.markers.back().color.a = visualization_color_[3];
        msg.markers.back().points.push_back(geometry_msgs::msg::Point());
        msg.markers.back().points.back().x = beacons_positions[i].x;
        msg.markers.back().points.back().y = beacons_positions[i].y;
    }
    visualization_pub_->publish(msg);
}

void RobotTriangulation::loadTeamColor()
{
    auto request = std::make_shared<hrc_interfaces::srv::GetTeamColor::Request>();
    while (!get_team_color_client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
            rclcpp::shutdown();
        }
        RCLCPP_INFO(get_logger(), "service not available, waiting again...");
        usleep(10000);
    }

    auto result = get_team_color_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS) {
        const std::string team_color = result.get()->team_color;
        RCLCPP_INFO(get_logger(), "Received team color: %s", team_color.c_str());
        if (std::find(team_colors_.begin(), team_colors_.end(), team_color) == team_colors_.end()) {
            RCLCPP_ERROR(
                get_logger(),
                "The received team color '%s' is not in the list of team colors. Taking the first one",
                team_color.c_str());
            team_color_ = team_colors_.at(0);
        }
        else {
            team_color_ = team_color;
        }
    }
    else {
        RCLCPP_ERROR(get_logger(), "Failed to get team color. Taking the first one");
        team_color_ = team_colors_.at(0);
    }
}

void RobotTriangulation::initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    const double yaw = tf2::getYaw(msg->pose.pose.orientation);
    RCLCPP_INFO(
        get_logger(), "Initial pose received: (%.3f %.3f), yaw: %.3f",
        msg->pose.pose.position.x, msg->pose.pose.position.y, yaw);

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.frame_id = global_frame_;
    odom_msg.header.stamp = this->now();
    odom_msg.child_frame_id = lidar_frame_;
    odom_msg.pose.pose.position.x = msg->pose.pose.position.x;
    odom_msg.pose.pose.position.y = msg->pose.pose.position.y;
    odom_msg.pose.pose.orientation = msg->pose.pose.orientation;
    odom_msg.pose.covariance = msg->pose.covariance;
    triangulation_pub_->publish(odom_msg);
}

} // namespace hrc_localization
