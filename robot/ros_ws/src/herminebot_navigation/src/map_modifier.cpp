#include "herminebot_navigation/map_modifier.hpp"
#include "nav2_util/occ_grid_values.hpp"
#include "geometry_msgs/msg/point32.hpp"

#include <algorithm>
#include <memory>
#include <opencv2/opencv.hpp>

namespace hrc_map
{

MapModifier::MapModifier(const rclcpp::NodeOptions& options) : rclcpp::Node("map_modifier", options)
{
    declare_parameter("mask_filter_topic", rclcpp::ParameterValue("/mask_filter"));
    declare_parameter("initial_mask_topic", rclcpp::ParameterValue("/elements_mask"));

    std::string mask_filter_topic, elements_mask_topic;
    get_parameter("mask_filter_topic", mask_filter_topic);
    get_parameter("initial_mask_topic", elements_mask_topic);

    srv_ = create_service<hrc_interfaces::srv::ManageObjectsMap>(
        "manage_object_map",
        std::bind(&MapModifier::manageObjectsCb, this, std::placeholders::_1, std::placeholders::_2)
    );

    rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    mask_filter_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(mask_filter_topic, qos);

    elements_mask_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        elements_mask_topic, qos,
        std::bind(&MapModifier::initialMaskCb, this, std::placeholders::_1)
    );

    parameters_handle_ = add_on_set_parameters_callback(
        std::bind(
            &MapModifier::dynamicParametersCallback,
            this,
            std::placeholders::_1
    ));

    RCLCPP_INFO(get_logger(), "Node ready. Will publish mask filter on topic '%s'", mask_filter_topic.c_str());
}

void MapModifier::manageObjectsCb(
    const std::shared_ptr<hrc_interfaces::srv::ManageObjectsMap::Request> req,
    const std::shared_ptr<hrc_interfaces::srv::ManageObjectsMap::Response> /*res*/
)
{
    // Remove polygons
    geometry_msgs::msg::Polygon poly;
    for (const geometry_msgs::msg::Point32 point32 : req->points_objects_to_remove) {
        std::vector<size_t> to_be_removed;
        for (size_t i = 0 ; i < added_polygons_.size() ; i ++) {
            poly = added_polygons_.at(i);
            if (isPointInPoly(poly, point32.x, point32.y)) {
                applyPolyValue(poly, nav2_util::OCC_GRID_FREE);
                to_be_removed.push_back(i);
            }
        }

        for (size_t idx : to_be_removed) {
            added_polygons_.erase(std::next(added_polygons_.begin(), idx));
        }
    }

    // Add polygons
    for (geometry_msgs::msg::Polygon polygon : req->new_objects) {
        for (geometry_msgs::msg::Point32& point32 : polygon.points) {
            // Move 1 cell to top so the polygon is totally taken into account
            point32.y -= mask_resolution_;
        }

        applyPolyValue(polygon, nav2_util::OCC_GRID_OCCUPIED);
        added_polygons_.push_back(polygon);
    }

    publishMask();
}

void MapModifier::applyPolyValue(const geometry_msgs::msg::Polygon& polygon, const int8_t value)
{
    std::vector<std::array<unsigned int, HRC_UTILS__POINT_ARRAY_SIZE>> poly;
    poly.reserve(polygon.points.size());

    std::array<unsigned int, HRC_UTILS__POINT_ARRAY_SIZE> map_point;
    for (const geometry_msgs::msg::Point32& p : polygon.points) { // Convert to map coordinates
        worldToMap(p.x, p.y, map_point[0], map_point[1]);
        poly.push_back(
            {
                map_point[0] >= std::numeric_limits<unsigned int>::max() - 1 ? 0 : map_point[0],
                map_point[1] >= std::numeric_limits<unsigned int>::max() - 1 ? 0 : map_point[1]
            });
    }

    unsigned int right = 0;
    unsigned int left = std::numeric_limits<int>::max();
    unsigned int top = 0;
    unsigned int bottom = std::numeric_limits<int>::max();

    for (const std::array<unsigned int, 2>& point : poly) {
        right = std::max(right, point.at(0));
        left = std::min(left, point.at(0));
        top = std::max(top, point.at(1));
        bottom = std::min(bottom, point.at(1));
    }

    unsigned int x, y;
    for (x = left ; x <= right ; x ++) {
        for (y = bottom ; y <= top ; y ++) {
            if (isPointInPoly(poly, x, y)) {
                setMaskValue(x, y, value);
            }
        }
    }
}

void MapModifier::setMaskValue(const unsigned int x, const unsigned int y, const int8_t value)
{
    if (x > mask_map_width_ - 1 || y > mask_map_height_ - 1) {
        RCLCPP_WARN(get_logger(), "Trying to set value out of mask bounds (%d, %d) - Ignoring", x, y);
        return;
    }

    mask_.at(mask_map_width_ * y + x) = value;
}

void MapModifier::publishMask() const
{
    nav_msgs::msg::OccupancyGrid msg;
    msg.header.frame_id = global_frame_;
    msg.header.stamp = rclcpp::Clock().now();

    msg.info.height = mask_map_height_;
    msg.info.width = mask_map_width_;
    msg.info.resolution = mask_resolution_;
    msg.info.origin.position.x = mask_origin_.x;
    msg.info.origin.position.y = mask_origin_.y;
    msg.info.map_load_time = rclcpp::Clock().now();

    msg.data = mask_;

    RCLCPP_INFO(get_logger(), "Publishing mask filter");

    mask_filter_pub_->publish(msg);
}

void MapModifier::initialMaskCb(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    RCLCPP_INFO(get_logger(), "Initial mask received");
    added_polygons_.clear();
    mask_.clear();

    global_frame_ = msg->header.frame_id;
    mask_map_height_ = msg->info.height;
    mask_map_width_ = msg->info.width;
    mask_resolution_ = msg->info.resolution;
    mask_origin_.x = msg->info.origin.position.x;
    mask_origin_.y = msg->info.origin.position.y;

    auto get_value = [msg, this](unsigned int x, unsigned int y) {
        return msg->data.at(mask_map_width_ * y + x);
    };

    // Fill in values
    unsigned int x, y;
    int8_t value;
    cv::Mat mat(mask_map_height_, mask_map_width_, CV_8UC1);
    for (y = 0 ; y < mask_map_height_ ; y ++) {
        for (x = 0 ; x < mask_map_width_ ; x ++) {
            value = get_value(x, y);
            mask_.push_back(value);

            if (value == nav2_util::OCC_GRID_UNKNOWN) {
                value = nav2_util::OCC_GRID_FREE;
            }
            mat.at<uint8_t>(y, x) = static_cast<uint8_t>(value);
        }
    }

    // Get polygons
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mat, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    // Draw contours around contours because sometimes, the contours is too much in the polygon
    for (size_t i = 0; i < contours.size(); ++i) {
        cv::drawContours(
            mat, contours, static_cast<int>(i),
            cv::Scalar(nav2_util::OCC_GRID_OCCUPIED, nav2_util::OCC_GRID_OCCUPIED, nav2_util::OCC_GRID_OCCUPIED), 2);
    }
    cv::findContours(mat, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    RCLCPP_INFO(get_logger(), "Found %ld polygons in the mask", contours.size());

    double wx, wy;
    for (auto poly : contours) {
        geometry_msgs::msg::Polygon polygon;
        polygon.points.reserve(poly.size());
        for (cv::Point point : poly) {
            geometry_msgs::msg::Point32 point32;
            mapToWorld(point.x, point.y, wx, wy);
            // Move the point to top left to correct the fact that the polygons are moved bottom right at the creation
            if (point.x != (int) mask_map_width_ - 1 && point.y != (int) mask_map_height_ - 1) {
                wx -= mask_resolution_;
                wy -= mask_resolution_;
            }
            point32.x = wx;
            point32.y = wy;
            polygon.points.push_back(point32);
        }
        added_polygons_.push_back(polygon);
    }

    mask_filter_pub_->publish(std::move(*msg));
}

void MapModifier::worldToMap(const double wx, const double wy, unsigned int& mx, unsigned int& my)
{
    mx = worldToMapVal(wx - mask_origin_.x);
    my = worldToMapVal(wy - mask_origin_.y);
}

unsigned int MapModifier::worldToMapVal(const double val)
{
    return std::round(val / mask_resolution_);
}

void MapModifier::mapToWorld(const unsigned int mx, const unsigned int my, double& wx, double& wy)
{
    wx = mask_origin_.x + (mx + 0.5) * mask_resolution_;
    wy = mask_origin_.y + (my + 0.5) * mask_resolution_;
}

bool MapModifier::isPointInPoly(const geometry_msgs::msg::Polygon& polygon, const double x, const double y)
{
    std::vector<cv::Point2f> poly;
    poly.reserve(polygon.points.size());
    for (const geometry_msgs::msg::Point32& point : polygon.points) {
        poly.emplace_back(point.x, point.y);
    }

    const double state = cv::pointPolygonTest(poly, cv::Point2f(x, y), false);
    return state >= 0;
}

bool MapModifier::isPointInPoly(
    const std::vector<std::array<unsigned int, HRC_UTILS__POINT_ARRAY_SIZE>>& poly,
    unsigned int x, unsigned int y)
{
    geometry_msgs::msg::Polygon polygon;
    polygon.points.reserve(poly.size());
    for (const std::array<unsigned int, HRC_UTILS__POINT_ARRAY_SIZE>& point : poly) {
        geometry_msgs::msg::Point32 point32;
        point32.x = point.at(0);
        point32.y = point.at(1);
        polygon.points.push_back(point32);
    }

    return isPointInPoly(polygon, (double) x, (double) y);
}

rcl_interfaces::msg::SetParametersResult MapModifier::dynamicParametersCallback(
    std::vector<rclcpp::Parameter> parameters)
{
    for (auto param : parameters) {
        auto param_type = param.get_type();
        auto param_name = param.get_name();

        if (param_type == rclcpp::ParameterType::PARAMETER_STRING) {
            if (param_name == "mask_filter_topic") {
                mask_filter_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
                    param.as_string(),
                    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable()
                );
            }
            else if (param_name == "global_frame") {
                global_frame_ = param.as_string();
            }
        }
    }

    return rcl_interfaces::msg::SetParametersResult();
}

}


#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(hrc_map::MapModifier)
