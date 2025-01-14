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
    auto point32_to_vector = [](const geometry_msgs::msg::Point32 p) {
            return std::vector<float>({p.x, p.y});
        };

    // Remove polygons
    std::vector<float> point;
    std::vector<std::vector<float>> poly;
    for (const geometry_msgs::msg::Point32 point32 : req->points_objects_to_remove) {
        std::vector<size_t> to_be_removed;
        point = point32_to_vector(point32);
        for (size_t i = 0 ; i < added_polygons_.size() ; i ++) {
            poly = added_polygons_.at(i);
            if (isPointInPoly(poly, point.at(0), point.at(1))) {
                applyPolyValue(poly, nav2_util::OCC_GRID_FREE);
                to_be_removed.push_back(i);
            }
        }

        for (size_t idx : to_be_removed) {
            added_polygons_.erase(std::next(added_polygons_.begin(), idx));
        }
    }

    // Add polygons
    for (const geometry_msgs::msg::Polygon& polygon : req->new_objects) {
        std::vector<std::vector<float>> poly;
        for (geometry_msgs::msg::Point32 point32 : polygon.points) {
            // Move 1 cell to top so the polygon is totally taken into account
            point32.y -= mask_resolution_;
            poly.push_back(point32_to_vector(point32));
        }

        applyPolyValue(poly, nav2_util::OCC_GRID_OCCUPIED);
        added_polygons_.push_back(poly);
    }

    publishMask();
}

void MapModifier::applyPolyValue(const std::vector<std::vector<float>> polygon, const int8_t value)
{
    std::vector<std::vector<unsigned int>> poly;
    unsigned int x, y;
    for (std::vector<float> p : polygon) {
        worldToMap(p.at(0), p.at(1), x, y);
        poly.push_back(
            {
                x >= std::numeric_limits<unsigned int>::max() - 1 ? 0 : x,
                y >= std::numeric_limits<unsigned int>::max() - 1 ? 0 : y
            });
    }

    unsigned int right = 0;
    unsigned int left = std::numeric_limits<int>::max();
    unsigned int top = 0;
    unsigned int bottom = std::numeric_limits<int>::max();

    for (const std::vector<unsigned int>& point : poly) {
        right = std::max(right, point.at(0));
        left = std::min(left, point.at(0));
        top = std::max(top, point.at(1));
        bottom = std::min(bottom, point.at(1));
    }

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
    msg.info.origin.position.x = mask_origin_.at(0);
    msg.info.origin.position.y = mask_origin_.at(1);
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
    mask_origin_.clear();

    global_frame_ = msg->header.frame_id;
    mask_map_height_ = msg->info.height;
    mask_map_width_ = msg->info.width;
    mask_resolution_ = msg->info.resolution;
    mask_origin_.push_back(msg->info.origin.position.x);
    mask_origin_.push_back(msg->info.origin.position.y);

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
        std::vector<std::vector<float>> points;
        for (cv::Point point : poly) {
            mapToWorld(point.x, point.y, wx, wy);
            // Move the point to top left to correct the fact that the polygons are moved bottom right at the creation
            if (point.x != (int) mask_map_width_ - 1 && point.y != (int) mask_map_height_ - 1) {
                wx -= mask_resolution_;
                wy -= mask_resolution_;
            }
            points.emplace_back(std::vector<float>({(float) wx, (float) wy}));
        }
        added_polygons_.push_back(points);
    }

    mask_filter_pub_->publish(std::move(*msg));
}

void MapModifier::worldToMap(const double wx, const double wy, unsigned int& mx, unsigned int& my)
{
    mx = worldToMapVal(wx - mask_origin_.at(0));
    my = worldToMapVal(wy - mask_origin_.at(1));
}

unsigned int MapModifier::worldToMapVal(const double val)
{
    return std::round(val / mask_resolution_);
}

void MapModifier::mapToWorld(const unsigned int mx, const unsigned int my, double& wx, double& wy)
{
    wx = mask_origin_.at(0) + (mx + 0.5) * mask_resolution_;
    wy = mask_origin_.at(1) + (my + 0.5) * mask_resolution_;
}

template<typename T>
bool MapModifier::isPointInPoly(const std::vector<std::vector<T>> polygon, const double x, const double y)
{
    std::vector<cv::Point2f> poly;
    for (auto point : polygon) {
        cv::Point2f p(point.at(0), point.at(1));
        poly.push_back(p);
    }

    const double state = cv::pointPolygonTest(poly, cv::Point2f(x, y), false);
    return state >= 0;
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
