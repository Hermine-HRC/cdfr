#include "herminebot_navigation/map_modifier.hpp"
#include "nav2_util/occ_grid_values.hpp"
#include "geometry_msgs/msg/point32.hpp"

#include <algorithm>
#include <memory>

namespace hrc_map
{

MapModifier::MapModifier(const rclcpp::NodeOptions &options) : rclcpp::Node("map_modifier", options)
{
    declare_parameter("mask_filter_topic", rclcpp::ParameterValue("/mask_filter"));
    declare_parameter("global_frame", rclcpp::ParameterValue("map"));
    declare_parameter("mask_width", rclcpp::ParameterValue(3.0));
    declare_parameter("mask_height", rclcpp::ParameterValue(2.0));
    declare_parameter("mask_resolution", rclcpp::ParameterValue(0.1));
    declare_parameter("mask_origin", rclcpp::PARAMETER_DOUBLE_ARRAY);

    std::string mask_filter_topic;
    double mask_width, mask_height;
    get_parameter("mask_filter_topic", mask_filter_topic);
    get_parameter("mask_width", mask_width);
    get_parameter("mask_height", mask_height);
    get_parameter("global_frame", global_frame_);
    get_parameter("mask_resolution", mask_resolution_);
    get_parameter("mask_origin", mask_origin_);

    mask_map_width_ = worldToMapVal(mask_width);
    mask_map_height_ = worldToMapVal(mask_height);

    for (size_t i  = 0 ; i < mask_map_height_ * mask_map_width_ ; i ++) {
        mask_.push_back(nav2_util::OCC_GRID_FREE);
    }

    srv_ = create_service<hrc_interfaces::srv::ManageObjectsMap>(
        "manage_object_map", 
        std::bind(&MapModifier::manageObjectsCb, this, std::placeholders::_1, std::placeholders::_2)
    );

    mask_filter_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        mask_filter_topic, 
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable()
    );

    parameters_handle_ = add_on_set_parameters_callback(std::bind(
        &MapModifier::dynamicParametersCallback,
        this,
        std::placeholders::_1
    ));

    RCLCPP_INFO(
        get_logger(), 
        "Node ready. Will publish mask filter of size (%.3f, %.3f) "
        "meters with resolution %.3f on topic '%s'",
        mask_width, mask_height, mask_resolution_, mask_filter_topic.c_str()
    );

    publishMask();
}

void MapModifier::manageObjectsCb(
    const std::shared_ptr<hrc_interfaces::srv::ManageObjectsMap::Request> req, 
    const std::shared_ptr<hrc_interfaces::srv::ManageObjectsMap::Response> /*res*/
) {
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
        poly.push_back({x, y});
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
        RCLCPP_WARN(get_logger(), "Trying to set value out of mask bunds (%d, %d) - Ignoring", x, y);
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

void MapModifier::worldToMap(const double wx, const double wy, unsigned int& mx, unsigned int& my) 
{
    mx = worldToMapVal(wx - mask_origin_.at(0));
    my = worldToMapVal(wy - mask_origin_.at(1));
}

unsigned int MapModifier::worldToMapVal(const double val)
{
    return std::round(val / mask_resolution_);
}

template<typename T>
bool MapModifier::isPointInPoly(const std::vector<std::vector<T>> poly, const double x, const double y)
{
    // Adaptation of Shimrat, Moshe. "Algorithm 112: position of point relative to polygon."
    // Communications of the ACM 5.8 (1962): 434.
    // Implementation of ray crossings algorithm for point in polygon task solving.
    // Y coordinate is fixed. Moving the ray on X+ axis starting from given point.
    // Odd number of intersections with polygon boundaries means the point is inside polygon. 

    const int poly_size = poly.size();
    int i, j;  // Polygon vertex iterators
    bool res = false;  // Final result, initialized with already inverted value

    // Starting from the edge where the last point of polygon is connected to the first
    i = poly_size - 1;
    for (j = 0; j < poly_size; j++) {
        // Checking the edge only if given point is between edge boundaries by Y coordinates.
        // One of the condition should contain equality in order to exclude the edges
        // parallel to X+ ray.
        if ((y < poly[i].at(1)) == (y >= poly[j].at(1))) {
            // Calculating the intersection coordinate of X+ ray
            const double x_inter = poly[i].at(0) +
                (y - poly[i].at(1)) * (poly[j].at(0) - poly[i].at(0)) /
                (poly[j].at(1) - poly[i].at(1));
            // If intersection with checked edge is greater than point.x coordinate, inverting the result
            if (x_inter > x) {
                res = !res;
            }
        }
        i = j;
    }
    
    return res;
}

rcl_interfaces::msg::SetParametersResult MapModifier::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
    for (auto param : parameters) {
        auto param_type = param.get_type();
        auto param_name=  param.get_name();

        if (param_type == rclcpp::ParameterType::PARAMETER_STRING){ 
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
