#ifndef HRC_MAP_MODIFIER_HPP
#define HRC_MAP_MODIFIER_HPP

#include "rclcpp/node.hpp"
#include "hrc_interfaces/srv/manage_objects_map.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include <vector>
#include <string>

namespace hrc_map
{

/**
 * @brief MapModifier publishes a keepout_filter mask.
 * It listens on the service 'manage_object_map' to add objects to the mask or can remove objects that has been added.
 */
class MapModifier : public rclcpp::Node
{
public:
    MapModifier(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    /**
     * @brief Remove the objects to be removed and add those to be added to the mask
     * @param req The request
     * @param res The response
     */
    void manageObjectsCb(
        const std::shared_ptr<hrc_interfaces::srv::ManageObjectsMap::Request> request, 
        const std::shared_ptr<hrc_interfaces::srv::ManageObjectsMap::Response> /*response*/
    );

    /**
     * @brief Callback executed when a parameter change is detected
     * @param parameters ParameterEvent message
     */
    rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

protected:
    std::string global_frame_;
    double mask_resolution_;
    std::vector<double> mask_origin_;
    unsigned int mask_map_width_;
    unsigned int mask_map_height_;
    std::vector<int8_t> mask_;
    std::vector<std::vector<std::vector<float>>> added_polygons_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr mask_filter_pub_;
    std::shared_ptr<rclcpp::Service<hrc_interfaces::srv::ManageObjectsMap>> srv_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameters_handle_;

    /**
     * @brief Apply polygon to the mask with the given value
     * @param polygon The polygon to apply
     * @param value The value to fill
     */
    void applyPolyValue(const std::vector<std::vector<float>> polygon, const int8_t value);

    /**
     * @brief Set the value at given map coordinates
     * @param x The X coordinate
     * @param y The Y coodinate
     * @param value The value to set
     */
    void setMaskValue(const unsigned int x, const unsigned int y, const int8_t value);

    /**
     * @brief Publish the mask filter
     */
    void publishMask() const;

    /**
     * @brief Convert from world coordinate to map coordinate.
     * @param wx World coordinate along x-axis (in meters)
     * @param wy World coordinate along y-axis (in meters)
     * @param mx Map coordinate along x-axis (in cells)
     * @param my Map coordinate along y-axis (in cells)
     */
    void worldToMap(const double wx, const double wy, unsigned int& mx, unsigned int& my);

    /**
     * @brief Convert from world coordinate to map coordinate.
     * @param val World value (in meters)
     * @return  Map value in cells
     */
    unsigned int worldToMapVal(const double val);

    /**
     * @brief Check whether a point is in a polygon
     * @param poly The polygon
     * @param x The X coordinate of the point
     * @param y The Y coordinate of the point
     * @return Whether the point is in the polygon
     */
    template<typename T>
    bool isPointInPoly(const std::vector<std::vector<T>> poly, const double x, const double y);
};

}

#endif // HRC_MAP_MODIFIER_HPP
