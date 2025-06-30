#ifndef HRC_MAP_MODIFIER_HPP
#define HRC_MAP_MODIFIER_HPP

#include "rclcpp/node.hpp"
#include "hrc_interfaces/srv/manage_objects_map.hpp"
#include "hrc_interfaces/srv/get_robot_pose.hpp"
#include "hrc_utils/utils.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include <vector>
#include <string>

#define HRC_MAP__MAP_RELATIVE_POINTS nullptr

namespace hrc_map
{

/**
 * @brief MapModifier publishes a keepout_filter mask.
 * It listens on the service 'manage_object_map' to add objects to the mask or can remove objects that has been added.
 */
class MapModifier : public rclcpp::Node
{
public:
    MapModifier(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

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

    /**
     * @brief Create a mask from the msg and publishes it
     */
    void initialMaskCb(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

protected:
    std::string global_frame_;
    double mask_resolution_;
    geometry_msgs::msg::Point mask_origin_;
    unsigned int mask_map_width_;
    unsigned int mask_map_height_;
    std::vector<int8_t> mask_;
    std::vector<geometry_msgs::msg::Polygon> added_polygons_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr mask_filter_pub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr elements_mask_sub_;
    std::shared_ptr<rclcpp::Service<hrc_interfaces::srv::ManageObjectsMap>> manage_objects_map_srv_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameters_handle_;
    rclcpp::Client<hrc_interfaces::srv::GetRobotPose>::SharedPtr get_robot_pose_client_;

    /**
     * @brief Apply polygon to the mask with the given value
     * @param polygon The polygon to apply
     * @param value The value to fill
     */
    void applyPolyValue(const geometry_msgs::msg::Polygon& polygon, const int8_t value);

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
     * @brief Convert from map coordinate to world coordinate.
     * @param mx Map coordinate along x-axis (in cells)
     * @param my Map coordinate along y-axis (in cells)
     * @param wx World coordinate along x-axis (in meters)
     * @param wy World coordinate along y-axis (in meters)
     */
    void mapToWorld(const unsigned int mx, const unsigned int my, double& wx, double& wy);

    /**
     * @brief Check whether a point is in a polygon
     * @param poly The polygon
     * @param x The X coordinate of the point
     * @param y The Y coordinate of the point
     * @return Whether the point is in the polygon
     */
    bool isPointInPoly(const geometry_msgs::msg::Polygon& poly, const double x, const double y);
    bool isPointInPoly(
        const std::vector<std::array<unsigned int, HRC_UTILS__POINT_ARRAY_SIZE>>& poly, unsigned int x, unsigned int y);

    /**
     * @brief Remove the polygons that are to be removed from the map
     * @param points_objects_to_remove The points where the objects are to be removed
     * @param robot_pose The robot pose to use if the points are in robot coordinates.
     * If the points are in the map coordinates, set the parameter to nullptr.
     */
    void removePolygons(
        const std::vector<geometry_msgs::msg::Point32>& points_objects_to_remove,
        const geometry_msgs::msg::Pose2D* robot_pose);

    /**
     * @brief Add the polygons to the map
     * @param new_polygon The polygns to add
     * @param robot_pose The robot pose to use if the points are in robot coordinates.
     * If the points are in the map coordinates, set the parameter to nullptr.
     */
    void addPolygons(
        const std::vector<geometry_msgs::msg::Polygon>& new_polygon, const geometry_msgs::msg::Pose2D* robot_pose);
};

}

#endif // HRC_MAP_MODIFIER_HPP
