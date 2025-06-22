#ifndef HRC_BT_UTILS_HPP
#define HRC_BT_UTILS_HPP

#include <vector>

#include "behaviortree_cpp/behavior_tree.h"
#include "geometry_msgs/msg/point.hpp"
#include "hrc_utils/utils.hpp"
#include "nav2_util/array_parser.hpp"

namespace BT
{

/**
 * @brief Converts a string to a vector of Points
 * The string must have the following format:
 * "[[x1, y1], [x2, y2], ...]"
 */
template<>
inline std::vector<geometry_msgs::msg::Point> convertFromString(const StringView str)
{
    std::string error;
    std::string str_str = std::string(str);
    std::replace(str_str.begin(), str_str.end(), '\n', ' ');
    std::vector<std::vector<float>> result = nav2_util::parseVVF(str_str, error);
    if (!error.empty()) {
        throw std::runtime_error(error);
    }

    std::vector<geometry_msgs::msg::Point> res;
    res.reserve(result.size());
    geometry_msgs::msg::Point point;
    for (const std::vector<float>& p : result) {
        point.x = p.at(0);
        point.y = p.at(1);
        res.push_back(point);
    }
    return res;
}

/**
 * @brief Converts a string to a vector of vectors of Points
 * The string must have the following format:
 * "[[x1, y1], [x2, y2], ...]; [[x1, y1], [x2, y2], ...]; ..."
 */
template<>
inline std::vector<std::vector<geometry_msgs::msg::Point>> convertFromString(const StringView str)
{
    std::vector<StringView> polygons = splitString(str, ';');
    std::vector<std::vector<geometry_msgs::msg::Point>> result;
    result.reserve(polygons.size());

    for (StringView polygon : polygons) {
        auto points = convertFromString<std::vector<geometry_msgs::msg::Point>>(polygon);
        result.push_back(points);
    }
    return result;
}

}

#endif  // HRC_BT_UTILS_HPP
