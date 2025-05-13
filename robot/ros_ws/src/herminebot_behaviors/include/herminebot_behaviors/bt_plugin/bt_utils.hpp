#ifndef HRC_BT_UTILS_HPP
#define HRC_BT_UTILS_HPP

#include <vector>

#include "behaviortree_cpp/behavior_tree.h"
#include "nav2_util/array_parser.hpp"

namespace BT
{

/**
 * @brief Converts a string to a vector of vectors
 * The string must have the following format:
 * "[[x1, y1], [x2, y2], ...]"
 */
template<>
inline std::vector<std::vector<double>> convertFromString(const StringView str)
{
    std::string error;
    std::string str_str = std::string(str);
    std::replace(str_str.begin(), str_str.end(), '\n', ' ');
    std::vector<std::vector<float>> result = nav2_util::parseVVF(str_str, error);
    if (!error.empty()) {
        throw std::runtime_error(error);
    }

    std::vector<std::vector<double>> res;
    for (const std::vector<float>& p : result) {
        std::vector<double> d;
        for (const float& c : p) {
            d.push_back((double) c);
        }
        res.push_back(d);
    }
    return res;
}

/**
 * @brief Converts a string to a vector of vectors of vectors
 * The string must have the following format:
 * "[[x1, y1], [x2, y2], ...]; [[x1, y1], [x2, y2], ...]; ..."
 */
template<>
inline std::vector<std::vector<std::vector<double>>> convertFromString(const StringView str)
{
    std::vector<std::vector<std::vector<double>>> result;
    for (StringView polygon : splitString(str, ';')) {
        auto points = convertFromString<std::vector<std::vector<double>>>(polygon);
        result.push_back(points);
    }
    return result;
}

}

#endif  // HRC_BT_UTILS_HPP
