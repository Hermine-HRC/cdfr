/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020 Samsung Research Russia
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the <ORGANIZATION> nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Alexey Merzlyakov
 *********************************************************************/

#include <string>
#include <memory>
#include <algorithm>
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "herminebot_navigation/costmap_keepout_filter.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"

namespace hrc_costmap_2d
{

KeepoutFilter::KeepoutFilter()
: filter_info_sub_(nullptr), mask_sub_(nullptr), mask_costmap_(nullptr),
    mask_frame_(""), global_frame_("")
{
}

void KeepoutFilter::initializeFilter(
    const std::string & filter_info_topic)
{
    std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

    rclcpp_lifecycle::LifecycleNode::SharedPtr node = node_.lock();
    if (!node) {
        throw std::runtime_error{"Failed to lock node"};
    }

    filter_info_topic_ = filter_info_topic;
    // Setting new costmap filter info subscriber
    RCLCPP_INFO(
        logger_,
        "KeepoutFilter: Subscribing to \"%s\" topic for filter info...",
        filter_info_topic_.c_str());
    filter_info_sub_ = node->create_subscription<nav2_msgs::msg::CostmapFilterInfo>(
        filter_info_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
        std::bind(&KeepoutFilter::filterInfoCallback, this, std::placeholders::_1));

    global_frame_ = layered_costmap_->getGlobalFrameID();

    declareParameter("inflation_radius", rclcpp::ParameterValue(0.1));

    node->get_parameter("robot_radius", robot_radius_);
    node->get_parameter(name_ + ".inflation_radius", inflation_radius_);
}

void KeepoutFilter::filterInfoCallback(
  const nav2_msgs::msg::CostmapFilterInfo::SharedPtr msg)
{
    std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

    rclcpp_lifecycle::LifecycleNode::SharedPtr node = node_.lock();
    if (!node) {
        throw std::runtime_error{"Failed to lock node"};
    }

    if (!mask_sub_) {
        RCLCPP_INFO(
        logger_,
        "KeepoutFilter: Received filter info from %s topic.", filter_info_topic_.c_str());
    } else {
        RCLCPP_WARN(
        logger_,
        "KeepoutFilter: New costmap filter info arrived from %s topic. Updating old filter info.",
        filter_info_topic_.c_str());
        // Resetting previous subscriber each time when new costmap filter information arrives
        mask_sub_.reset();
    }

    // Checking that base and multiplier are set to their default values
    if (msg->base != nav2_costmap_2d::BASE_DEFAULT || msg->multiplier != nav2_costmap_2d::MULTIPLIER_DEFAULT) {
        RCLCPP_ERROR(
        logger_,
        "KeepoutFilter: For proper use of keepout filter base and multiplier"
        " in CostmapFilterInfo message should be set to their default values (%f and %f)",
        nav2_costmap_2d::BASE_DEFAULT, nav2_costmap_2d::MULTIPLIER_DEFAULT);
    }

    mask_topic_ = msg->filter_mask_topic;

    // Setting new filter mask subscriber
    RCLCPP_INFO(
        logger_,
        "KeepoutFilter: Subscribing to \"%s\" topic for filter mask...",
        mask_topic_.c_str());
    mask_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
        mask_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
        std::bind(&KeepoutFilter::maskCallback, this, std::placeholders::_1));
}

void KeepoutFilter::maskCallback(
  const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

    rclcpp_lifecycle::LifecycleNode::SharedPtr node = node_.lock();
    if (!node) {
        throw std::runtime_error{"Failed to lock node"};
    }

    if (!mask_costmap_) {
        RCLCPP_INFO(
        logger_,
        "KeepoutFilter: Received filter mask from %s topic.", mask_topic_.c_str());
    } else {
        RCLCPP_WARN(
        logger_,
        "KeepoutFilter: New filter mask arrived from %s topic. Updating old filter mask.",
        mask_topic_.c_str());
        mask_costmap_.reset();
    }

    // Making a new mask_costmap_
    mask_costmap_ = std::make_unique<nav2_costmap_2d::Costmap2D>(*msg);
    mask_frame_ = msg->header.frame_id;
}

void KeepoutFilter::process(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j,
    const geometry_msgs::msg::Pose2D & /*pose*/)
{
    std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

    if (!mask_costmap_) {
        // Show warning message every 2 seconds to not litter an output
        RCLCPP_WARN_THROTTLE(
        logger_, *(clock_), 2000,
        "KeepoutFilter: Filter mask was not received");
        return;
    }

    tf2::Transform tf2_transform;
    tf2_transform.setIdentity();  // initialize by identical transform
    int mg_min_x, mg_min_y;  // masger_grid indexes of bottom-left window corner
    int mg_max_x, mg_max_y;  // masger_grid indexes of top-right window corner

    if (mask_frame_ != global_frame_) {
        // Filter mask and current layer are in different frames:
        // prepare frame transformation if mask_frame_ != global_frame_
        geometry_msgs::msg::TransformStamped transform;
        try {
        transform = tf_->lookupTransform(
            mask_frame_, global_frame_, tf2::TimePointZero,
            transform_tolerance_);
        } catch (tf2::TransformException & ex) {
        RCLCPP_ERROR(
            logger_,
            "KeepoutFilter: Failed to get costmap frame (%s) "
            "transformation to mask frame (%s) with error: %s",
            global_frame_.c_str(), mask_frame_.c_str(), ex.what());
        return;
        }
        tf2::fromMsg(transform.transform, tf2_transform);

        mg_min_x = min_i;
        mg_min_y = min_j;
        mg_max_x = max_i;
        mg_max_y = max_j;
    } else {
        // Filter mask and current layer are in the same frame:
        // apply the following optimization - iterate only in overlapped
        // (min_i, min_j)..(max_i, max_j) & mask_costmap_ area.
        //
        //           mask_costmap_
        //       *----------------------------*
        //       |                            |
        //       |                            |
        //       |      (2)                   |
        // *-----+-------*                    |
        // |     |///////|<- overlapped area  |
        // |     |///////|   to iterate in    |
        // |     *-------+--------------------*
        // |    (1)      |
        // |             |
        // *-------------*
        //  master_grid (min_i, min_j)..(max_i, max_j) window
        //
        // ToDo: after costmap rotation will be added, this should be re-worked.

        double wx, wy;  // world coordinates

        // Calculating bounds corresponding to bottom-left overlapping (1) corner
        // mask_costmap_ -> master_grid intexes conversion
        const double half_cell_size = 0.5 * mask_costmap_->getResolution();
        wx = mask_costmap_->getOriginX() + half_cell_size;
        wy = mask_costmap_->getOriginY() + half_cell_size;
        master_grid.worldToMapNoBounds(wx, wy, mg_min_x, mg_min_y);
        // Calculation of (1) corner bounds
        if (mg_min_x >= max_i || mg_min_y >= max_j) {
            // There is no overlapping. Do nothing.
            return;
        }
        mg_min_x = std::max(min_i, mg_min_x);
        mg_min_y = std::max(min_j, mg_min_y);

        // Calculating bounds corresponding to top-right window (2) corner
        // mask_costmap_ -> master_grid intexes conversion
        wx = mask_costmap_->getOriginX() +
            mask_costmap_->getSizeInCellsX() * mask_costmap_->getResolution() + half_cell_size;
        wy = mask_costmap_->getOriginY() +
            mask_costmap_->getSizeInCellsY() * mask_costmap_->getResolution() + half_cell_size;
        master_grid.worldToMapNoBounds(wx, wy, mg_max_x, mg_max_y);
        // Calculation of (2) corner bounds
        if (mg_max_x <= min_i || mg_max_y <= min_j) {
            // There is no overlapping. Do nothing.
            return;
        }
        mg_max_x = std::min(max_i, mg_max_x);
        mg_max_y = std::min(max_j, mg_max_y);
    }

    // unsigned<-signed conversions.
    unsigned const int mg_min_x_u = static_cast<unsigned int>(mg_min_x);
    unsigned const int mg_min_y_u = static_cast<unsigned int>(mg_min_y);
    unsigned const int mg_max_x_u = static_cast<unsigned int>(mg_max_x);
    unsigned const int mg_max_y_u = static_cast<unsigned int>(mg_max_y);

    unsigned int i, j;  // master_grid iterators
    unsigned int index;  // corresponding index of master_grid
    double gl_wx, gl_wy;  // world coordinates in a global_frame_
    double msk_wx, msk_wy;  // world coordinates in a mask_frame_
    unsigned int mx, my;  // mask_costmap_ coordinates
    unsigned char data, old_data;  // master_grid element data

    // Main master_grid updating loop
    // Iterate in costmap window by master_grid indexes
    unsigned char * master_array = master_grid.getCharMap();
    for (i = mg_min_x_u; i < mg_max_x_u; i++) {
        for (j = mg_min_y_u; j < mg_max_y_u; j++) {
            index = master_grid.getIndex(i, j);
            old_data = master_array[index];
            // Calculating corresponding to (i, j) point at mask_costmap_:
            // Get world coordinates in global_frame_
            master_grid.mapToWorld(i, j, gl_wx, gl_wy);
            if (mask_frame_ != global_frame_) {
                // Transform (i, j) point from global_frame_ to mask_frame_
                tf2::Vector3 point(gl_wx, gl_wy, 0);
                point = tf2_transform * point;
                msk_wx = point.x();
                msk_wy = point.y();
            } else {
                // In this case master_grid and filter-mask are in the same frame
                msk_wx = gl_wx;
                msk_wy = gl_wy;
            }
            // Get mask coordinates corresponding to (i, j) point at mask_costmap_
            if (mask_costmap_->worldToMap(msk_wx, msk_wy, mx, my)) {
                data = mask_costmap_->getCost(mx, my);
                // Update if mask_ data is valid and greater than existing master_grid's one
                if (data == nav2_costmap_2d::NO_INFORMATION) {
                    continue;
                }
                if (data > old_data || old_data == nav2_costmap_2d::NO_INFORMATION) {
                    master_array[index] = data;
                    if (data == nav2_costmap_2d::LETHAL_OBSTACLE) {
                        applyRobotRadius(master_grid, master_array, i, j);
                    }
                }
            }
        }
    }
}

void KeepoutFilter::applyRobotRadius(
    nav2_costmap_2d::Costmap2D& master_grid, 
    unsigned char* master_array, 
    const unsigned int x_center, 
    const unsigned int y_center
){
    const unsigned int pix_robot_radius = master_grid.cellDistance(robot_radius_);
    const unsigned int sq_pix_robot_radius = pix_robot_radius * pix_robot_radius;
    const unsigned int pix_inflation_radius = master_grid.cellDistance(robot_radius_ + inflation_radius_);
    const unsigned int sq_pix_inflation_radius = pix_inflation_radius * pix_inflation_radius;
    unsigned int x, y, dx, dy, idx, start_x, end_x, start_y, end_y, sq_dist;

    start_x = std::max(0, (int) (x_center - pix_inflation_radius));
    start_y = std::max(0, (int) (y_center - pix_inflation_radius));
    end_x = std::min((int) (x_center + pix_inflation_radius), (int) master_grid.getSizeInCellsX() - 1);
    end_y = std::min((int) (y_center + pix_inflation_radius), (int) master_grid.getSizeInCellsY() - 1);
 
    for (x = start_x ; x < end_x ; x ++) {
        for (y = start_y ; y < end_y ; y ++) {
            dx = x - x_center;
            dy = y - y_center;
            idx = master_grid.getIndex(x, y);
            sq_dist = dx * dx + dy * dy;
            if (sq_dist < sq_pix_robot_radius && master_array[idx] != nav2_costmap_2d::LETHAL_OBSTACLE) {
                master_array[idx] = nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
            }
            else if (sq_dist < sq_pix_inflation_radius && master_array[idx] < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
                master_array[idx] = nav2_costmap_2d::MAX_NON_OBSTACLE;
            }
        }
    }
}

void KeepoutFilter::resetFilter()
{
    std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

    filter_info_sub_.reset();
    mask_sub_.reset();
}

bool KeepoutFilter::isActive()
{
    std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

    if (mask_costmap_) {
        return true;
    }
    return false;
}

}  // namespace hrc_costmap_2d

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(hrc_costmap_2d::KeepoutFilter, nav2_costmap_2d::Layer)
