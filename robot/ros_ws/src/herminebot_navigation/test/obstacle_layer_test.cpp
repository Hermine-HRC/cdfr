/*
 * Copyright (c) 2013, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @author David Lu!!
 * Test harness for ObstacleLayer for Costmap2D
 */

#include <memory>
#include <string>
#include <algorithm>
#include <utility>

#include "gtest/gtest.h"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/observation_buffer.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/static_layer.hpp"
#include "herminebot_navigation/costmap_obstacle_layer.hpp"

using std::begin;
using std::end;
using std::for_each;
using std::all_of;
using std::none_of;
using std::pair;
using std::string;

const double MAX_Z(1.0);

class RclCppFixture
{
public:
    RclCppFixture() {rclcpp::init(0, nullptr);}
    ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

class TestLifecycleNode : public nav2_util::LifecycleNode
{
public:
    explicit TestLifecycleNode(const string & name)
      : nav2_util::LifecycleNode(name)
    {
    }

    nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &)
    {
        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &)
    {
        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &)
    {
        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
    {
        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn onShutdown(const rclcpp_lifecycle::State &)
    {
        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn onError(const rclcpp_lifecycle::State &)
    {
        return nav2_util::CallbackReturn::SUCCESS;
    }
};

class TestObstacleLayer : public hrc_costmap_2d::ObstacleLayer
{
public:
    void setPolygon(std::vector<std::pair<double, double>> poly)
    {
        poly_ = poly;
    }

    void setInflationRadius(double radius)
    {
        inflation_radius_ = radius;
    }
};

class TestNode : public ::testing::Test
{
public:
    TestNode()
    {
        node_ = std::make_shared<TestLifecycleNode>("obstacle_test_node");
        node_->declare_parameter("map_topic", rclcpp::ParameterValue(std::string("map")));
        node_->declare_parameter("track_unknown_space", rclcpp::ParameterValue(false));
        node_->declare_parameter("use_maximum", rclcpp::ParameterValue(false));
        node_->declare_parameter("lethal_cost_threshold", rclcpp::ParameterValue(100));
        node_->declare_parameter(
            "unknown_cost_value",
            rclcpp::ParameterValue(static_cast<unsigned char>(0xff)));
        node_->declare_parameter("trinary_costmap", rclcpp::ParameterValue(true));
        node_->declare_parameter("transform_tolerance", rclcpp::ParameterValue(0.3));
        node_->declare_parameter("observation_sources", rclcpp::ParameterValue(std::string("")));
    }

    ~TestNode() {}

protected:
    std::shared_ptr<TestLifecycleNode> node_;
};

void addObstacleLayer(
    nav2_costmap_2d::LayeredCostmap & layers,
    tf2_ros::Buffer & tf, nav2_util::LifecycleNode::SharedPtr node,
    std::shared_ptr<hrc_costmap_2d::ObstacleLayer> & olayer,
    rclcpp::CallbackGroup::SharedPtr callback_group = nullptr)
{
    olayer = std::make_shared<hrc_costmap_2d::ObstacleLayer>();
    olayer->initialize(&layers, "obstacles", &tf, node, callback_group);
    layers.addPlugin(std::shared_ptr<nav2_costmap_2d::Layer>(olayer));
}

void addStaticLayer(
    nav2_costmap_2d::LayeredCostmap & layers,
    tf2_ros::Buffer & tf, nav2_util::LifecycleNode::SharedPtr node,
    std::shared_ptr<nav2_costmap_2d::StaticLayer> & slayer,
    rclcpp::CallbackGroup::SharedPtr callback_group = nullptr)
{
    slayer = std::make_shared<nav2_costmap_2d::StaticLayer>();
    layers.addPlugin(std::shared_ptr<nav2_costmap_2d::Layer>(slayer));
    slayer->initialize(&layers, "static", &tf, node, callback_group);
}


void addObservation(
    std::shared_ptr<hrc_costmap_2d::ObstacleLayer> olayer, double x, double y, double z = 0.0,
    double ox = 0.0, double oy = 0.0, double oz = MAX_Z, bool marking = true, bool clearing = true,
    double raytrace_max_range = 100.0,
    double raytrace_min_range = 0.0,
    double obstacle_max_range = 100.0,
    double obstacle_min_range = 0.0)
{
    sensor_msgs::msg::PointCloud2 cloud;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(1);
    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
    *iter_x = x;
    *iter_y = y;
    *iter_z = z;

    geometry_msgs::msg::Point p;
    p.x = ox;
    p.y = oy;
    p.z = oz;

    nav2_costmap_2d::Observation obs(p, cloud, obstacle_max_range, obstacle_min_range,
        raytrace_max_range, raytrace_min_range);
    olayer->addStaticObservation(obs, marking, clearing);
}

unsigned int countValues(
    nav2_costmap_2d::Costmap2D & costmap,
    unsigned char value, bool equal = true)
{
    unsigned int count = 0;
    for (unsigned int i = 0; i < costmap.getSizeInCellsY(); i++) {
        for (unsigned int j = 0; j < costmap.getSizeInCellsX(); j++) {
            unsigned char c = costmap.getCost(j, i);
            if ((equal && c == value) || (!equal && c != value)) {
                count += 1;
            }
        }
    }
    return count;
}

/*
 * For reference, the static map looks like this:
 *
 *   0   0   0   0   0   0   0 254 254 254
 *
 *   0   0   0   0   0   0   0 254 254 254
 *
 *   0   0   0 254 254 254   0   0   0   0
 *
 *   0   0   0   0   0   0   0   0   0   0
 *
 *   0   0   0   0   0   0   0   0   0   0
 *
 *   0   0   0   0 254   0   0 254 254 254
 *
 *   0   0   0   0 254   0   0 254 254 254
 *
 *   0   0   0   0   0   0   0 254 254 254
 *
 *   0   0   0   0   0   0   0   0   0   0
 *
 *   0   0   0   0   0   0   0   0   0   0
 *
 *   upper left is 0,0, lower right is 9,9
 */


/**
 * Verify correct init/reset cycling of layer
 */
TEST_F(TestNode, testRepeatedResets) {
    tf2_ros::Buffer tf(node_->get_clock());
    nav2_costmap_2d::LayeredCostmap layers("frame", false, false);

    std::shared_ptr<nav2_costmap_2d::StaticLayer> slayer = nullptr;
    addStaticLayer(layers, tf, node_, slayer);

    // TODO(orduno) Add obstacle layer

    // Define a node-level parameter
    pair<string, string> node_dummy = {"node_dummy_param", "node_dummy_val"};
    node_->declare_parameter(node_dummy.first, rclcpp::ParameterValue(node_dummy.second));

    // Define a layer-level parameter
    pair<string, string> layer_dummy = {"dummy_param", "dummy_val"};

    // Set parameters
    auto plugins = layers.getPlugins();
    for_each(
        begin(*plugins), end(*plugins), [&layer_dummy](const auto & plugin) {
            string layer_param = layer_dummy.first + "_" + plugin->getName();

            // Notice we are using Layer::declareParameter
            plugin->declareParameter(layer_param, rclcpp::ParameterValue(layer_dummy.second));
        });

    // Check that all parameters have been set
    // node-level param
    ASSERT_TRUE(node_->has_parameter(node_dummy.first));

    // layer-level param
    ASSERT_TRUE(
        all_of(
            begin(*plugins), end(*plugins), [&layer_dummy](const auto & plugin) {
                string layer_param = layer_dummy.first + "_" + plugin->getName();
                return plugin->hasParameter(layer_param);
            }));

    // Reset all layers. Parameters should be declared if not declared, otherwise skipped.
    ASSERT_NO_THROW(
        for_each(
            begin(*plugins), end(*plugins), [](const auto & plugin) {
                plugin->reset();
            }));
}


/**
 * Test for ray tracing free space
 */
TEST_F(TestNode, testRaytracing) {
    tf2_ros::Buffer tf(node_->get_clock());

    nav2_costmap_2d::LayeredCostmap layers("frame", false, false);
    layers.resizeMap(10, 10, 1, 0, 0);

    std::shared_ptr<nav2_costmap_2d::StaticLayer> slayer = nullptr;
    addStaticLayer(layers, tf, node_, slayer);
    std::shared_ptr<hrc_costmap_2d::ObstacleLayer> olayer = nullptr;
    addObstacleLayer(layers, tf, node_, olayer);

    addObservation(olayer, 0.0, 0.0, MAX_Z / 2, 0, 0, MAX_Z / 2);

    // This actually puts the LETHAL (254) point in the costmap at (0,0)
    layers.updateMap(0, 0, 0);  // 0, 0, 0 is robot pose
    // printMap(*(layers.getCostmap()));

    int lethal_count = countValues(*(layers.getCostmap()), nav2_costmap_2d::LETHAL_OBSTACLE);

    ASSERT_EQ(lethal_count, 1);

    addObservation(olayer, 1.0, 1.0, MAX_Z / 2, 0, 0, MAX_Z / 2, true, true, 100.0, 5.0, 100.0, 5.0);

    // This actually puts the LETHAL (254) point in the costmap at (0,0)
    layers.updateMap(0, 0, 0);  // 0, 0, 0 is robot pose
    // printMap(*(layers.getCostmap()));

    // New observation should not be recorded as min_range is higher than obstacle range
    lethal_count = countValues(*(layers.getCostmap()), nav2_costmap_2d::LETHAL_OBSTACLE);

    ASSERT_EQ(lethal_count, 1);
}

/**
 * Test dynamic parameter setting of obstacle layer
 */
TEST_F(TestNode, testDynParamsSetObstacle)
{
    auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");

    // Add obstacle layer
    std::vector<std::string> plugins_str;
    plugins_str.push_back("obstacle_layer");
    costmap->set_parameter(rclcpp::Parameter("plugins", plugins_str));
    costmap->declare_parameter(
        "obstacle_layer.plugin",
        rclcpp::ParameterValue(std::string("hrc_costmap_2d::ObstacleLayer")));

    costmap->set_parameter(rclcpp::Parameter("global_frame", std::string("base_link")));
    costmap->on_configure(rclcpp_lifecycle::State());

    costmap->on_activate(rclcpp_lifecycle::State());

    auto parameter_client = std::make_shared<rclcpp::AsyncParametersClient>(
        costmap->get_node_base_interface(), costmap->get_node_topics_interface(),
        costmap->get_node_graph_interface(),
        costmap->get_node_services_interface());

    auto results = parameter_client->set_parameters_atomically(
    {
        rclcpp::Parameter("obstacle_layer.combination_method", 5),
        rclcpp::Parameter("obstacle_layer.max_obstacle_height", 4.0),
        rclcpp::Parameter("obstacle_layer.enabled", false),
        rclcpp::Parameter("obstacle_layer.footprint_clearing_enabled", false),
        rclcpp::Parameter("obstacle_layer.inflation_radius", 0.1),
        rclcpp::Parameter("obstacle_layer.polygon", std::vector<double>{1.0, 1.0, 1.0, -1.0, -1.0, -1.0, -1.0, 1.0}),
    });

    rclcpp::spin_until_future_complete(
        costmap->get_node_base_interface(),
        results);

    EXPECT_EQ(costmap->get_parameter("obstacle_layer.combination_method").as_int(), 5);
    EXPECT_EQ(costmap->get_parameter("obstacle_layer.max_obstacle_height").as_double(), 4.0);
    EXPECT_EQ(costmap->get_parameter("obstacle_layer.enabled").as_bool(), false);
    EXPECT_EQ(costmap->get_parameter("obstacle_layer.footprint_clearing_enabled").as_bool(), false);
    EXPECT_EQ(costmap->get_parameter("obstacle_layer.inflation_radius").as_double(), 0.1);
    std::vector<double> polygon;
    costmap->get_parameter("obstacle_layer.polygon", polygon);
    ASSERT_EQ(polygon.size(), 8);
    EXPECT_EQ(polygon[0], 1.0);
    EXPECT_EQ(polygon[1], 1.0);
    EXPECT_EQ(polygon[2], 1.0);
    EXPECT_EQ(polygon[3], -1.0);
    EXPECT_EQ(polygon[4], -1.0);
    EXPECT_EQ(polygon[5], -1.0);
    EXPECT_EQ(polygon[6], -1.0);
    EXPECT_EQ(polygon[7], 1.0);

    costmap->on_deactivate(rclcpp_lifecycle::State());
    costmap->on_cleanup(rclcpp_lifecycle::State());
    costmap->on_shutdown(rclcpp_lifecycle::State());
}

/**
 * Test dynamic parameter setting of voxel layer
 */
TEST_F(TestNode, testDynParamsSetVoxel)
{
    auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");

    // Add voxel layer
    std::vector<std::string> plugins_str;
    plugins_str.push_back("voxel_layer");
    costmap->set_parameter(rclcpp::Parameter("plugins", plugins_str));
    costmap->declare_parameter(
        "voxel_layer.plugin",
        rclcpp::ParameterValue(std::string("nav2_costmap_2d::VoxelLayer")));

    costmap->set_parameter(rclcpp::Parameter("global_frame", std::string("base_link")));
    costmap->on_configure(rclcpp_lifecycle::State());

    costmap->on_activate(rclcpp_lifecycle::State());

    auto parameter_client = std::make_shared<rclcpp::AsyncParametersClient>(
        costmap->get_node_base_interface(), costmap->get_node_topics_interface(),
        costmap->get_node_graph_interface(),
        costmap->get_node_services_interface());

    auto results = parameter_client->set_parameters_atomically(
    {
        rclcpp::Parameter("voxel_layer.combination_method", 0),
        rclcpp::Parameter("voxel_layer.mark_threshold", 1),
        rclcpp::Parameter("voxel_layer.unknown_threshold", 10),
        rclcpp::Parameter("voxel_layer.z_resolution", 0.4),
        rclcpp::Parameter("voxel_layer.origin_z", 1.0),
        rclcpp::Parameter("voxel_layer.z_voxels", 14),
        rclcpp::Parameter("voxel_layer.max_obstacle_height", 4.0),
        rclcpp::Parameter("voxel_layer.footprint_clearing_enabled", false),
        rclcpp::Parameter("voxel_layer.enabled", false),
        rclcpp::Parameter("voxel_layer.publish_voxel_map", true)
    });

    rclcpp::spin_until_future_complete(
        costmap->get_node_base_interface(),
        results);

    EXPECT_EQ(costmap->get_parameter("voxel_layer.combination_method").as_int(), 0);
    EXPECT_EQ(costmap->get_parameter("voxel_layer.mark_threshold").as_int(), 1);
    EXPECT_EQ(costmap->get_parameter("voxel_layer.unknown_threshold").as_int(), 10);
    EXPECT_EQ(costmap->get_parameter("voxel_layer.z_resolution").as_double(), 0.4);
    EXPECT_EQ(costmap->get_parameter("voxel_layer.origin_z").as_double(), 1.0);
    EXPECT_EQ(costmap->get_parameter("voxel_layer.z_voxels").as_int(), 14);
    EXPECT_EQ(costmap->get_parameter("voxel_layer.max_obstacle_height").as_double(), 4.0);
    EXPECT_EQ(costmap->get_parameter("voxel_layer.footprint_clearing_enabled").as_bool(), false);
    EXPECT_EQ(costmap->get_parameter("voxel_layer.enabled").as_bool(), false);
    EXPECT_EQ(costmap->get_parameter("voxel_layer.publish_voxel_map").as_bool(), true);

    costmap->on_deactivate(rclcpp_lifecycle::State());
    costmap->on_cleanup(rclcpp_lifecycle::State());
    costmap->on_shutdown(rclcpp_lifecycle::State());
}

/**
 * Test dynamic parameter setting of static layer
 */
TEST_F(TestNode, testDynParamsSetStatic)
{
    auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");

    costmap->set_parameter(rclcpp::Parameter("global_frame", std::string("base_link")));
    costmap->on_configure(rclcpp_lifecycle::State());

    costmap->on_activate(rclcpp_lifecycle::State());

    auto parameter_client = std::make_shared<rclcpp::AsyncParametersClient>(
        costmap->get_node_base_interface(), costmap->get_node_topics_interface(),
        costmap->get_node_graph_interface(),
        costmap->get_node_services_interface());

    auto results = parameter_client->set_parameters_atomically(
    {
        rclcpp::Parameter("static_layer.transform_tolerance", 1.0),
        rclcpp::Parameter("static_layer.enabled", false),
        rclcpp::Parameter("static_layer.map_subscribe_transient_local", false),
        rclcpp::Parameter("static_layer.map_topic", "dynamic_topic"),
        rclcpp::Parameter("static_layer.subscribe_to_updates", true)
    });

    rclcpp::spin_until_future_complete(
        costmap->get_node_base_interface(),
        results);

    EXPECT_EQ(costmap->get_parameter("static_layer.transform_tolerance").as_double(), 1.0);
    EXPECT_EQ(costmap->get_parameter("static_layer.enabled").as_bool(), false);
    EXPECT_EQ(costmap->get_parameter("static_layer.map_subscribe_transient_local").as_bool(), false);
    EXPECT_EQ(costmap->get_parameter("static_layer.map_topic").as_string(), "dynamic_topic");
    EXPECT_EQ(costmap->get_parameter("static_layer.subscribe_to_updates").as_bool(), true);

    costmap->on_deactivate(rclcpp_lifecycle::State());
    costmap->on_cleanup(rclcpp_lifecycle::State());
    costmap->on_shutdown(rclcpp_lifecycle::State());
}

TEST_F(TestNode, testInflation)
{
    tf2_ros::Buffer tf(node_->get_clock());

    nav2_costmap_2d::LayeredCostmap layers("frame", false, false);
    layers.resizeMap(10, 10, 1, 0, 0);

    // Add obstacle layer
    std::shared_ptr<TestObstacleLayer> olayer = std::make_shared<TestObstacleLayer>();
    olayer->initialize(&layers, "obstacles", &tf, node_, nullptr);
    olayer->setInflationRadius(3.0);
    olayer->setPolygon({{0.0, 0.0}, {0.0, 10.0}, {10.0, 10.0}, {10.0, 0.0}});
    layers.addPlugin(olayer);

    addObservation(olayer, 1.0, 5.0, MAX_Z / 2, 0, 0, MAX_Z / 2);
    layers.updateMap(0, 0, 0);

    const std::vector<uint8_t> expected_array = {
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        254, 254, 254, 254, 0, 0, 0, 0, 0, 0,
        254, 254, 254, 254, 0, 0, 0, 0, 0, 0,
        254, 254, 254, 254, 0, 0, 0, 0, 0, 0,
        254, 254, 254, 254, 0, 0, 0, 0, 0, 0,
        254, 254, 254, 254, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    };

    // Check KeepoutFilter
    ASSERT_EQ(expected_array.size(), olayer->getSizeInCellsX() * olayer->getSizeInCellsY());
    for (size_t i = 0 ; i < expected_array.size() ; i ++) {
        EXPECT_EQ(olayer->getCost(i), expected_array.at(i)) << "at index " << i;
    }
}

TEST_F(TestNode, testOutOfPolygon)
{
    tf2_ros::Buffer tf(node_->get_clock());

    nav2_costmap_2d::LayeredCostmap layers("frame", false, false);
    layers.resizeMap(10, 10, 1, 0, 0);

    // Add obstacle layer
    std::shared_ptr<TestObstacleLayer> olayer = std::make_shared<TestObstacleLayer>();
    olayer->initialize(&layers, "obstacles", &tf, node_, nullptr);
    olayer->setInflationRadius(3.0);
    olayer->setPolygon({{5.0, 5.0}, {5.0, 10.0}, {10.0, 10.0}, {10.0, 0.0}});
    layers.addPlugin(olayer);

    addObservation(olayer, 1.0, 1.0, MAX_Z / 2, 0, 0, MAX_Z / 2);
    layers.updateMap(0, 0, 0);

    // No inflation should be applied, only the point is marked
    int lethal_count = countValues(*(layers.getCostmap()), nav2_costmap_2d::LETHAL_OBSTACLE);
    ASSERT_EQ(lethal_count, 1);
}
