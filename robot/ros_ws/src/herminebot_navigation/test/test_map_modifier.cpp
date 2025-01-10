#include <gtest/gtest.h>
#include "herminebot_navigation/map_modifier.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/duration.hpp"
#include "nav2_util/occ_grid_values.hpp"

using namespace std::chrono_literals;

class MapModifierWrapper : public hrc_map::MapModifier
{
public:
    // Each polygon MUST separated by at leat 3 OCC_GRID_FREE cells to be considered as another polygon
    uint8_t getPolygonsCount() const
    {
        return added_polygons_.size();
    }

    // Make method public for testing
    void mapToWorld(const unsigned int mx, const unsigned int my, double& wx, double& wy)
    {
        MapModifier::mapToWorld(mx, my, wx, wy);
    }
};

class Tester : public ::testing::Test
{
public:
    Tester();

    bool waitMask(const std::chrono::nanoseconds& timeout);
    void manageObjectsCb(const std::shared_ptr<hrc_interfaces::srv::ManageObjectsMap::Request> req);
    bool checkMaskEq(const std::vector<int8_t>& data);

protected:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    std::shared_ptr<MapModifierWrapper> mp_;
    nav_msgs::msg::OccupancyGrid map_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
};

Tester::Tester()
{
    mp_ = std::make_shared<MapModifierWrapper>();
    const rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    map_sub_ = mp_->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/mask_filter", qos,
        std::bind(&Tester::mapCallback, this, std::placeholders::_1));
}

void Tester::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    map_ = *msg;
}

bool Tester::waitMask(const std::chrono::nanoseconds& timeout)
{
    rclcpp::Time start_time = mp_->now();
    while (rclcpp::ok() && mp_->now() - start_time <= rclcpp::Duration(timeout)) {
        if (map_.data.size()) {
            return true;
        }
        rclcpp::spin_some(mp_->get_node_base_interface());
        std::this_thread::sleep_for(10ms);
    }
    return false;
}

void Tester::manageObjectsCb(const std::shared_ptr<hrc_interfaces::srv::ManageObjectsMap::Request> req)
{
    map_.data.clear();
    const std::shared_ptr<hrc_interfaces::srv::ManageObjectsMap::Response> res;
    mp_->manageObjectsCb(req, res);
}

bool Tester::checkMaskEq(const std::vector<int8_t>& data)
{
    if (map_.data.size() != data.size()) {
        RCLCPP_ERROR(rclcpp::get_logger("MapModifierTester"), "Size unmatch");
        return false;
    }
    bool res = true;
    for (size_t i = 0; i < data.size(); i++) {
        if (map_.data[i] != data[i]) {
            res = false;
            RCLCPP_ERROR(
                rclcpp::get_logger("MapModifierTester"),
                "Wrong value '%d' at index '%ld'. Expect '%d'", map_.data[i], i, data[i]);
        }
    }
    return res;
}

TEST_F(Tester, testMapModifier)
{
    constexpr int8_t o = nav2_util::OCC_GRID_OCCUPIED;
    constexpr int8_t f = nav2_util::OCC_GRID_FREE;

    std::vector<int8_t> data = {
        f, f, f, f, f, f, f, f, f, f,
        f, o, o, f, f, f, f, f, f, f,
        f, o, o, f, f, f, f, f, o, o,
        f, f, f, f, f, f, f, f, o, o,
        f, f, f, f, f, f, f, f, f, f,
        f, f, f, f, f, f, f, f, f, f,
        f, o, o, o, o, f, f, f, f, f,
        f, o, o, o, o, f, f, f, f, f,
        f, f, o, o, f, f, f, f, f, f,
        f, f, f, f, f, f, f, f, f, f
    };

    std::unique_ptr<nav_msgs::msg::OccupancyGrid> map = std::make_unique<nav_msgs::msg::OccupancyGrid>();
    map->info.width = 10;
    map->info.height = 10;
    map->info.resolution = 0.1;
    map->info.origin.position.x = 0.4;
    map->info.origin.position.y = -0.3;
    map->header.frame_id = "map";
    map->header.stamp = mp_->now();
    map->data = data;
    mp_->initialMaskCb(std::move(map));

    // Verify initial mask
    waitMask(100ms);
    ASSERT_TRUE(checkMaskEq(data));
    ASSERT_EQ(mp_->getPolygonsCount(), 3);

    auto req = std::make_shared<hrc_interfaces::srv::ManageObjectsMap::Request>();
    geometry_msgs::msg::Polygon poly;
    geometry_msgs::msg::Point32 p;
    double wx, wy;

    // Add
    mp_->mapToWorld(8, 7, wx, wy);
    p.x = (float) wx;
    p.y = (float) wy;
    poly.points.push_back(p);
    mp_->mapToWorld(8, 8, wx, wy);
    p.x = (float) wx;
    p.y = (float) wy;
    poly.points.push_back(p);
    mp_->mapToWorld(6, 8, wx, wy);
    p.x = (float) wx;
    p.y = (float) wy;
    poly.points.push_back(p);
    mp_->mapToWorld(6, 7, wx, wy);
    p.x = (float) wx;
    p.y = (float) wy;
    poly.points.push_back(p);
    req->new_objects.push_back(poly);

    // Remove
    mp_->mapToWorld(3, 7, wx, wy);
    p.x = (float) wx;
    p.y = (float) wy;
    req->points_objects_to_remove.push_back(p);
    mp_->mapToWorld(1, 1, wx, wy);
    p.x = (float) wx;
    p.y = (float) wy;
    req->points_objects_to_remove.push_back(p);

    data = {
        f, f, f, f, f, f, f, f, f, f,
        f, f, f, f, f, f, f, f, f, f,
        f, f, f, f, f, f, f, f, o, o,
        f, f, f, f, f, f, f, f, o, o,
        f, f, f, f, f, f, f, f, f, f,
        f, f, f, f, f, f, f, f, f, f,
        f, f, f, f, f, f, f, f, f, f,
        f, f, f, f, f, f, o, o, o, f,
        f, f, f, f, f, f, o, o, o, f,
        f, f, f, f, f, f, f, f, f, f
    };

    manageObjectsCb(req);

    // Verify after update
    waitMask(100ms);
    ASSERT_TRUE(checkMaskEq(data));
    ASSERT_EQ(mp_->getPolygonsCount(), 2);

    // Clear all
    req->new_objects.clear();
    req->points_objects_to_remove.clear();
    mp_->mapToWorld(9, 3, wx, wy);
    p.x = (float) wx;
    p.y = (float) wy;
    req->points_objects_to_remove.push_back(p);
    mp_->mapToWorld(8, 7, wx, wy);
    p.x = (float) wx;
    p.y = (float) wy;
    req->points_objects_to_remove.push_back(p);

    data = {
        f, f, f, f, f, f, f, f, f, f,
        f, f, f, f, f, f, f, f, f, f,
        f, f, f, f, f, f, f, f, f, f,
        f, f, f, f, f, f, f, f, f, f,
        f, f, f, f, f, f, f, f, f, f,
        f, f, f, f, f, f, f, f, f, f,
        f, f, f, f, f, f, f, f, f, f,
        f, f, f, f, f, f, f, f, f, f,
        f, f, f, f, f, f, f, f, f, f,
        f, f, f, f, f, f, f, f, f, f
    };

    manageObjectsCb(req);

    // Verify after update
    waitMask(100ms);
    ASSERT_EQ(mp_->getPolygonsCount(), 0);
    ASSERT_TRUE(checkMaskEq(data));

}

int main(int argc, char** argv)
{
    // Initialize the system
    testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);

    // Actual testing
    bool test_result = RUN_ALL_TESTS();

    // Shutdown
    rclcpp::shutdown();

    return test_result;
}
