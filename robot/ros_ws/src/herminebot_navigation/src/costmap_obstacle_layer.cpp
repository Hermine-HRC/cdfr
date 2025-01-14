#include "herminebot_navigation/costmap_obstacle_layer.hpp"

using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::FREE_SPACE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;

using nav2_costmap_2d::ObservationBuffer;
using nav2_costmap_2d::Observation;
using rcl_interfaces::msg::ParameterType;

namespace hrc_costmap_2d
{

void ObstacleLayer::onInitialize()
{
    bool track_unknown_space;
    double transform_tolerance;

    // The topics that we'll subscribe to from the parameter server
    std::string topics_string;

    declareParameter("enabled", rclcpp::ParameterValue(true));
    declareParameter("footprint_clearing_enabled", rclcpp::ParameterValue(true));
    declareParameter("min_obstacle_height", rclcpp::ParameterValue(0.0));
    declareParameter("max_obstacle_height", rclcpp::ParameterValue(2.0));
    declareParameter("combination_method", rclcpp::ParameterValue(1));
    declareParameter("observation_sources", rclcpp::ParameterValue(std::string("")));
    declareParameter("inflation_radius", rclcpp::ParameterValue(0.0));
    declareParameter("polygon", rclcpp::PARAMETER_DOUBLE_ARRAY);

    auto node = node_.lock();
    if (!node) {
        throw std::runtime_error{"Failed to lock node"};
    }

    node->get_parameter(name_ + "." + "enabled", enabled_);
    node->get_parameter(name_ + "." + "footprint_clearing_enabled", footprint_clearing_enabled_);
    node->get_parameter(name_ + "." + "min_obstacle_height", min_obstacle_height_);
    node->get_parameter(name_ + "." + "max_obstacle_height", max_obstacle_height_);
    node->get_parameter(name_ + "." + "combination_method", combination_method_);
    node->get_parameter("track_unknown_space", track_unknown_space);
    node->get_parameter("transform_tolerance", transform_tolerance);
    node->get_parameter(name_ + "." + "observation_sources", topics_string);
    node->get_parameter(name_ + "." + "inflation_radius", inflation_radius_);
    std::vector<double> poly;
    node->get_parameter(name_ + "." + "polygon", poly);

    for (unsigned int i = 0 ; i < poly.size() ; i += 2) {
        poly_.emplace_back(poly.at(i), poly.at(i + 1));
    }

    dyn_params_handler_ = node->add_on_set_parameters_callback(
        std::bind(
            &ObstacleLayer::dynamicParametersCallback,
            this,
            std::placeholders::_1));

    RCLCPP_INFO(
        logger_,
        "Subscribed to Topics: %s", topics_string.c_str());

    rolling_window_ = layered_costmap_->isRolling();

    if (track_unknown_space) {
        default_value_ = NO_INFORMATION;
    }
    else {
        default_value_ = FREE_SPACE;
    }

    ObstacleLayer::matchSize();
    current_ = true;
    was_reset_ = false;

    global_frame_ = layered_costmap_->getGlobalFrameID();

    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = callback_group_;

    // now we need to split the topics based on whitespace which we can use a stringstream for
    std::stringstream ss(topics_string);

    std::string source;
    while (ss >> source) {
        // get the parameters for the specific topic
        double observation_keep_time, expected_update_rate, min_obstacle_height, max_obstacle_height;
        std::string topic, sensor_frame, data_type;
        bool inf_is_valid, clearing, marking;

        declareParameter(source + "." + "topic", rclcpp::ParameterValue(source));
        declareParameter(source + "." + "sensor_frame", rclcpp::ParameterValue(std::string("")));
        declareParameter(source + "." + "observation_persistence", rclcpp::ParameterValue(0.0));
        declareParameter(source + "." + "expected_update_rate", rclcpp::ParameterValue(0.0));
        declareParameter(source + "." + "data_type", rclcpp::ParameterValue(std::string("LaserScan")));
        declareParameter(source + "." + "min_obstacle_height", rclcpp::ParameterValue(0.0));
        declareParameter(source + "." + "max_obstacle_height", rclcpp::ParameterValue(0.0));
        declareParameter(source + "." + "inf_is_valid", rclcpp::ParameterValue(false));
        declareParameter(source + "." + "marking", rclcpp::ParameterValue(true));
        declareParameter(source + "." + "clearing", rclcpp::ParameterValue(false));
        declareParameter(source + "." + "obstacle_max_range", rclcpp::ParameterValue(2.5));
        declareParameter(source + "." + "obstacle_min_range", rclcpp::ParameterValue(0.0));
        declareParameter(source + "." + "raytrace_max_range", rclcpp::ParameterValue(3.0));
        declareParameter(source + "." + "raytrace_min_range", rclcpp::ParameterValue(0.0));

        node->get_parameter(name_ + "." + source + "." + "topic", topic);
        node->get_parameter(name_ + "." + source + "." + "sensor_frame", sensor_frame);
        node->get_parameter(
            name_ + "." + source + "." + "observation_persistence",
            observation_keep_time);
        node->get_parameter(
            name_ + "." + source + "." + "expected_update_rate",
            expected_update_rate);
        node->get_parameter(name_ + "." + source + "." + "data_type", data_type);
        node->get_parameter(name_ + "." + source + "." + "min_obstacle_height", min_obstacle_height);
        node->get_parameter(name_ + "." + source + "." + "max_obstacle_height", max_obstacle_height);
        node->get_parameter(name_ + "." + source + "." + "inf_is_valid", inf_is_valid);
        node->get_parameter(name_ + "." + source + "." + "marking", marking);
        node->get_parameter(name_ + "." + source + "." + "clearing", clearing);

        if (!(data_type == "PointCloud2" || data_type == "LaserScan")) {
            RCLCPP_FATAL(
                logger_,
                "Only topics that use point cloud2s or laser scans are currently supported");
            throw std::runtime_error(
                      "Only topics that use point cloud2s or laser scans are currently supported");
        }

        // get the obstacle range for the sensor
        double obstacle_max_range, obstacle_min_range;
        node->get_parameter(name_ + "." + source + "." + "obstacle_max_range", obstacle_max_range);
        node->get_parameter(name_ + "." + source + "." + "obstacle_min_range", obstacle_min_range);

        // get the raytrace ranges for the sensor
        double raytrace_max_range, raytrace_min_range;
        node->get_parameter(name_ + "." + source + "." + "raytrace_min_range", raytrace_min_range);
        node->get_parameter(name_ + "." + source + "." + "raytrace_max_range", raytrace_max_range);


        RCLCPP_DEBUG(
            logger_,
            "Creating an observation buffer for source %s, topic %s, frame %s",
            source.c_str(), topic.c_str(),
            sensor_frame.c_str());

        // create an observation buffer
        observation_buffers_.push_back(
            std::shared_ptr<ObservationBuffer>(
                new ObservationBuffer(
                    node, topic, observation_keep_time, expected_update_rate,
                    min_obstacle_height,
                    max_obstacle_height, obstacle_max_range, obstacle_min_range, raytrace_max_range,
                    raytrace_min_range, *tf_,
                    global_frame_,
                    sensor_frame, tf2::durationFromSec(transform_tolerance))));

        // check if we'll add this buffer to our marking observation buffers
        if (marking) {
            marking_buffers_.push_back(observation_buffers_.back());
        }

        // check if we'll also add this buffer to our clearing observation buffers
        if (clearing) {
            clearing_buffers_.push_back(observation_buffers_.back());
        }

        RCLCPP_DEBUG(
            logger_,
            "Created an observation buffer for source %s, topic %s, global frame: %s, "
            "expected update rate: %.2f, observation persistence: %.2f",
            source.c_str(), topic.c_str(),
            global_frame_.c_str(), expected_update_rate, observation_keep_time);

        rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_sensor_data;
        custom_qos_profile.depth = 50;

        // create a callback for the topic
        if (data_type == "LaserScan") {
            auto sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::LaserScan,
                    rclcpp_lifecycle::LifecycleNode>>(node, topic, custom_qos_profile, sub_opt);
            sub->unsubscribe();

            auto filter = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(
                *sub, *tf_, global_frame_, 50,
                node->get_node_logging_interface(),
                node->get_node_clock_interface(),
                tf2::durationFromSec(transform_tolerance));

            if (inf_is_valid) {
                filter->registerCallback(
                    std::bind(
                        &ObstacleLayer::laserScanValidInfCallback, this, std::placeholders::_1,
                        observation_buffers_.back()));

            }
            else {
                filter->registerCallback(
                    std::bind(
                        &ObstacleLayer::laserScanCallback, this, std::placeholders::_1,
                        observation_buffers_.back()));
            }

            observation_subscribers_.push_back(sub);

            observation_notifiers_.push_back(filter);
            observation_notifiers_.back()->setTolerance(rclcpp::Duration::from_seconds(0.05));

        }
        else {
            auto sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2,
                    rclcpp_lifecycle::LifecycleNode>>(node, topic, custom_qos_profile, sub_opt);
            sub->unsubscribe();

            if (inf_is_valid) {
                RCLCPP_WARN(
                    logger_,
                    "obstacle_layer: inf_is_valid option is not applicable to PointCloud observations.");
            }

            auto filter = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>>(
                *sub, *tf_, global_frame_, 50,
                node->get_node_logging_interface(),
                node->get_node_clock_interface(),
                tf2::durationFromSec(transform_tolerance));

            filter->registerCallback(
                std::bind(
                    &ObstacleLayer::pointCloud2Callback, this, std::placeholders::_1,
                    observation_buffers_.back()));

            observation_subscribers_.push_back(sub);
            observation_notifiers_.push_back(filter);
        }

        if (sensor_frame != "") {
            std::vector<std::string> target_frames;
            target_frames.push_back(global_frame_);
            target_frames.push_back(sensor_frame);
            observation_notifiers_.back()->setTargetFrames(target_frames);
        }
    }
}

void ObstacleLayer::updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y, double * max_x, double * max_y
)
{
    std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
    reset();
    if (rolling_window_) {
        updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
    }
    if (!enabled_) {
        return;
    }
    useExtraBounds(min_x, min_y, max_x, max_y);

    bool current = true;
    std::vector<Observation> observations, clearing_observations;

    // get the marking observations
    current = current && getMarkingObservations(observations);

    // get the clearing observations
    current = current && getClearingObservations(clearing_observations);

    // update the global current status
    current_ = current;

    // raytrace freespace
    for (unsigned int i = 0; i < clearing_observations.size(); ++i) {
        raytraceFreespace(clearing_observations[i], min_x, min_y, max_x, max_y);
    }

    // place the new obstacles into a priority queue... each with a priority of zero to begin with
    for (std::vector<Observation>::const_iterator it = observations.begin();
        it != observations.end(); ++it)
    {
        const Observation & obs = *it;

        const sensor_msgs::msg::PointCloud2 & cloud = *(obs.cloud_);

        double sq_obstacle_max_range = obs.obstacle_max_range_ * obs.obstacle_max_range_;
        double sq_obstacle_min_range = obs.obstacle_min_range_ * obs.obstacle_min_range_;

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            double px = *iter_x, py = *iter_y, pz = *iter_z;

            // if the obstacle is too low, we won't add it
            if (pz < min_obstacle_height_) {
                RCLCPP_DEBUG(logger_, "The point is too low");
                continue;
            }

            // if the obstacle is too high or too far away from the robot we won't add it
            if (pz > max_obstacle_height_) {
                RCLCPP_DEBUG(logger_, "The point is too high");
                continue;
            }

            // compute the squared distance from the hitpoint to the pointcloud's origin
            double sq_dist =
                (px -
                obs.origin_.x) * (px - obs.origin_.x) + (py - obs.origin_.y) * (py - obs.origin_.y) +
                (pz - obs.origin_.z) * (pz - obs.origin_.z);

            // if the point is far enough away... we won't consider it
            if (sq_dist >= sq_obstacle_max_range) {
                RCLCPP_DEBUG(logger_, "The point is too far away");
                continue;
            }

            // if the point is too close, do not conisder it
            if (sq_dist < sq_obstacle_min_range) {
                RCLCPP_DEBUG(logger_, "The point is too close");
                continue;
            }

            // now we need to compute the map coordinates for the observation
            unsigned int mx, my;
            if (!worldToMap(px, py, mx, my)) {
                RCLCPP_DEBUG(logger_, "Computing map coords failed");
                continue;
            }

            unsigned int index = getIndex(mx, my);
            costmap_[index] = LETHAL_OBSTACLE;
            applyInflation(mx, my);
            touch(px, py, min_x, min_y, max_x, max_y);
        }
    }

    updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}

rcl_interfaces::msg::SetParametersResult
ObstacleLayer::dynamicParametersCallback(
    std::vector<rclcpp::Parameter> parameters)
{
    for (auto parameter : parameters) {
        const auto & param_type = parameter.get_type();
        const auto & param_name = parameter.get_name();

        if (param_type == ParameterType::PARAMETER_DOUBLE) {
            if (param_name == name_ + "." + "inflation_radius") {
                inflation_radius_ = parameter.as_double();
            }
        }
        else if (param_type == ParameterType::PARAMETER_DOUBLE_ARRAY) {
            if (param_name == name_ + "." + "polygon") {
                std::vector<double> poly = parameter.as_double_array();
                for (unsigned int i = 0 ; i < poly.size() ; i += 2) {
                    poly_.emplace_back(poly.at(i), poly.at(i + 1));
                }
            }
        }
    }
    return nav2_costmap_2d::ObstacleLayer::dynamicParametersCallback(parameters);
}

void ObstacleLayer::applyInflation(const unsigned int x_center, const unsigned int y_center)
{
    if (!isPointInside(poly_, x_center, y_center)) {return;}

    const unsigned int pix_inflation_radius = cellDistance(inflation_radius_);
    const unsigned int sq_pix_inflation_radius = pix_inflation_radius * pix_inflation_radius;
    unsigned int x, y, dx, dy;

    for (x = std::max((int) (x_center - pix_inflation_radius), 0) ;
        x < std::min(x_center + pix_inflation_radius, getSizeInCellsX()) ; x ++)
    {
        for (y = std::max((int) (y_center - pix_inflation_radius), 0) ;
            y < std::min(y_center + pix_inflation_radius, getSizeInCellsY()) ; y ++)
        {
            dx = x - x_center;
            dy = y - y_center;
            if (dx * dx + dy * dy < sq_pix_inflation_radius) {
                setCost(x, y, LETHAL_OBSTACLE);
            }
        }
    }
}

bool ObstacleLayer::isPointInside(std::vector<std::pair<double, double>>& poly, const int x, const int y) const
{
    const int poly_size = poly.size();
    if (poly_size == 0) { // By default, all points should be counted as inside
        return true;
    }
    // Adaptation of Shimrat, Moshe. "Algorithm 112: position of point relative to polygon."
    // Communications of the ACM 5.8 (1962): 434.
    // Implementation of ray crossings algorithm for point in polygon task solving.
    // Y coordinate is fixed. Moving the ray on X+ axis starting from given point.
    // Odd number of intersections with polygon boundaries means the point is inside polygon.
    double wx, wy;
    mapToWorld(x, y, wx, wy);

    int i, j;  // Polygon vertex iterators
    bool res = false;  // Final result, initialized with already inverted value

    // Starting from the edge where the last point of polygon is connected to the first
    i = poly_size - 1;
    for (j = 0; j < poly_size; j++) {
        // Checking the edge only if given point is between edge boundaries by Y coordinates.
        // One of the condition should contain equality in order to exclude the edges
        // parallel to X+ ray.
        if ((wy <= poly[i].second) == (wy > poly[j].second)) {
            // Calculating the intersection coordinate of X+ ray
            const double x_inter = poly[i].first +
                (wy - poly[i].second) * (poly[j].first - poly[i].first) /
                (poly[j].second - poly[i].second);
            // If intersection with checked edge is greater than point.x coordinate, inverting the result
            if (x_inter > wx) {
                res = !res;
            }
        }
        i = j;
    }
    return res;
}

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(hrc_costmap_2d::ObstacleLayer, nav2_costmap_2d::Layer)
