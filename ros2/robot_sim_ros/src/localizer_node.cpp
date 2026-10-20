#include "robot_sim_ros/localizer_node.hpp"

#include <chrono>
#include <string>

#include "robot_sim_ros/conversions.hpp"

namespace robot_sim_ros {

namespace {

MclConfig mclConfig(int particles, const Pose& sensorOffset) {
    MclConfig c;
    c.particles = particles;
    c.sensorOffset = sensorOffset;
    return c;
}

}  // namespace

LocalizerNode::LocalizerNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("localizer_node", options),
      filter_(mclConfig(static_cast<int>(declare_parameter<int>("particles", 500)),
                        {declare_parameter<double>("sensor_x", 0.0), declare_parameter<double>("sensor_y", 0.0),
                         declare_parameter<double>("sensor_yaw", 0.0)})) {
    const std::string mapFile = declare_parameter<std::string>("map_file", "");
    const bool global = declare_parameter<bool>("global", false);
    const Pose initial{declare_parameter<double>("initial_x", 0.0), declare_parameter<double>("initial_y", 0.0),
                       declare_parameter<double>("initial_yaw", 0.0)};
    mapToOdom_ = {};  // the odom frame starts aligned with the map frame

    particlesPub_ = create_publisher<geometry_msgs::msg::PoseArray>("particlecloud", 10);
    posePub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("amcl_pose", 10);
    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    if (!mapFile.empty()) {
        // Our own map file: load it and publish it, like map_server would.
        const Scenario scenario = loadScenario(mapFile);
        const Point s = scenario.grid.cellCenter(scenario.start);
        useMap(knownMap(scenario.grid), {0.0, 0.0}, {s.x, s.y, 0.0}, global);
        mapPub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("map", rclcpp::QoS(1).transient_local().reliable());
        mapPub_->publish(toOccupancyGrid(*map_, "map", now()));
        RCLCPP_INFO(get_logger(), "localizing on %s (%s start)", mapFile.c_str(), global ? "global" : "known");
    } else {
        // A map from a map_server (e.g. Nav2's), with its own origin.
        mapSub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            "map", rclcpp::QoS(1).transient_local().reliable(),
            [this, initial, global](const nav_msgs::msg::OccupancyGrid& msg) {
                if (map_) return;  // a saved map does not change
                useMap(fromOccupancyGrid(msg), {msg.info.origin.position.x, msg.info.origin.position.y}, initial,
                       global);
                RCLCPP_INFO(get_logger(), "map received: %ux%u cells of %.2f m, origin (%.2f, %.2f)", msg.info.width,
                            msg.info.height, msg.info.resolution, origin_.x, origin_.y);
            });
    }

    odomSub_ = create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, [this](const nav_msgs::msg::Odometry& msg) { latestOdom_ = fromRosPose(msg.pose.pose); });
    scanSub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::LaserScan& msg) { onScan(msg); });
    initialPoseSub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "initialpose", 10, [this](const geometry_msgs::msg::PoseWithCovarianceStamped& msg) {
            filter_.initAround(toCore(fromRosPose(msg.pose.pose)), 0.2, 0.2);
            RCLCPP_INFO(get_logger(), "initial pose set from /initialpose");
        });

    // The map -> odom transform must be published continuously, not only when
    // the filter updates, or other nodes' TF lookups go stale.
    tfTimer_ = create_wall_timer(std::chrono::milliseconds(50), [this] { publishMapToOdom(); });
}

void LocalizerNode::useMap(const OccupancyMap& map, Point origin, const Pose& startInMapFrame, bool global) {
    map_ = map;
    origin_ = origin;
    if (global)
        filter_.initUniform(*map_);
    else
        filter_.initAround(toCore(startInMapFrame), 0.1, 0.1);
}

Pose LocalizerNode::estimate() const {
    return toRos(filter_.estimate());
}

void LocalizerNode::onScan(const sensor_msgs::msg::LaserScan& msg) {
    if (!latestOdom_ || !map_) return;  // no odometry or map yet
    const Pose odom = *latestOdom_;
    if (!odomAtLastUpdate_) odomAtLastUpdate_ = odom;

    filter_.predict(*odomAtLastUpdate_, odom);
    odomAtLastUpdate_ = odom;
    if (!filter_.correct(fromLaserScanMsg(msg), *map_)) return;  // not moved enough to update

    // map -> odom = (map -> base_link) * (odom -> base_link)^-1
    const Pose est = estimate();
    mapToOdom_ = compose(est, inverse(odom));

    geometry_msgs::msg::PoseArray cloud;
    cloud.header.stamp = msg.header.stamp;
    cloud.header.frame_id = "map";
    for (const Particle& p : filter_.particles()) cloud.poses.push_back(toRosPose(toRos(p.pose)));
    particlesPub_->publish(cloud);

    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header = cloud.header;
    pose.pose.pose = toRosPose(est);
    const double variance = filter_.spread() * filter_.spread() / 2;
    pose.pose.covariance[0] = variance;  // x
    pose.pose.covariance[7] = variance;  // y
    pose.pose.covariance[35] = 0.05;     // yaw (rough)
    posePub_->publish(pose);
}

void LocalizerNode::publishMapToOdom() {
    tf_->sendTransform(toTransform(mapToOdom_, "map", "odom", now()));
}

}  // namespace robot_sim_ros
