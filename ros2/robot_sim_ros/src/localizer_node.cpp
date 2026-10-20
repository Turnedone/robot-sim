#include "robot_sim_ros/localizer_node.hpp"

#include <chrono>
#include <string>

#include "robot_sim_ros/conversions.hpp"

namespace robot_sim_ros {

namespace {

MclConfig mclConfig(int particles) {
    MclConfig c;
    c.particles = particles;
    return c;
}

}  // namespace

LocalizerNode::LocalizerNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("localizer_node", options),
      filter_(mclConfig(static_cast<int>(declare_parameter<int>("particles", 500)))) {
    const std::string mapFile = declare_parameter<std::string>("map_file", "");
    const bool global = declare_parameter<bool>("global", false);

    const Scenario scenario = loadScenario(mapFile);
    map_ = knownMap(scenario.grid);
    if (global) {
        filter_.initUniform(*map_);
    } else {
        // Start near the map's start cell, like giving AMCL an initial pose.
        const Point s = scenario.grid.cellCenter(scenario.start);
        filter_.initAround({s.x, s.y, 0.0}, 0.1, 0.1);
    }
    mapToOdom_ = {};  // the odom frame starts aligned with the map frame

    mapPub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("map", rclcpp::QoS(1).transient_local().reliable());
    mapPub_->publish(toOccupancyGrid(*map_, "map", now()));
    particlesPub_ = create_publisher<geometry_msgs::msg::PoseArray>("particlecloud", 10);
    posePub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("amcl_pose", 10);
    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    odomSub_ = create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, [this](const nav_msgs::msg::Odometry& msg) { latestOdom_ = fromRosPose(msg.pose.pose); });
    scanSub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::LaserScan& msg) { onScan(msg); });
    initialPoseSub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "initialpose", 10, [this](const geometry_msgs::msg::PoseWithCovarianceStamped& msg) {
            filter_.initAround(fromRosPose(msg.pose.pose), 0.2, 0.2);
            RCLCPP_INFO(get_logger(), "initial pose set from /initialpose");
        });

    // The map -> odom transform must be published continuously, not only when
    // the filter updates, or other nodes' TF lookups go stale.
    tfTimer_ = create_wall_timer(std::chrono::milliseconds(50), [this] { publishMapToOdom(); });
    RCLCPP_INFO(get_logger(), "localizing on %s (%s start)", mapFile.c_str(), global ? "global" : "known");
}

void LocalizerNode::onScan(const sensor_msgs::msg::LaserScan& msg) {
    if (!latestOdom_) return;  // no odometry yet
    const Pose odom = *latestOdom_;
    if (!odomAtLastUpdate_) odomAtLastUpdate_ = odom;

    filter_.predict(*odomAtLastUpdate_, odom);
    odomAtLastUpdate_ = odom;
    if (!filter_.correct(fromLaserScanMsg(msg), *map_)) return;  // not moved enough to update

    // map -> odom = (map -> base_link) * (odom -> base_link)^-1
    const Pose estimate = filter_.estimate();
    mapToOdom_ = compose(estimate, inverse(odom));

    geometry_msgs::msg::PoseArray cloud;
    cloud.header.stamp = msg.header.stamp;
    cloud.header.frame_id = "map";
    for (const Particle& p : filter_.particles()) cloud.poses.push_back(toRosPose(p.pose));
    particlesPub_->publish(cloud);

    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header = cloud.header;
    pose.pose.pose = toRosPose(estimate);
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
