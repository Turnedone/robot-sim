#include "robot_sim_ros/mapper_node.hpp"

#include <chrono>

#include <tf2/exceptions.h>

#include "robot_sim_ros/conversions.hpp"

namespace robot_sim_ros {

MapperNode::MapperNode(const rclcpp::NodeOptions& options) : rclcpp::Node("mapper_node", options) {
    const auto width = static_cast<int>(declare_parameter<int>("width", 30));
    const auto height = static_cast<int>(declare_parameter<int>("height", 13));
    const double resolution = declare_parameter<double>("resolution", 0.5);
    map_.emplace(width, height, resolution);

    tfBuffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
    mapPub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("map", rclcpp::QoS(1).transient_local().reliable());
    scanSub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::LaserScan& msg) { onScan(msg); });
    publishTimer_ =
        create_wall_timer(std::chrono::seconds(1), [this] { mapPub_->publish(toOccupancyGrid(*map_, "map", now())); });
}

void MapperNode::onScan(const sensor_msgs::msg::LaserScan& msg) {
    Pose pose;
    try {
        // Where was the robot when this scan was taken?
        pose = fromTransform(tfBuffer_->lookupTransform("map", msg.header.frame_id, rclcpp::Time(msg.header.stamp),
                                                        rclcpp::Duration::from_seconds(0.1)));
    } catch (const tf2::TransformException& e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "no pose for scan: %s", e.what());
        return;
    }
    map_->integrate(fromLaserScanMsg(msg), pose);
}

}  // namespace robot_sim_ros
