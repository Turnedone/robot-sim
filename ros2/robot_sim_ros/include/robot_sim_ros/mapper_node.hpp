#pragma once

#include <memory>
#include <optional>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "occupancy_map.hpp"

namespace robot_sim_ros {

// Occupancy-grid mapping from /scan, using the robot's pose from TF
// (map -> base_link at the time of each scan). Mapping only, not SLAM: it
// trusts the pose it is given, so it is used with odometry that does not slip.
//   subscribes  /scan
//   publishes   /map (1 Hz, latched)
// Parameters: width, height (cells), resolution (m).
class MapperNode : public rclcpp::Node {
public:
    explicit MapperNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    void onScan(const sensor_msgs::msg::LaserScan& msg);

    std::optional<OccupancyMap> map_;
    std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
    std::shared_ptr<tf2_ros::TransformListener> tfListener_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scanSub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr mapPub_;
    rclcpp::TimerBase::SharedPtr publishTimer_;
};

}  // namespace robot_sim_ros
