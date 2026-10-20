#pragma once

#include <memory>
#include <optional>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "occupancy_map.hpp"
#include "particle_filter.hpp"

namespace robot_sim_ros {

// Monte Carlo localization against a saved map, in the role of ROS's AMCL
// (and of map_server: it loads the map file and publishes /map).
//   subscribes  /scan, /odom, /initialpose (RViz "2D Pose Estimate")
//   publishes   /map (latched), /particlecloud, /amcl_pose, TF map -> odom
// Parameters: map_file (string), global (bool: start with no idea where the
// robot is), particles (int).
class LocalizerNode : public rclcpp::Node {
public:
    explicit LocalizerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    Pose estimate() const { return filter_.estimate(); }

private:
    void onScan(const sensor_msgs::msg::LaserScan& msg);
    void publishMapToOdom();

    std::optional<OccupancyMap> map_;
    ParticleFilter filter_;
    std::optional<Pose> latestOdom_;
    std::optional<Pose> odomAtLastUpdate_;
    Pose mapToOdom_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scanSub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialPoseSub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr mapPub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particlesPub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr posePub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
    rclcpp::TimerBase::SharedPtr tfTimer_;
};

}  // namespace robot_sim_ros
