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

// Monte Carlo localization against a saved map, in the role of ROS's AMCL.
//   subscribes  /scan, /odom, /initialpose (RViz "2D Pose Estimate"),
//               /map (when map_file is empty: from a map_server)
//   publishes   /particlecloud, /amcl_pose, TF map -> odom,
//               and /map (latched) when it loaded map_file itself
// Parameters:
//   map_file           a robot-sim map file; empty = use the /map topic
//   global             start with no idea where the robot is
//   initial_x/_y/_yaw  start pose in the map frame (map_file: its start cell)
//   particles          particle count when confident
//   sensor_x/_y/_yaw   where the lidar sits on the robot (base_link frame)
//
// ROS maps can have their origin anywhere (Nav2's TurtleBot3 map: -10, -10);
// the core's map starts at 0, 0. Poses are shifted by the map origin on the
// way in and out.
class LocalizerNode : public rclcpp::Node {
public:
    explicit LocalizerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    // The estimate in the ROS map frame.
    Pose estimate() const;

private:
    void useMap(const OccupancyMap& map, Point origin, const Pose& startInMapFrame, bool global);
    void onScan(const sensor_msgs::msg::LaserScan& msg);
    void publishMapToOdom();
    Pose toCore(const Pose& p) const { return {p.x - origin_.x, p.y - origin_.y, p.theta}; }
    Pose toRos(const Pose& p) const { return {p.x + origin_.x, p.y + origin_.y, p.theta}; }

    std::optional<OccupancyMap> map_;
    Point origin_{};
    ParticleFilter filter_;
    std::optional<Pose> latestOdom_;
    std::optional<Pose> odomAtLastUpdate_;
    Pose mapToOdom_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scanSub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialPoseSub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr mapSub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr mapPub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particlesPub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr posePub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
    rclcpp::TimerBase::SharedPtr tfTimer_;
};

}  // namespace robot_sim_ros
