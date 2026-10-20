#pragma once

#include <memory>
#include <optional>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "grid.hpp"
#include "lidar.hpp"
#include "odometry.hpp"
#include "robot.hpp"

namespace robot_sim_ros {

// Stands in for the robot hardware and its drivers: takes velocity commands
// and publishes what a real robot's drivers would.
//   subscribes  /cmd_vel        geometry_msgs/Twist
//   publishes   /scan           sensor_msgs/LaserScan (5 Hz, frame base_link)
//               /odom           nav_msgs/Odometry (50 Hz) + TF odom -> base_link
//               /ground_truth   geometry_msgs/PoseStamped: where the robot REALLY is
//                               (for evaluation only; a real robot has no such topic)
// Parameters: map_file (string), slip (bool: slipping wheels, drifting odometry), seed (int).
class SimNode : public rclcpp::Node {
public:
    explicit SimNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    Pose truePose() const { return robot_->pose(); }
    Pos goalCell() const { return scenario_->goal; }
    const Scenario& scenario() const { return *scenario_; }

private:
    void physicsStep();
    void publishScan();

    std::optional<Scenario> scenario_;
    std::optional<DiffDriveRobot> robot_;
    std::optional<Odometry> odometry_;
    std::optional<Lidar> lidar_;
    geometry_msgs::msg::Twist command_;
    rclcpp::Time lastCommandTime_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdSub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scanPub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr truthPub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
    rclcpp::TimerBase::SharedPtr physicsTimer_;
    rclcpp::TimerBase::SharedPtr scanTimer_;
};

}  // namespace robot_sim_ros
