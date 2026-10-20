#pragma once

#include <memory>
#include <optional>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

#include "navigator.hpp"
#include "occupancy_map.hpp"

namespace robot_sim_ros {

// A* planning + pure pursuit, in the role of Nav2.
//   subscribes  /goal_pose (RViz "2D Goal Pose"), /map; robot pose from TF map -> base_link
//   publishes   /cmd_vel (50 Hz), /plan
class NavigatorNode : public rclcpp::Node {
public:
    explicit NavigatorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    NavStatus status() const { return navigator_.status(); }

private:
    std::optional<Pose> robotPose();
    void control();
    void replan();

    Navigator navigator_;
    std::optional<OccupancyMap> map_;
    std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
    std::shared_ptr<tf2_ros::TransformListener> tfListener_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goalSub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr mapSub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdPub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pathPub_;
    rclcpp::TimerBase::SharedPtr controlTimer_;
    rclcpp::TimerBase::SharedPtr replanTimer_;
};

}  // namespace robot_sim_ros
