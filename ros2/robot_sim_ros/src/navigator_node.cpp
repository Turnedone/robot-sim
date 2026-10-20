#include "robot_sim_ros/navigator_node.hpp"

#include <chrono>

#include <tf2/exceptions.h>

#include "robot_sim_ros/conversions.hpp"

namespace robot_sim_ros {

NavigatorNode::NavigatorNode(const rclcpp::NodeOptions& options) : rclcpp::Node("navigator_node", options) {
    tfBuffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
    cmdPub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    pathPub_ = create_publisher<nav_msgs::msg::Path>("plan", 10);

    mapSub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map", rclcpp::QoS(1).transient_local().reliable(),
        [this](const nav_msgs::msg::OccupancyGrid& msg) { map_ = fromOccupancyGrid(msg); });
    goalSub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "goal_pose", 10, [this](const geometry_msgs::msg::PoseStamped& msg) {
            if (!map_) {
                RCLCPP_WARN(get_logger(), "goal ignored: no map yet");
                return;
            }
            const Pos cell = map_->worldToCell({msg.pose.position.x, msg.pose.position.y});
            navigator_.setGoal(cell);
            RCLCPP_INFO(get_logger(), "new goal: cell (%d, %d)", cell.x, cell.y);
            replan();
        });

    controlTimer_ = create_wall_timer(std::chrono::milliseconds(20), [this] { control(); });
    replanTimer_ = create_wall_timer(std::chrono::milliseconds(200), [this] { replan(); });
}

std::optional<Pose> NavigatorNode::robotPose() {
    try {
        return fromTransform(tfBuffer_->lookupTransform("map", "base_link", tf2::TimePointZero));
    } catch (const tf2::TransformException& e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "no robot pose: %s", e.what());
        return std::nullopt;
    }
}

void NavigatorNode::control() {
    const NavStatus before = navigator_.status();
    geometry_msgs::msg::Twist cmd;
    if (map_ && before == NavStatus::Driving) {
        if (const std::optional<Pose> pose = robotPose()) {
            const Twist t = navigator_.command(*pose, *map_);
            cmd.linear.x = t.linear;
            cmd.angular.z = t.angular;
        }
    }
    cmdPub_->publish(cmd);
    if (before != NavStatus::Arrived && navigator_.status() == NavStatus::Arrived)
        RCLCPP_INFO(get_logger(), "goal reached");
}

void NavigatorNode::replan() {
    if (!map_) return;
    const std::optional<Pose> pose = robotPose();
    if (!pose) return;
    navigator_.replan(*map_, *pose);

    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = "map";
    for (const Point& p : navigator_.path()) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = path.header;
        ps.pose.position.x = p.x;
        ps.pose.position.y = p.y;
        ps.pose.orientation.w = 1.0;
        path.poses.push_back(ps);
    }
    pathPub_->publish(path);
}

}  // namespace robot_sim_ros
