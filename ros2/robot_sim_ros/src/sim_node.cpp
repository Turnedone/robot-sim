#include "robot_sim_ros/sim_node.hpp"

#include <chrono>
#include <string>

#include "robot_sim_ros/conversions.hpp"

namespace robot_sim_ros {

namespace {

constexpr double kPhysicsDt = 0.02;      // 50 Hz
constexpr double kScanPeriod = 0.2;      // 5 Hz
constexpr double kCommandTimeout = 0.5;  // s without /cmd_vel -> stop (a standard safety behaviour)

DiffDriveConfig robotConfig(bool slip) {
    DiffDriveConfig c;
    if (slip) {
        c.leftWheelScale = 1.005;
        c.rightWheelScale = 0.995;
        c.slipNoise = 0.05;
    }
    return c;
}

}  // namespace

SimNode::SimNode(const rclcpp::NodeOptions& options) : rclcpp::Node("sim_node", options) {
    const std::string mapFile = declare_parameter<std::string>("map_file", "");
    const bool slip = declare_parameter<bool>("slip", false);
    const auto seed = static_cast<unsigned>(declare_parameter<int>("seed", 1));

    scenario_ = loadScenario(mapFile);
    const Point start = scenario_->grid.cellCenter(scenario_->start);
    const DiffDriveConfig config = robotConfig(slip);
    robot_.emplace(config, Pose{start.x, start.y, 0.0}, seed);
    // Odometry starts at the robot's start pose, so the odom frame coincides
    // with the map frame at the start.
    odometry_.emplace(config.wheelBase, robot_->pose());
    lidar_.emplace(LidarConfig{}, seed + 100);
    lastCommandTime_ = now();

    cmdSub_ =
        create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, [this](const geometry_msgs::msg::Twist& msg) {
            command_ = msg;
            lastCommandTime_ = now();
        });
    scanPub_ = create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS());
    odomPub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    truthPub_ = create_publisher<geometry_msgs::msg::PoseStamped>("ground_truth", 10);
    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    using std::chrono::duration;
    physicsTimer_ = create_wall_timer(duration<double>(kPhysicsDt), [this] { physicsStep(); });
    scanTimer_ = create_wall_timer(duration<double>(kScanPeriod), [this] { publishScan(); });
    RCLCPP_INFO(get_logger(), "simulating %s (%dx%d cells), wheel slip %s", mapFile.c_str(), scenario_->grid.width(),
                scenario_->grid.height(), slip ? "on" : "off");
}

void SimNode::physicsStep() {
    const bool fresh = (now() - lastCommandTime_).seconds() < kCommandTimeout;
    robot_->setCommand(fresh ? Twist{command_.linear.x, command_.angular.z} : Twist{});
    robot_->step(kPhysicsDt, scenario_->grid);
    odometry_->update(robot_->leftSpeed(), robot_->rightSpeed(), kPhysicsDt);

    const auto stamp = now();
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose = toRosPose(odometry_->pose());
    const Twist v = robot_->velocity();
    odom.twist.twist.linear.x = v.linear;
    odom.twist.twist.angular.z = v.angular;
    odomPub_->publish(odom);
    tf_->sendTransform(toTransform(odometry_->pose(), "odom", "base_link", stamp));

    geometry_msgs::msg::PoseStamped truth;
    truth.header.stamp = stamp;
    truth.header.frame_id = "map";
    truth.pose = toRosPose(robot_->pose());
    truthPub_->publish(truth);
}

void SimNode::publishScan() {
    const LaserScan scan = lidar_->scan(scenario_->grid, robot_->pose());
    scanPub_->publish(toLaserScanMsg(scan, "base_link", now(), kScanPeriod));
}

}  // namespace robot_sim_ros
