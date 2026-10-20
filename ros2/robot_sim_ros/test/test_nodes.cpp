// The ROS graph closing the loop: simulator, localizer and navigator run in one
// process but talk only through ROS topics and TF, exactly as separate
// processes would. A goal is published like RViz's "2D Goal Pose" does, and the
// simulated robot must actually drive there.

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "robot_sim_ros/localizer_node.hpp"
#include "robot_sim_ros/navigator_node.hpp"
#include "robot_sim_ros/sim_node.hpp"

using namespace std::chrono_literals;

class RosGraph : public ::testing::Test {
protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }
};

TEST_F(RosGraph, RobotDrivesToAGoalUsingOnlyTopicsAndTf) {
    const std::string mapFile = std::string(ROBOT_SIM_MAPS_DIR) + "/warehouse.txt";
    rclcpp::NodeOptions simOptions;
    simOptions.parameter_overrides({{"map_file", mapFile}, {"slip", true}});
    rclcpp::NodeOptions locOptions;
    locOptions.parameter_overrides({{"map_file", mapFile}});

    auto sim = std::make_shared<robot_sim_ros::SimNode>(simOptions);
    auto localizer = std::make_shared<robot_sim_ros::LocalizerNode>(locOptions);
    auto navigator = std::make_shared<robot_sim_ros::NavigatorNode>();
    auto client = std::make_shared<rclcpp::Node>("test_client");
    auto goalPub = client->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);

    rclcpp::executors::SingleThreadedExecutor executor;
    for (const auto& node : std::initializer_list<rclcpp::Node::SharedPtr>{sim, localizer, navigator, client})
        executor.add_node(node);

    // Goal: the centre of cell (6, 2), about 2.5 m from the start.
    const Point goal = sim->scenario().grid.cellCenter({6, 2});
    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = "map";
    msg.pose.position.x = goal.x;
    msg.pose.position.y = goal.y;
    msg.pose.orientation.w = 1.0;

    const auto start = std::chrono::steady_clock::now();
    bool goalSent = false;
    double distance = 1e9;
    while (std::chrono::steady_clock::now() - start < 60s) {
        executor.spin_some(10ms);
        const auto elapsed = std::chrono::steady_clock::now() - start;
        // Give the nodes a moment to connect (map latched, TF flowing), then send the goal.
        if (!goalSent && elapsed > 2s) {
            msg.header.stamp = client->now();
            goalPub->publish(msg);
            goalSent = true;
        }
        const Pose p = sim->truePose();
        distance = std::hypot(p.x - goal.x, p.y - goal.y);
        if (goalSent && navigator->status() == NavStatus::Arrived) break;
        std::this_thread::sleep_for(1ms);
    }
    EXPECT_EQ(navigator->status(), NavStatus::Arrived);
    // "Arrived" means the ESTIMATE is within the goal tolerance (0.12 m), and
    // the estimate may be up to 0.2 m off (checked below), so the true robot
    // must be within their sum.
    EXPECT_LT(distance, 0.12 + 0.2) << "the robot must really be at the goal, not just believe it";
    const Pose est = localizer->estimate();
    const Pose truth = sim->truePose();
    EXPECT_LT(std::hypot(est.x - truth.x, est.y - truth.y), 0.2) << "the localizer must agree with the truth";
}
