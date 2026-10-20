#include <rclcpp/rclcpp.hpp>

#include "robot_sim_ros/navigator_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<robot_sim_ros::NavigatorNode>());
    rclcpp::shutdown();
    return 0;
}
