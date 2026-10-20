#include <rclcpp/rclcpp.hpp>

#include "robot_sim_ros/mapper_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<robot_sim_ros::MapperNode>());
    rclcpp::shutdown();
    return 0;
}
