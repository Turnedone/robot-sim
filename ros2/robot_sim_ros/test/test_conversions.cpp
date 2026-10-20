#include <gtest/gtest.h>

#include <cmath>

#include "robot_sim_ros/conversions.hpp"

using robot_sim_ros::compose;
using robot_sim_ros::fromOccupancyGrid;
using robot_sim_ros::inverse;
using robot_sim_ros::quaternionToYaw;
using robot_sim_ros::toOccupancyGrid;
using robot_sim_ros::yawToQuaternion;

TEST(Conversions, YawSurvivesAQuaternionRoundTrip) {
    for (double yaw : {0.0, 0.5, -1.2, 3.0, -3.0}) EXPECT_NEAR(quaternionToYaw(yawToQuaternion(yaw)), yaw, 1e-12);
}

TEST(Conversions, ComposeWithInverseIsIdentity) {
    const Pose a{1.5, -2.0, 0.7};
    const Pose id = compose(a, inverse(a));
    EXPECT_NEAR(id.x, 0, 1e-12);
    EXPECT_NEAR(id.y, 0, 1e-12);
    EXPECT_NEAR(id.theta, 0, 1e-12);
}

TEST(Conversions, MapToOdomFromEstimateAndOdometry) {
    // If the robot is at `estimate` in the map and odometry says `odom`, then
    // map->odom composed with odom->base_link must give back the estimate.
    const Pose estimate{4.0, 2.0, 1.0};
    const Pose odom{3.5, 2.4, 0.8};
    const Pose mapToOdom = compose(estimate, inverse(odom));
    const Pose back = compose(mapToOdom, odom);
    EXPECT_NEAR(back.x, estimate.x, 1e-12);
    EXPECT_NEAR(back.y, estimate.y, 1e-12);
    EXPECT_NEAR(back.theta, estimate.theta, 1e-12);
}

TEST(Conversions, OccupancyGridRowsGoBottomUp) {
    OccupancyMap map(4, 3, 0.5);
    map.set({1, 0}, Occupancy::Occupied);  // top row in robot-sim
    map.set({2, 2}, Occupancy::Free);      // bottom row in robot-sim
    const nav_msgs::msg::OccupancyGrid g = toOccupancyGrid(map, "map", builtin_interfaces::msg::Time());
    ASSERT_EQ(g.data.size(), 12U);
    EXPECT_EQ(g.data[2 * 4 + 1], 97) << "robot-sim's top row is ROS's LAST row";
    EXPECT_EQ(g.data[0 * 4 + 2], 12) << "robot-sim's bottom row is ROS's FIRST row";
    EXPECT_EQ(g.data[5], -1) << "unknown";

    const OccupancyMap back = fromOccupancyGrid(g);
    EXPECT_EQ(back.classify({1, 0}), Occupancy::Occupied);
    EXPECT_EQ(back.classify({2, 2}), Occupancy::Free);
    EXPECT_EQ(back.classify({0, 0}), Occupancy::Unknown);
}

TEST(Conversions, LaserScanKeepsMissingReadingsMissing) {
    LaserScan s;
    s.angleMin = -kPi;
    s.angleIncrement = kPi / 2;
    s.rangeMin = 0.12;
    s.rangeMax = 3.5;
    s.ranges = {1.0, std::numeric_limits<double>::infinity(), 2.5, 0.5};
    const LaserScan back = robot_sim_ros::fromLaserScanMsg(
        robot_sim_ros::toLaserScanMsg(s, "base_link", builtin_interfaces::msg::Time(), 0.2));
    ASSERT_EQ(back.ranges.size(), 4U);
    EXPECT_NEAR(back.ranges[0], 1.0, 1e-6);
    EXPECT_TRUE(std::isinf(back.ranges[1]));
    EXPECT_NEAR(back.angle(2), 0.0, 1e-6);
}
