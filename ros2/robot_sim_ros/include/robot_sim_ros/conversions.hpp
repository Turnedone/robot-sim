#pragma once

// Conversions between the robot-sim core types and ROS 2 messages.
// The core knows nothing about ROS; everything ROS-specific is here.

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "lidar.hpp"
#include "occupancy_map.hpp"
#include "robot.hpp"

namespace robot_sim_ros {

// A heading (yaw) as a quaternion: a rotation about the z axis.
inline geometry_msgs::msg::Quaternion yawToQuaternion(double yaw) {
    geometry_msgs::msg::Quaternion q;
    q.z = std::sin(yaw / 2);
    q.w = std::cos(yaw / 2);
    return q;
}

inline double quaternionToYaw(const geometry_msgs::msg::Quaternion& q) {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

inline geometry_msgs::msg::Pose toRosPose(const Pose& p) {
    geometry_msgs::msg::Pose m;
    m.position.x = p.x;
    m.position.y = p.y;
    m.orientation = yawToQuaternion(p.theta);
    return m;
}

inline Pose fromRosPose(const geometry_msgs::msg::Pose& m) {
    return {m.position.x, m.position.y, quaternionToYaw(m.orientation)};
}

// 2D rigid transforms, written as poses: compose(a, b) applies b in a's frame.
inline Pose compose(const Pose& a, const Pose& b) {
    const double c = std::cos(a.theta), s = std::sin(a.theta);
    return {a.x + c * b.x - s * b.y, a.y + s * b.x + c * b.y, normalizeAngle(a.theta + b.theta)};
}

inline Pose inverse(const Pose& a) {
    const double c = std::cos(a.theta), s = std::sin(a.theta);
    return {-(c * a.x + s * a.y), -(-s * a.x + c * a.y), normalizeAngle(-a.theta)};
}

inline geometry_msgs::msg::TransformStamped toTransform(const Pose& p, const std::string& parent,
                                                        const std::string& child,
                                                        const builtin_interfaces::msg::Time& stamp) {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = stamp;
    t.header.frame_id = parent;
    t.child_frame_id = child;
    t.transform.translation.x = p.x;
    t.transform.translation.y = p.y;
    t.transform.rotation = yawToQuaternion(p.theta);
    return t;
}

inline Pose fromTransform(const geometry_msgs::msg::TransformStamped& t) {
    return {t.transform.translation.x, t.transform.translation.y, quaternionToYaw(t.transform.rotation)};
}

// ROS occupancy grids store rows from the BOTTOM of the map up (row 0 at the
// origin, y growing up); robot-sim stores row 0 at the TOP. Values: -1 unknown,
// 0..100 = probability of being occupied in percent.
inline nav_msgs::msg::OccupancyGrid toOccupancyGrid(const OccupancyMap& map, const std::string& frame,
                                                    const builtin_interfaces::msg::Time& stamp) {
    nav_msgs::msg::OccupancyGrid g;
    g.header.frame_id = frame;
    g.header.stamp = stamp;
    g.info.resolution = static_cast<float>(map.resolution());
    g.info.width = static_cast<uint32_t>(map.width());
    g.info.height = static_cast<uint32_t>(map.height());
    g.info.origin.orientation.w = 1.0;
    g.data.resize(static_cast<size_t>(map.width()) * static_cast<size_t>(map.height()));
    for (int rowUp = 0; rowUp < map.height(); ++rowUp) {
        for (int x = 0; x < map.width(); ++x) {
            const Pos cell{x, map.height() - 1 - rowUp};
            const int8_t value = map.observed(cell) ? static_cast<int8_t>(std::lround(100.0 * map.probability(cell)))
                                                    : static_cast<int8_t>(-1);
            g.data[static_cast<size_t>(rowUp) * static_cast<size_t>(map.width()) + static_cast<size_t>(x)] = value;
        }
    }
    return g;
}

// The other way: a ROS occupancy grid (with its origin at 0, 0) as a map the
// planner can use. >= 65% occupied counts as a wall, <= 25% as free.
inline OccupancyMap fromOccupancyGrid(const nav_msgs::msg::OccupancyGrid& g) {
    const int width = static_cast<int>(g.info.width);
    const int height = static_cast<int>(g.info.height);
    OccupancyMap map(width, height, g.info.resolution);
    for (int rowUp = 0; rowUp < height; ++rowUp) {
        for (int x = 0; x < width; ++x) {
            const int8_t v = g.data[static_cast<size_t>(rowUp) * static_cast<size_t>(width) + static_cast<size_t>(x)];
            const Pos cell{x, height - 1 - rowUp};
            if (v >= 65)
                map.set(cell, Occupancy::Occupied);
            else if (v >= 0 && v <= 25)
                map.set(cell, Occupancy::Free);
        }
    }
    return map;
}

inline sensor_msgs::msg::LaserScan toLaserScanMsg(const LaserScan& s, const std::string& frame,
                                                  const builtin_interfaces::msg::Time& stamp, double scanTime) {
    sensor_msgs::msg::LaserScan m;
    m.header.frame_id = frame;
    m.header.stamp = stamp;
    m.angle_min = static_cast<float>(s.angleMin);
    m.angle_increment = static_cast<float>(s.angleIncrement);
    m.angle_max = static_cast<float>(s.angle(s.ranges.empty() ? 0 : s.ranges.size() - 1));
    m.range_min = static_cast<float>(s.rangeMin);
    m.range_max = static_cast<float>(s.rangeMax);
    m.scan_time = static_cast<float>(scanTime);
    m.ranges.reserve(s.ranges.size());
    for (double r : s.ranges) m.ranges.push_back(static_cast<float>(r));  // infinity stays infinity
    return m;
}

inline LaserScan fromLaserScanMsg(const sensor_msgs::msg::LaserScan& m) {
    LaserScan s;
    s.angleMin = m.angle_min;
    s.angleIncrement = m.angle_increment;
    s.rangeMin = m.range_min;
    s.rangeMax = m.range_max;
    s.ranges.reserve(m.ranges.size());
    for (float r : m.ranges) {
        const bool valid = std::isfinite(r) && r >= m.range_min && r <= m.range_max;
        s.ranges.push_back(valid ? static_cast<double>(r) : std::numeric_limits<double>::infinity());
    }
    return s;
}

}  // namespace robot_sim_ros
