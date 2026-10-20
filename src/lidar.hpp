#pragma once

#include <random>
#include <vector>

#include "grid.hpp"
#include "robot.hpp"

// One full turn of the lidar: a distance per beam angle.
// The same layout as ROS's sensor_msgs/LaserScan.
struct LaserScan {
    double angleMin = 0;        // angle of beam 0, relative to the robot's heading, rad
    double angleIncrement = 0;  // angle between neighbouring beams, rad
    double rangeMin = 0;        // readings closer than this are not valid, m
    double rangeMax = 0;        // readings further than this are not valid, m
    std::vector<double> ranges; // m; infinity = no return (nothing in range, or a dropout)

    double angle(size_t i) const { return angleMin + static_cast<double>(i) * angleIncrement; }
};

// Roughly the TurtleBot3's LDS-01 lidar.
struct LidarConfig {
    int beams = 180;             // one every 2 degrees, all the way round
    double rangeMin = 0.12;      // m
    double rangeMax = 3.5;       // m
    double noiseStdDev = 0.015;  // m, Gaussian noise on every reading
    double dropoutChance = 0.01; // chance that a beam returns nothing at all
    double rateHz = 5.0;         // scans per second
};

// Distance from `origin` along `angle` (map frame) to the first wall, using a
// DDA grid walk: it visits exactly the cells the ray passes through, in order.
// Returns infinity if no wall is found within maxRange.
double castRay(const Grid& grid, Point origin, double angle, double maxRange);

// A simulated lidar: perfect ray casting plus realistic imperfections.
class Lidar {
public:
    explicit Lidar(const LidarConfig& config, unsigned seed = 42);

    // Takes one scan from the robot's pose. Beam angles are relative to the
    // robot's heading, as on a real robot (the sensor turns with the robot).
    LaserScan scan(const Grid& grid, const Pose& pose);

    const LidarConfig& config() const { return config_; }

private:
    LidarConfig config_;
    std::mt19937 rng_;  // seeded, so a run can be repeated exactly
};
