#include "lidar.hpp"

#include <cmath>
#include <limits>

#include "raycast.hpp"

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

}  // namespace

double castRay(const Grid& grid, Point origin, double angle, double maxRange) {
    double hit = kInf;
    walkRay(grid.height(), grid.resolution(), origin, angle, maxRange, [&](Pos cell, double enter) {
        if (grid.at(cell) != Cell::Wall) return true;  // keep going (outside the map counts as wall)
        hit = enter;
        return false;
    });
    return hit;
}

Lidar::Lidar(const LidarConfig& config, unsigned seed) : config_(config), rng_(seed) {}

LaserScan Lidar::scan(const Grid& grid, const Pose& pose) {
    LaserScan s;
    s.angleMin = -kPi;
    s.angleIncrement = 2 * kPi / config_.beams;
    s.rangeMin = config_.rangeMin;
    s.rangeMax = config_.rangeMax;
    s.ranges.resize(static_cast<size_t>(config_.beams));

    // std::normal_distribution needs a spread above zero, so a perfect sensor skips it.
    const bool noisy = config_.noiseStdDev > 0;
    std::normal_distribution<double> noise(0.0, noisy ? config_.noiseStdDev : 1.0);
    std::uniform_real_distribution<double> chance(0.0, 1.0);

    for (size_t i = 0; i < s.ranges.size(); ++i) {
        const double trueRange = castRay(grid, {pose.x, pose.y}, pose.theta + s.angle(i), config_.rangeMax);
        double reading = trueRange + (noisy ? noise(rng_) : 0.0);
        if (chance(rng_) < config_.dropoutChance) reading = kInf;  // e.g. a black or shiny surface
        if (reading < config_.rangeMin || reading > config_.rangeMax) reading = kInf;
        s.ranges[i] = reading;
    }
    return s;
}
