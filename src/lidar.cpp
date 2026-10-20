#include "lidar.hpp"

#include <cmath>
#include <limits>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kInf = std::numeric_limits<double>::infinity();

}  // namespace

double castRay(const Grid& grid, Point origin, double angle, double maxRange) {
    // Work in cell units with rows counted from the BOTTOM, so x and y both
    // grow the same way as in the map frame.
    const double res = grid.resolution();
    const double ox = origin.x / res;
    const double oy = origin.y / res;
    const double dx = std::cos(angle);
    const double dy = std::sin(angle);

    int col = static_cast<int>(std::floor(ox));
    int rowUp = static_cast<int>(std::floor(oy));
    const int stepX = dx > 0 ? 1 : -1;
    const int stepY = dy > 0 ? 1 : -1;

    // tDelta: how far along the ray (in cells) to cross one whole cell in x / y.
    // tMax:   how far along the ray the NEXT vertical / horizontal grid line is.
    const double tDeltaX = dx != 0 ? std::abs(1 / dx) : kInf;
    const double tDeltaY = dy != 0 ? std::abs(1 / dy) : kInf;
    double tMaxX = dx > 0 ? (col + 1 - ox) / dx : dx < 0 ? (ox - col) / -dx : kInf;
    double tMaxY = dy > 0 ? (rowUp + 1 - oy) / dy : dy < 0 ? (oy - rowUp) / -dy : kInf;

    const double maxT = maxRange / res;
    double t = 0;
    while (t <= maxT) {
        if (grid.at({col, grid.height() - 1 - rowUp}) == Cell::Wall) return t * res;
        // Step into whichever neighbouring cell the ray reaches first.
        if (tMaxX < tMaxY) {
            t = tMaxX;
            tMaxX += tDeltaX;
            col += stepX;
        } else {
            t = tMaxY;
            tMaxY += tDeltaY;
            rowUp += stepY;
        }
    }
    return kInf;
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
