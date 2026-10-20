// The simulated lidar, checked against distances worked out by hand.

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "lidar.hpp"

namespace {

constexpr double kEps = 1e-9;

// A perfect sensor, so expected distances are exact.
LidarConfig perfect() {
    LidarConfig c;
    c.noiseStdDev = 0;
    c.dropoutChance = 0;
    return c;
}

// A 5 m x 5 m room: 10 x 10 cells of 0.5 m, nothing inside. Outside the map
// counts as wall, so the room's edges are at x = 0, x = 5, y = 0 and y = 5.
const Grid& room() {
    static const Grid g(10, 10, 0.5);
    return g;
}

constexpr Point kMiddle{2.5, 2.5};

size_t beamStraightAhead(const LaserScan& s) {
    return static_cast<size_t>(std::lround(-s.angleMin / s.angleIncrement));
}

}  // namespace

TEST(CastRay, StraightLinesHitTheWalls) {
    EXPECT_NEAR(castRay(room(), kMiddle, 0, 10), 2.5, kEps);
    EXPECT_NEAR(castRay(room(), kMiddle, kPi / 2, 10), 2.5, kEps);
    EXPECT_NEAR(castRay(room(), kMiddle, kPi, 10), 2.5, kEps);
}

TEST(CastRay, DiagonalHitsTheCorner) {
    EXPECT_NEAR(castRay(room(), kMiddle, kPi / 4, 10), 2.5 * std::sqrt(2.0), kEps);
}

TEST(CastRay, OffCentreOriginMeasuresToTheCellEdge) {
    EXPECT_NEAR(castRay(room(), {1.0, 1.3}, 0, 10), 4.0, kEps);
}

TEST(CastRay, NothingWithinMaxRangeIsInfinity) {
    EXPECT_TRUE(std::isinf(castRay(room(), kMiddle, 0, 2.0)));
}

TEST(CastRay, StopsAtTheFirstWall) {
    Grid g(10, 10, 0.5);
    for (int row = 0; row < 10; ++row) g.set({7, row}, Cell::Wall);  // a wall whose face is at x = 3.5
    EXPECT_NEAR(castRay(g, {1.0, 2.5}, 0, 10), 2.5, kEps);
}

TEST(Lidar, BeamAnglesAreRelativeToTheRobot) {
    Lidar lidar(perfect());
    const LaserScan s = lidar.scan(room(), {2.5, 2.5, kPi / 2});  // robot facing up
    ASSERT_EQ(s.ranges.size(), 180U);
    const size_t ahead = beamStraightAhead(s);
    EXPECT_NEAR(s.angle(ahead), 0, kEps);
    EXPECT_NEAR(s.ranges[ahead], 2.5, kEps) << "straight ahead = the top wall";
}

TEST(Lidar, CloserThanMinimumRangeIsNoReading) {
    Lidar lidar(perfect());
    const LaserScan s = lidar.scan(room(), {0.05, 2.5, kPi});  // 5 cm from the left wall, facing it
    EXPECT_TRUE(std::isinf(s.ranges[beamStraightAhead(s)]));
}

TEST(Lidar, NoiseAveragesOutWithTheConfiguredSpread) {
    LidarConfig c = perfect();
    c.noiseStdDev = 0.015;
    Lidar lidar(c, 7);
    std::vector<double> readings;
    readings.reserve(200);
    for (int i = 0; i < 200; ++i) readings.push_back(lidar.scan(room(), {2.5, 2.5, 0}).ranges[90]);

    double mean = 0;
    for (double r : readings) mean += r;
    mean /= static_cast<double>(readings.size());
    double var = 0;
    for (double r : readings) var += (r - mean) * (r - mean);
    const double stddev = std::sqrt(var / static_cast<double>(readings.size() - 1));

    EXPECT_NEAR(mean, 2.5, 0.003);
    EXPECT_NEAR(stddev, 0.015, 0.003);
}

TEST(Lidar, SameSeedGivesIdenticalScans) {
    Lidar a(LidarConfig{}, 123);
    Lidar b(LidarConfig{}, 123);
    EXPECT_EQ(a.scan(room(), {2.5, 2.5, 0}).ranges, b.scan(room(), {2.5, 2.5, 0}).ranges);
}
