// Checks the simulated lidar against distances worked out by hand.
// Run with: ctest --test-dir build   (or .\build\test_lidar.exe)

#include <cmath>
#include <cstdio>
#include <vector>

#include "lidar.hpp"

namespace {

int failures = 0;

void check(bool ok, const char* what) {
    std::printf("%s  %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) ++failures;
}

bool near(double a, double b, double tolerance = 1e-9) { return std::abs(a - b) < tolerance; }

// A perfect sensor, so expected distances are exact.
LidarConfig perfect() {
    LidarConfig c;
    c.noiseStdDev = 0;
    c.dropoutChance = 0;
    return c;
}

}  // namespace

int main() {
    const double pi = std::acos(-1.0);

    // A 5 m x 5 m room: 10 x 10 cells of 0.5 m, no walls inside. Outside the map
    // counts as wall, so the room's edges are at x = 0, x = 5, y = 0, y = 5.
    const Grid room(10, 10, 0.5);
    const Point middle{2.5, 2.5};

    check(near(castRay(room, middle, 0, 10), 2.5), "ray right from the middle hits the wall 2.5 m away");
    check(near(castRay(room, middle, pi / 2, 10), 2.5), "ray up hits the top wall 2.5 m away");
    check(near(castRay(room, middle, pi, 10), 2.5), "ray left hits the left wall 2.5 m away");
    check(near(castRay(room, middle, pi / 4, 10), 2.5 * std::sqrt(2.0)),
          "diagonal ray hits the far corner at 2.5 * sqrt(2) m");
    check(near(castRay(room, {1.0, 1.3}, 0, 10), 4.0), "ray from an off-centre point measures to the cell edge");
    check(std::isinf(castRay(room, middle, 0, 2.0)), "no wall within max range -> infinity");

    {
        Grid g(10, 10, 0.5);
        for (int row = 0; row < 10; ++row) g.set({7, row}, Cell::Wall);  // a wall whose face is at x = 3.5
        check(near(castRay(g, {1.0, 2.5}, 0, 10), 2.5), "ray stops at the first wall, not the room edge");
    }

    {
        Lidar lidar(perfect());
        const LaserScan s = lidar.scan(room, {2.5, 2.5, pi / 2});  // robot facing up
        // Beam angles are relative to the robot: angle 0 is straight ahead (= up here).
        const size_t ahead = static_cast<size_t>(std::lround(-s.angleMin / s.angleIncrement));
        check(s.ranges.size() == 180 && near(s.angle(ahead), 0), "scan has 180 beams and one points straight ahead");
        check(near(s.ranges[ahead], 2.5), "beam straight ahead measures the wall the robot faces");
    }
    {
        Lidar lidar(perfect());
        const LaserScan s = lidar.scan(room, {0.05, 2.5, pi});  // 5 cm from the left wall, facing it
        const size_t ahead = static_cast<size_t>(std::lround(-s.angleMin / s.angleIncrement));
        check(std::isinf(s.ranges[ahead]), "closer than the 0.12 m minimum range -> no valid reading");
    }
    {
        // Noise: many readings of the same 2.5 m wall should average 2.5 m with ~1.5 cm spread.
        LidarConfig c = perfect();
        c.noiseStdDev = 0.015;
        Lidar lidar(c, 7);
        std::vector<double> readings;
        for (int i = 0; i < 200; ++i) {
            const LaserScan s = lidar.scan(room, {2.5, 2.5, 0});
            readings.push_back(s.ranges[90]);  // beam 90 points straight ahead
        }
        double mean = 0;
        for (double r : readings) mean += r;
        mean /= readings.size();
        double var = 0;
        for (double r : readings) var += (r - mean) * (r - mean);
        const double stddev = std::sqrt(var / (readings.size() - 1));
        check(near(mean, 2.5, 0.003), "noise: readings average out to the true distance");
        check(near(stddev, 0.015, 0.003), "noise: readings spread by about 1.5 cm");
    }
    {
        Lidar a(LidarConfig{}, 123);
        Lidar b(LidarConfig{}, 123);
        check(a.scan(room, {2.5, 2.5, 0}).ranges == b.scan(room, {2.5, 2.5, 0}).ranges,
              "same seed -> identical scans (runs can be repeated)");
    }

    std::printf("\n%s (%d failed)\n", failures == 0 ? "ALL PASSED" : "SOME FAILED", failures);
    return failures == 0 ? 0 : 1;
}
