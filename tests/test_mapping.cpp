// Checks that the robot's map, built only from lidar scans, matches the real world.
// Run with: ctest --test-dir build   (or .\build\test_mapping.exe)

#include <cmath>
#include <cstdio>

#include "lidar.hpp"
#include "occupancy_map.hpp"

namespace {

int failures = 0;

void check(bool ok, const char* what) {
    std::printf("%s  %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) ++failures;
}

bool near(double a, double b, double tolerance = 1e-9) { return std::abs(a - b) < tolerance; }

LidarConfig perfect() {
    LidarConfig c;
    c.noiseStdDev = 0;
    c.dropoutChance = 0;
    return c;
}

// A 5 m x 5 m room: 10 x 10 cells with walls all round and one pillar at (5, 5).
Grid roomWithPillar() {
    Grid g(10, 10, 0.5);
    for (int i = 0; i < 10; ++i) {
        g.set({i, 0}, Cell::Wall);
        g.set({i, 9}, Cell::Wall);
        g.set({0, i}, Cell::Wall);
        g.set({9, i}, Cell::Wall);
    }
    g.set({5, 5}, Cell::Wall);
    return g;
}

Pose poseAtCell(const Grid& g, Pos cell) {
    const Point c = g.cellCenter(cell);
    return {c.x, c.y, 0};
}

void scanRepeatedly(OccupancyMap& map, Lidar& lidar, const Grid& world, const Pose& pose, int times) {
    for (int i = 0; i < times; ++i) map.integrate(lidar.scan(world, pose), pose);
}

}  // namespace

int main() {
    check(near(logOddsToProbability(0), 0.5), "log-odds 0 means 50%");
    check(near(logOddsToProbability(std::log(0.7 / 0.3)), 0.7), "log-odds converts back to probability");

    Grid world = roomWithPillar();
    const Pose robot = poseAtCell(world, {2, 5});  // same row as the pillar, 3 cells left of it

    {
        const OccupancyMap map(10, 10, 0.5);
        check(map.classify({3, 3}) == Occupancy::Unknown && near(map.probability({3, 3}), 0.5),
              "a new map knows nothing: every cell is unknown, 50%");
    }
    {
        OccupancyMap map(10, 10, 0.5);
        Lidar lidar(perfect());
        scanRepeatedly(map, lidar, world, robot, 1);
        check(map.classify({3, 5}) == Occupancy::Unknown && map.probability({3, 5}) < 0.5,
              "one scan: a cell the beams crossed leans free, but one vote is not enough");
        check(map.classify({5, 5}) == Occupancy::Occupied, "one scan: the pillar the beams hit is occupied");
    }
    {
        OccupancyMap map(10, 10, 0.5);
        Lidar lidar(perfect());
        scanRepeatedly(map, lidar, world, robot, 3);
        check(map.classify({3, 5}) == Occupancy::Free, "three scans: the gap in front of the pillar is free");
        check(map.classify({9, 3}) == Occupancy::Occupied, "three scans: the outer wall is occupied");
        check(!map.observed({9, 2}), "a wall 3.69 m away, beyond the 3.5 m lidar range, stays unknown");
        check(map.classify({7, 5}) == Occupancy::Unknown && !map.observed({7, 5}),
              "the cell hidden behind the pillar stays unknown (no beam can reach it)");
    }
    {
        // The pillar is taken away: beams now pass through where it was, and the
        // map changes its mind. Without the log-odds cap this would take far longer.
        OccupancyMap map(10, 10, 0.5);
        Lidar lidar(perfect());
        scanRepeatedly(map, lidar, world, robot, 30);
        Grid moved = world;
        moved.set({5, 5}, Cell::Free);
        scanRepeatedly(map, lidar, moved, robot, 15);
        check(map.classify({5, 5}) == Occupancy::Free, "a removed obstacle fades from the map");
    }
    {
        // A realistic lidar (noise + dropouts) from a few places: every cell the map
        // is sure about must match reality.
        OccupancyMap map(10, 10, 0.5);
        Lidar lidar(LidarConfig{}, 99);
        for (Pos at : {Pos{2, 5}, Pos{2, 2}, Pos{7, 2}, Pos{7, 7}, Pos{3, 7}})
            scanRepeatedly(map, lidar, world, poseAtCell(world, at), 10);
        int sure = 0;
        int wrong = 0;
        for (int y = 0; y < 10; ++y) {
            for (int x = 0; x < 10; ++x) {
                const Occupancy o = map.classify({x, y});
                if (o == Occupancy::Unknown) continue;
                ++sure;
                const bool isWall = world.at({x, y}) == Cell::Wall;
                if ((o == Occupancy::Occupied) != isWall) ++wrong;
            }
        }
        std::printf("      (noisy lidar: %d cells classified, %d wrong)\n", sure, wrong);
        // The 4 corner cells can never be seen: the walls next to them always block the beam.
        check(sure >= 96 && wrong == 0, "noisy lidar from 5 places: the room is mapped with no mistakes");
    }

    std::printf("\n%s (%d failed)\n", failures == 0 ? "ALL PASSED" : "SOME FAILED", failures);
    return failures == 0 ? 0 : 1;
}
