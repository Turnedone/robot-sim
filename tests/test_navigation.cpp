// Checks A* planning, and then the whole robot working on its own: it starts
// knowing nothing, maps with its lidar, plans, drives, and must reach the goal.
// Run with: ctest --test-dir build   (or .\build\test_navigation.exe)

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

#include "lidar.hpp"
#include "navigator.hpp"
#include "occupancy_map.hpp"
#include "planner.hpp"

namespace {

int failures = 0;

void check(bool ok, const char* what) {
    std::printf("%s  %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) ++failures;
}

bool onPath(const std::vector<Pos>& path, Pos p) {
    for (Pos q : path)
        if (q == p) return true;
    return false;
}

struct Mission {
    bool arrived = false;
    double seconds = 0;
    int bumps = 0;
    double driven = 0;
    int replans = 0;
    int avoided = 0;
};

// The full loop, exactly as the viewer runs it but without graphics:
// physics at 50 Hz, lidar + mapping + re-planning at 5 Hz.
Mission runMission(const Scenario& s, unsigned noiseSeed, double timeLimit) {
    const Point c = s.grid.cellCenter(s.start);
    DiffDriveRobot robot(DiffDriveConfig{}, {c.x, c.y, 0});
    Lidar lidar(LidarConfig{}, noiseSeed);
    OccupancyMap map(s.grid.width(), s.grid.height(), s.grid.resolution());
    Navigator nav;
    nav.setGoal(s.goal);
    map.integrate(lidar.scan(s.grid, robot.pose()), robot.pose());
    nav.replan(map, robot.pose());

    Mission m;
    double scanTimer = 0;
    bool touching = false;
    while (m.seconds < timeLimit && nav.status() != NavStatus::Arrived) {
        robot.setCommand(nav.command(robot.pose(), map));
        const bool moved = robot.step(0.02, s.grid);
        if (!moved && !touching) ++m.bumps;  // count each new contact once
        touching = !moved;
        m.seconds += 0.02;
        scanTimer += 0.02;
        if (scanTimer >= 0.2) {
            scanTimer -= 0.2;
            map.integrate(lidar.scan(s.grid, robot.pose()), robot.pose());
            nav.replan(map, robot.pose());
        }
    }
    m.arrived = nav.status() == NavStatus::Arrived;
    m.driven = robot.distanceTravelled();
    m.replans = nav.replans();
    m.avoided = nav.collisionsAvoided();
    return m;
}

}  // namespace

int main() {
    {
        const OccupancyMap map(10, 10, 0.5);  // nothing known yet
        const auto path = planPath(map, {0, 0}, {4, 0});
        check(path && path->size() == 5 && path->front() == Pos{0, 0} && path->back() == Pos{4, 0},
              "open map: a straight row of 5 cells");
    }
    {
        OccupancyMap map(10, 10, 0.5);
        for (int y = 0; y < 10; ++y)
            if (y != 7) map.set({5, y}, Occupancy::Occupied);  // a wall with one gap at row 7
        const auto path = planPath(map, {1, 1}, {8, 1});
        check(path && onPath(*path, {5, 7}), "wall with a gap: the path goes through the gap");
    }
    {
        OccupancyMap map(10, 10, 0.5);
        for (int i = 3; i <= 5; ++i) {
            map.set({i, 3}, Occupancy::Occupied);
            map.set({i, 5}, Occupancy::Occupied);
            map.set({3, i}, Occupancy::Occupied);
            map.set({5, i}, Occupancy::Occupied);
        }
        check(!planPath(map, {0, 0}, {4, 4}), "goal walled in on all sides: no path");
        check(!planPath(map, {0, 0}, {3, 3}), "goal is a wall: no path");
    }
    {
        OccupancyMap map(10, 10, 0.5);
        map.set({1, 0}, Occupancy::Occupied);
        map.set({0, 1}, Occupancy::Occupied);
        check(!planPath(map, {0, 0}, {1, 1}), "no squeezing diagonally between two walls' corners");
    }
    {
        // Two equal-length routes: one through known free space, one through unknown.
        OccupancyMap map(10, 10, 0.5);
        for (int x = 0; x < 10; ++x)
            for (int y = 0; y < 10; ++y) map.set({x, y}, Occupancy::Free);
        for (int y = 2; y <= 8; ++y) map.set({5, y}, Occupancy::Occupied);  // a pillar in the middle...
        for (int x = 0; x < 10; ++x) map.set({x, 9}, Occupancy::Unknown);    // ...and the bottom row unknown
        const auto path = planPath(map, {2, 5}, {8, 5});
        bool goesOver = false;
        bool goesUnder = false;
        if (path)
            for (Pos p : *path) {
                if (p.x == 5 && p.y <= 1) goesOver = true;
                if (p.x == 5 && p.y >= 9) goesUnder = true;
            }
        check(path && goesOver && !goesUnder,
              "prefers going round through known free space over unknown space");
    }

    // The whole robot, on its own, on both maps. Every run gets different
    // lidar noise, so one lucky run cannot make this pass.
    constexpr unsigned kRuns = 10;
    for (const char* name : {"warehouse", "maze"}) {
        const Scenario s = loadScenario(std::string(ROBOT_SIM_MAPS_DIR) + "/" + name + ".txt");
        unsigned arrived = 0;
        int bumps = 0;
        double slowest = 0;
        for (unsigned seed = 1; seed <= kRuns; ++seed) {
            const Mission m = runMission(s, seed, 600);
            if (m.arrived) ++arrived;
            bumps += m.bumps;
            slowest = std::max(slowest, m.seconds);
            if (seed == 1)
                std::printf("      (%s, run 1: %.1f s, %.1f m driven, %d re-plans, %d unsafe arcs refused)\n", name,
                            m.seconds, m.driven, m.replans, m.avoided);
        }
        std::printf("      (%s: %u/%u runs arrived, slowest %.1f s, %d bumps in total)\n", name, arrived, kRuns,
                    slowest, bumps);
        const std::string what = std::string(name) + ": starts knowing nothing, reaches the goal on its own (10/10 runs)";
        check(arrived == kRuns, what.c_str());
        const std::string safe = std::string(name) + ": without touching a single wall in any run";
        check(bumps == 0, safe.c_str());
    }

    std::printf("\n%s (%d failed)\n", failures == 0 ? "ALL PASSED" : "SOME FAILED", failures);
    return failures == 0 ? 0 : 1;
}
