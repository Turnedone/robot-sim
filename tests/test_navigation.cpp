// The whole robot closing the loop on its own: it starts knowing nothing,
// maps with its lidar, plans, drives, and must reach the goal without
// touching a wall. Every run gets different lidar noise, so one lucky run
// cannot make these tests pass.

#include <gtest/gtest.h>

#include <string>

#include "lidar.hpp"
#include "navigator.hpp"

namespace {

struct Mission {
    bool arrived = false;
    double seconds = 0;
    int bumps = 0;
};

// The full loop, as the viewer runs it but without graphics:
// physics and control at 50 Hz, lidar + mapping + re-planning at 5 Hz.
Mission runMission(const Scenario& s, unsigned noiseSeed, double timeLimit) {
    const DiffDriveConfig robotConfig;
    const Point c = s.grid.cellCenter(s.start);
    DiffDriveRobot robot(robotConfig, {c.x, c.y, 0});
    Lidar lidar(LidarConfig{}, noiseSeed);
    OccupancyMap map(s.grid.width(), s.grid.height(), s.grid.resolution());
    NavigatorConfig navConfig;
    navConfig.robotRadius = robotConfig.radius;
    Navigator nav(navConfig);
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
    return m;
}

struct Case {
    const char* map;
    unsigned seed;
};

std::string caseName(const ::testing::TestParamInfo<Case>& info) {
    return std::string(info.param.map) + "_seed" + std::to_string(info.param.seed);
}

std::vector<Case> allCases() {
    std::vector<Case> cases;
    for (const char* map : {"warehouse", "maze"})
        for (unsigned seed = 1; seed <= 10; ++seed) cases.push_back({map, seed});
    return cases;
}

class ClosedLoop : public ::testing::TestWithParam<Case> {};

}  // namespace

TEST_P(ClosedLoop, ReachesTheGoalWithoutTouchingAWall) {
    const Case& c = GetParam();
    const Scenario s = loadScenario(std::string(ROBOT_SIM_MAPS_DIR) + "/" + c.map + ".txt");
    const Mission m = runMission(s, c.seed, 600);
    EXPECT_TRUE(m.arrived) << "did not reach the goal within " << m.seconds << " s";
    EXPECT_EQ(m.bumps, 0);
    RecordProperty("seconds", std::to_string(m.seconds));
}

INSTANTIATE_TEST_SUITE_P(BothMapsTenNoiseSeeds, ClosedLoop, ::testing::ValuesIn(allCases()), caseName);
