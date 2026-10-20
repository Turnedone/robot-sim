// SLAM: mapping and localizing at the same time, with slipping wheels and no map.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <string>

#include "lidar.hpp"
#include "navigator.hpp"
#include "odometry.hpp"
#include "slam.hpp"

namespace {

DiffDriveConfig slippyRobot() {
    DiffDriveConfig c;
    c.leftWheelScale = 1.005;
    c.rightWheelScale = 0.995;
    c.slipNoise = 0.05;
    return c;
}

double distance(const Pose& a, const Pose& b) {
    return std::hypot(a.x - b.x, a.y - b.y);
}

Scenario load(const std::string& name) {
    return loadScenario(std::string(ROBOT_SIM_MAPS_DIR) + "/" + name + ".txt");
}

Pose startPose(const Scenario& s) {
    const Point c = s.grid.cellCenter(s.start);
    return {c.x, c.y, 0};
}

// Cells a map is sure about but has wrong.
int wrongCells(const OccupancyMap& map, const Grid& truth) {
    int wrong = 0;
    for (int y = 0; y < truth.height(); ++y)
        for (int x = 0; x < truth.width(); ++x) {
            const Occupancy o = map.classify({x, y});
            if (o != Occupancy::Unknown && (o == Occupancy::Occupied) != (truth.at({x, y}) == Cell::Wall)) ++wrong;
        }
    return wrong;
}

struct Result {
    bool arrived = false;
    int bumps = 0;
    double finalError = 0;
    double worstError = 0;
    int slamWrongCells = 0;
    int odometryWrongCells = 0;  // the same scans mapped from odometry poses, i.e. without SLAM
};

// The robot starts with no map and slipping wheels. With `actOnSlam` it plans on
// the SLAM map and drives on the SLAM estimate (the real thing); otherwise it
// patrols waypoints driven by the true pose (testing SLAM on its own).
Result run(const std::string& name, unsigned seed, bool actOnSlam, double timeLimit) {
    const Scenario s = load(name);
    const DiffDriveConfig rc = slippyRobot();
    DiffDriveRobot robot(rc, startPose(s), seed);
    Odometry odometry(rc.wheelBase, startPose(s));
    Lidar lidar(LidarConfig{}, seed + 100);
    GridSlam slam(s.grid.width(), s.grid.height(), s.grid.resolution(), startPose(s), SlamConfig{}, seed + 300);
    OccupancyMap odometryMap(s.grid.width(), s.grid.height(), s.grid.resolution());
    const OccupancyMap truthMap = knownMap(s.grid);

    NavigatorConfig navConfig;
    navConfig.robotRadius = rc.radius;
    Navigator nav(navConfig);
    const bool maze = name == "maze";
    const Pos warehouseWaypoints[] = {{20, 3}, {1, 11}, {12, 1}, {28, 11}, {26, 3}, {8, 9}};
    const Pos mazeWaypoints[] = {{24, 11}, {1, 11}, {12, 9}, {24, 1}, {16, 1}, {1, 7}};
    const Pos* waypoints = maze ? mazeWaypoints : warehouseWaypoints;
    int nextWaypoint = 0;
    nav.setGoal(actOnSlam ? s.goal : waypoints[nextWaypoint++]);

    const auto plan = [&] {
        if (actOnSlam)
            nav.replan(slam.map(), slam.estimate());
        else
            nav.replan(truthMap, robot.pose());
    };
    plan();

    Result r;
    Pose lastOdometry = odometry.pose();
    double t = 0;
    double scanTimer = 0;
    bool touching = false;
    while (t < timeLimit) {
        if (nav.status() == NavStatus::Arrived) {
            if (actOnSlam) break;
            nav.setGoal(waypoints[nextWaypoint++ % 6]);
            plan();
        }
        robot.setCommand(actOnSlam ? nav.command(slam.estimate(), slam.map()) : nav.command(robot.pose(), truthMap));
        const bool moved = robot.step(0.02, s.grid);
        if (!moved && !touching) ++r.bumps;
        touching = !moved;
        odometry.update(robot.leftSpeed(), robot.rightSpeed(), 0.02);
        t += 0.02;
        scanTimer += 0.02;
        if (scanTimer >= 0.2) {
            scanTimer -= 0.2;
            const LaserScan scan = lidar.scan(s.grid, robot.pose());
            slam.predict(lastOdometry, odometry.pose());
            lastOdometry = odometry.pose();
            slam.update(scan);
            odometryMap.integrate(scan, odometry.pose());
            plan();
            r.worstError = std::max(r.worstError, distance(slam.estimate(), robot.pose()));
        }
    }
    r.arrived = nav.status() == NavStatus::Arrived;
    r.finalError = distance(slam.estimate(), robot.pose());
    r.slamWrongCells = wrongCells(slam.map(), s.grid);
    r.odometryWrongCells = wrongCells(odometryMap, s.grid);
    return r;
}

std::string describe(const Result& r) {
    return "arrived=" + std::to_string(r.arrived) + " bumps=" + std::to_string(r.bumps) +
           " finalErr=" + std::to_string(r.finalError) + " worst=" + std::to_string(r.worstError) +
           " slamWrong=" + std::to_string(r.slamWrongCells) + " odomWrong=" + std::to_string(r.odometryWrongCells);
}

}  // namespace

TEST(Slam, ScanFitsBestAtTheTruePose) {
    const Scenario s = load("warehouse");
    const OccupancyMap map = knownMap(s.grid);
    Lidar lidar(LidarConfig{}, 3);
    const Pose truth{3.25, 3.75, 0.3};
    const LaserScan scan = lidar.scan(s.grid, truth);
    const GridSlam slam(s.grid.width(), s.grid.height(), s.grid.resolution(), truth);
    const double atTruth = slam.scanFit(scan, map, truth);
    EXPECT_GT(atTruth, slam.scanFit(scan, map, {truth.x + 0.1, truth.y, truth.theta}));
    EXPECT_GT(atTruth, slam.scanFit(scan, map, {truth.x, truth.y, truth.theta + 0.1}));
}

TEST(Slam, ScanMatchingPullsAnOffsetGuessBackToTheTruth) {
    const Scenario s = load("warehouse");
    const OccupancyMap map = knownMap(s.grid);
    Lidar lidar(LidarConfig{}, 4);
    const Pose truth{3.25, 3.75, 0.3};
    const LaserScan scan = lidar.scan(s.grid, truth);
    SlamConfig noPrior;
    noPrior.priorSigmaXY = 0;  // the prior would (rightly) hold the guess back
    const GridSlam slam(s.grid.width(), s.grid.height(), s.grid.resolution(), truth, noPrior);
    const Pose matched = slam.matchScan(scan, map, {truth.x + 0.08, truth.y - 0.06, truth.theta + 0.05});
    EXPECT_LT(distance(matched, truth), 0.03);
    EXPECT_NEAR(normalizeAngle(matched.theta - truth.theta), 0.0, 0.02);
}

class SlamRuns : public ::testing::TestWithParam<unsigned> {};

TEST_P(SlamRuns, BuildsAnAccurateMapWhileOdometryDrifts) {
    for (const char* name : {"warehouse", "maze"}) {
        const Result r = run(name, GetParam(), false, 150);
        EXPECT_LT(r.worstError, 0.15) << name << ": " << describe(r);
        EXPECT_LT(r.finalError, 0.1) << name << ": " << describe(r);
        EXPECT_LE(r.slamWrongCells, 5) << name << ": " << describe(r);
        EXPECT_GT(r.odometryWrongCells, 3 * std::max(r.slamWrongCells, 3)) << name << ": " << describe(r);
    }
}

TEST_P(SlamRuns, ExploresToTheGoalWithNoMapUsingOnlyItsOwnEstimate) {
    for (const char* name : {"warehouse", "maze"}) {
        const Result r = run(name, GetParam(), true, 600);
        EXPECT_TRUE(r.arrived) << name << ": " << describe(r);
        EXPECT_EQ(r.bumps, 0) << name << ": " << describe(r);
        EXPECT_LT(r.worstError, 0.15) << name << ": " << describe(r);
    }
}

INSTANTIATE_TEST_SUITE_P(FiveSeeds, SlamRuns, ::testing::Range(1U, 6U));
