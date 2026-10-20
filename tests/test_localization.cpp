// Odometry drift and Monte Carlo localization.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
#include <string>

#include "lidar.hpp"
#include "navigator.hpp"
#include "odometry.hpp"
#include "particle_filter.hpp"

namespace {

// Wheels 0.5% off in size (in opposite directions) plus 5% random slip per step.
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

Scenario load(const char* name) {
    return loadScenario(std::string(ROBOT_SIM_MAPS_DIR) + "/" + name + ".txt");
}

Pose startPose(const Scenario& s) {
    const Point c = s.grid.cellCenter(s.start);
    return {c.x, c.y, 0};
}

enum class Start : std::uint8_t { NearTruth, NoIdea };

struct RunOptions {
    Start start = Start::NearTruth;
    bool navigateWithEstimate = false;  // false: drive with the true pose (testing the filter alone)
    std::optional<double> kidnapAt;     // seconds
    Pose kidnapTo;
    bool patrol = false;  // keep driving between waypoints instead of stopping at the goal
    double timeLimit = 300;
};

struct RunResult {
    bool arrived = false;
    int bumps = 0;
    double seconds = 0;
    double finalError = 0;                 // filter estimate vs truth at the end, m
    double finalOdometryError = 0;         // odometry vs truth at the end, m
    double worstErrorAfterSettling = 0;    // filter error, ignoring the first 10 s and 10 s after a kidnap
    std::optional<double> convergedAfter;  // since when the error has stayed under 0.2 m
};

// The full loop: physics, odometry and control at 50 Hz; lidar, filter and
// re-planning at 5 Hz. The robot has a saved map of the world (no mapping).
RunResult run(const Scenario& s, unsigned seed, const RunOptions& o) {
    const DiffDriveConfig robotConfig = slippyRobot();
    DiffDriveRobot robot(robotConfig, startPose(s), seed);
    Odometry odometry(robotConfig.wheelBase, startPose(s));
    Lidar lidar(LidarConfig{}, seed + 100);
    const OccupancyMap map = knownMap(s.grid);

    ParticleFilter filter(MclConfig{}, seed + 200);
    if (o.start == Start::NearTruth)
        filter.initAround(startPose(s), 0.1, 0.1);
    else
        filter.initUniform(map);

    NavigatorConfig navConfig;
    navConfig.robotRadius = robotConfig.radius;
    Navigator nav(navConfig);
    nav.setGoal(s.goal);
    const auto navPose = [&] { return o.navigateWithEstimate ? filter.estimate() : robot.pose(); };
    // A robot acting on its estimate must not drive towards a goal while it is
    // lost: it turns on the spot instead, which gives the filter new views
    // (like Nav2's spin recovery).
    const auto command = [&] {
        if (o.navigateWithEstimate && !filter.healthy()) return Twist{0.0, 0.8};
        return nav.command(navPose(), map);
    };
    nav.replan(map, navPose());

    RunResult r;
    Pose lastOdometry = odometry.pose();
    double scanTimer = 0;
    double ignoreUntil = 10;
    bool touching = false;
    bool kidnapped = false;
    // A lost robot has to keep moving to find itself (a robot standing still
    // sees the same scan over and over), so the search scenarios patrol.
    const Pos waypoints[] = {{20, 3}, {1, 11}, {12, 1}, {28, 11}};
    int nextWaypoint = 0;
    while (r.seconds < o.timeLimit) {
        if (nav.status() == NavStatus::Arrived) {
            if (!o.patrol) break;
            nav.setGoal(waypoints[nextWaypoint++ % 4]);
            nav.replan(map, navPose());
        }
        if (o.kidnapAt && !kidnapped && r.seconds >= *o.kidnapAt) {
            robot.teleport(o.kidnapTo);  // nobody tells the robot
            kidnapped = true;
            ignoreUntil = r.seconds + 10;
        }
        robot.setCommand(command());
        const bool moved = robot.step(0.02, s.grid);
        if (!moved && !touching) ++r.bumps;
        touching = !moved;
        odometry.update(robot.leftSpeed(), robot.rightSpeed(), 0.02);
        r.seconds += 0.02;

        scanTimer += 0.02;
        if (scanTimer >= 0.2) {
            scanTimer -= 0.2;
            filter.predict(lastOdometry, odometry.pose());
            lastOdometry = odometry.pose();
            filter.correct(lidar.scan(s.grid, robot.pose()), map);
            nav.replan(map, navPose());

            const double err = distance(filter.estimate(), robot.pose());
            if (r.seconds > ignoreUntil) r.worstErrorAfterSettling = std::max(r.worstErrorAfterSettling, err);
            if (err < 0.2) {
                if (!r.convergedAfter) r.convergedAfter = r.seconds;
            } else {
                r.convergedAfter.reset();
            }
        }
    }
    r.arrived = nav.status() == NavStatus::Arrived || nextWaypoint > 0;
    r.finalError = distance(filter.estimate(), robot.pose());
    r.finalOdometryError = distance(odometry.pose(), robot.pose());
    return r;
}

std::string describe(const RunResult& r) {
    return "arrived=" + std::to_string(r.arrived) + " t=" + std::to_string(r.seconds) +
           " finalErr=" + std::to_string(r.finalError) + " odomErr=" + std::to_string(r.finalOdometryError) +
           " worst=" + std::to_string(r.worstErrorAfterSettling) + " bumps=" + std::to_string(r.bumps);
}

}  // namespace

TEST(Odometry, MatchesTheTruthWhenWheelsDoNotSlip) {
    const Grid floor(40, 40, 0.5);
    DiffDriveRobot robot(DiffDriveConfig{}, {10, 10, 0});
    Odometry odometry(DiffDriveConfig{}.wheelBase, robot.pose());
    robot.setCommand({0.3, 0.4});
    for (int i = 0; i < 1000; ++i) {
        robot.step(0.02, floor);
        odometry.update(robot.leftSpeed(), robot.rightSpeed(), 0.02);
    }
    EXPECT_NEAR(distance(odometry.pose(), robot.pose()), 0.0, 1e-9);
    EXPECT_NEAR(odometry.pose().theta, robot.pose().theta, 1e-9);
}

TEST(Odometry, DriftsWhenWheelsSlip) {
    const Grid floor(40, 40, 0.5);
    DiffDriveRobot robot(slippyRobot(), {10, 10, 0}, 3);
    Odometry odometry(slippyRobot().wheelBase, robot.pose());
    robot.setCommand({0.3, 0.2});
    for (int i = 0; i < 3000; ++i) {  // 60 s, about 18 m of driving in circles
        robot.step(0.02, floor);
        odometry.update(robot.leftSpeed(), robot.rightSpeed(), 0.02);
    }
    EXPECT_GT(distance(odometry.pose(), robot.pose()), 0.3) << "small wheel errors must add up to a large drift";
}

TEST(ParticleFilter, PredictWithoutNoiseMovesEveryParticleLikeOdometry) {
    MclConfig c;
    c.alpha1 = c.alpha2 = c.alpha3 = c.alpha4 = 0;
    ParticleFilter f(c);
    f.initAround({1, 1, 0}, 0, 0);
    f.predict({5, 5, 0}, {5.5, 5.5, kPi / 2});  // 0.71 m diagonally, ending facing up
    for (const Particle& p : f.particles()) {
        EXPECT_NEAR(p.pose.x, 1.5, 1e-9);
        EXPECT_NEAR(p.pose.y, 1.5, 1e-9);
        EXPECT_NEAR(p.pose.theta, kPi / 2, 1e-9);
    }
}

TEST(ParticleFilter, ScanMatchIsHighOnlyAtTheTruePose) {
    const Scenario s = load("warehouse");
    const OccupancyMap map = knownMap(s.grid);
    Lidar lidar(LidarConfig{}, 5);
    const Pose truth{3.25, 3.75, 0.3};
    const LaserScan scan = lidar.scan(s.grid, truth);
    const ParticleFilter f;
    EXPECT_GT(f.scanMatchAt(scan, map, truth), 0.9);
    // Near the map's edge, beams ending outside the map must not count as matches.
    EXPECT_LT(f.scanMatchAt(scan, map, {0.6, 0.6, -2.0}), 0.5);
}

TEST(ParticleFilter, ScanFitsBestAtTheTruePose) {
    const Scenario s = load("warehouse");
    const OccupancyMap map = knownMap(s.grid);
    Lidar lidar(LidarConfig{}, 5);
    const Pose truth{3.25, 3.75, 0.3};
    const LaserScan scan = lidar.scan(s.grid, truth);
    const ParticleFilter f;
    const double atTruth = f.scanLogLikelihood(scan, map, truth);
    EXPECT_GT(atTruth, f.scanLogLikelihood(scan, map, {truth.x + 0.2, truth.y, truth.theta}));
    EXPECT_GT(atTruth, f.scanLogLikelihood(scan, map, {truth.x, truth.y - 0.2, truth.theta}));
    EXPECT_GT(atTruth, f.scanLogLikelihood(scan, map, {truth.x, truth.y, truth.theta + 0.15}));
}

TEST(ParticleFilter, UsesTheLidarMountingOffset) {
    // The lidar sits 0.3 m ahead of the robot's centre. A filter that knows this
    // must prefer the true robot pose; one that ignores it is 0.3 m off.
    const Scenario s = load("warehouse");
    const OccupancyMap map = knownMap(s.grid);
    const Pose robot{3.25, 3.75, 0.0};
    const Pose lidarPose{robot.x + 0.3, robot.y, robot.theta};
    Lidar lidar(LidarConfig{}, 6);
    const LaserScan scan = lidar.scan(s.grid, lidarPose);
    MclConfig c;
    c.sensorOffset = {0.3, 0.0, 0.0};
    const ParticleFilter withOffset(c);
    const ParticleFilter withoutOffset;
    EXPECT_GT(withOffset.scanLogLikelihood(scan, map, robot), withOffset.scanLogLikelihood(scan, map, lidarPose));
    EXPECT_GT(withoutOffset.scanLogLikelihood(scan, map, lidarPose), withoutOffset.scanLogLikelihood(scan, map, robot));
}

class Localization : public ::testing::TestWithParam<unsigned> {};

TEST_P(Localization, TracksThePoseWhileOdometryDrifts) {
    const RunResult r = run(load("warehouse"), GetParam(), {});
    ASSERT_TRUE(r.arrived) << describe(r);
    EXPECT_LT(r.worstErrorAfterSettling, 0.15) << describe(r);
    EXPECT_LT(r.finalError, 0.15) << describe(r);
    EXPECT_GT(r.finalOdometryError, 3 * r.finalError) << "the filter should beat dead reckoning: " << describe(r);
}

TEST_P(Localization, FindsItselfWithNoStartingGuess) {
    RunOptions o;
    o.start = Start::NoIdea;
    o.navigateWithEstimate = true;
    o.patrol = true;
    o.timeLimit = 150;
    const RunResult r = run(load("warehouse"), GetParam(), o);
    ASSERT_TRUE(r.convergedAfter) << "never found itself: " << describe(r);
    EXPECT_LT(*r.convergedAfter, 90) << describe(r);
    EXPECT_LT(r.finalError, 0.2) << describe(r);
}

TEST_P(Localization, RecoversAfterBeingKidnapped) {
    const Scenario s = load("warehouse");
    RunOptions o;
    o.kidnapAt = 15;
    const Point elsewhere = s.grid.cellCenter({15, 5});
    o.kidnapTo = {elsewhere.x, elsewhere.y, kPi / 2};
    o.navigateWithEstimate = true;
    o.patrol = true;
    o.timeLimit = 150;
    const RunResult r = run(s, GetParam(), o);
    ASSERT_TRUE(r.convergedAfter) << "never recovered: " << describe(r);
    EXPECT_LT(*r.convergedAfter - 15, 90) << "recovery took too long: " << describe(r);
    EXPECT_LT(r.finalError, 0.2) << describe(r);
}

TEST_P(Localization, NavigatesUsingOnlyItsEstimate) {
    RunOptions o;
    o.navigateWithEstimate = true;
    for (const char* name : {"warehouse", "maze"}) {
        const RunResult r = run(load(name), GetParam(), o);
        EXPECT_TRUE(r.arrived) << name << ": " << describe(r);
        EXPECT_EQ(r.bumps, 0) << name << ": " << describe(r);
        EXPECT_LT(r.finalError, 0.15) << name << ": " << describe(r);
    }
}

INSTANTIATE_TEST_SUITE_P(TenSeeds, Localization, ::testing::Range(1U, 11U));
