// Robot motion, checked against answers worked out by hand.

#include <gtest/gtest.h>

#include <cmath>

#include "robot.hpp"

namespace {

constexpr double kEps = 1e-6;

// Wheels reach their target speed at once, so the maths is easy to check.
DiffDriveConfig instantWheels() {
    DiffDriveConfig c;
    c.maxWheelAccel = 1e9;
    return c;
}

// 20 m x 20 m of open floor, so walls never get in the way.
const Grid& openFloor() {
    static const Grid floor(40, 40, 0.5);
    return floor;
}

void run(DiffDriveRobot& robot, double seconds) {
    const double dt = 0.02;
    const long steps = std::lround(seconds / dt);
    for (long i = 0; i < steps; ++i) robot.step(dt, openFloor());
}

}  // namespace

TEST(DiffDrive, StraightLineMovesSpeedTimesTime) {
    DiffDriveRobot r(instantWheels(), {10, 10, 0});
    r.setCommand({0.5, 0});
    run(r, 2.0);
    EXPECT_NEAR(r.pose().x, 11.0, kEps);
    EXPECT_NEAR(r.pose().y, 10.0, kEps);
    EXPECT_NEAR(r.pose().theta, 0.0, kEps);
}

TEST(DiffDrive, SpinInPlaceTurnsWithoutMoving) {
    DiffDriveRobot r(instantWheels(), {10, 10, 0});
    r.setCommand({0, 1.0});
    run(r, 1.0);
    EXPECT_NEAR(r.pose().x, 10.0, kEps);
    EXPECT_NEAR(r.pose().y, 10.0, kEps);
    EXPECT_NEAR(r.pose().theta, 1.0, kEps);
    EXPECT_NEAR(r.leftSpeed(), -r.rightSpeed(), kEps) << "wheels must turn in opposite directions";
}

TEST(DiffDrive, ForwardTurnLeftForward) {
    DiffDriveRobot r(instantWheels(), {10, 10, 0});
    r.setCommand({0.5, 0});
    run(r, 1.0);
    r.setCommand({0, kPi / 2});  // a quarter turn left takes 1 s
    run(r, 1.0);
    r.setCommand({0.5, 0});
    run(r, 1.0);
    EXPECT_NEAR(r.pose().x, 10.5, kEps);
    EXPECT_NEAR(r.pose().y, 10.5, kEps);
    EXPECT_NEAR(r.pose().theta, kPi / 2, kEps);
}

TEST(DiffDrive, FullCircleReturnsToStartAndOdometerMatches) {
    // One lap at omega = 2*pi / 12.5 takes exactly 12.5 s (625 steps).
    DiffDriveRobot r(instantWheels(), {10, 10, 0});
    r.setCommand({0.3, 2 * kPi / 12.5});
    run(r, 12.5);
    EXPECT_NEAR(r.pose().x, 10.0, kEps);
    EXPECT_NEAR(r.pose().y, 10.0, kEps);
    EXPECT_NEAR(r.distanceTravelled(), 0.3 * 12.5, kEps);
}

TEST(DiffDrive, QuarterCircleLandsExactlyOnTheArc) {
    // A full lap closes even with naive "straight then turn" steps, so it cannot tell
    // the methods apart. A quarter lap can: the exact arc lands precisely on (R, R)
    // from the start; the naive method is about v * dt = 6 mm off.
    const double omega = 2 * kPi / 12.8;  // quarter lap = 3.2 s = 160 steps
    const double radius = 0.3 / omega;
    DiffDriveRobot r(instantWheels(), {10, 10, 0});
    r.setCommand({0.3, omega});
    run(r, 3.2);
    EXPECT_NEAR(r.pose().x, 10 + radius, kEps);
    EXPECT_NEAR(r.pose().y, 10 + radius, kEps);
    EXPECT_NEAR(r.pose().theta, kPi / 2, kEps);
}

TEST(DiffDrive, SpeedLimitKeepsTheCurve) {
    DiffDriveRobot r(instantWheels(), {10, 10, 0});
    r.setCommand({1.2, 2.0});  // too fast: the wheels would need 0.9 and 1.5 m/s
    r.step(0.02, openFloor());
    const Twist v = r.velocity();
    EXPECT_NEAR(r.rightSpeed(), 0.6, kEps) << "the fastest wheel is capped at 0.6 m/s";
    EXPECT_NEAR(v.angular / v.linear, 2.0 / 1.2, kEps) << "curvature must be preserved";
}

TEST(DiffDrive, WheelsAccelerateGradually) {
    DiffDriveRobot r(DiffDriveConfig{}, {10, 10, 0});
    r.setCommand({0.6, 0});
    r.step(0.1, openFloor());
    EXPECT_NEAR(r.velocity().linear, 0.12, kEps) << "0.1 s at 1.2 m/s^2";
}

TEST(DiffDrive, BumperStopsTheRobotAtTheEdge) {
    // Drive left towards the map edge (x = 0); with radius 0.18 m it must stop just short of it.
    DiffDriveRobot r(instantWheels(), {1.0, 2.5, kPi});
    r.setCommand({0.5, 0});
    bool bumped = false;
    for (int i = 0; i < 200; ++i) bumped |= !r.step(0.02, openFloor());
    EXPECT_TRUE(bumped);
    EXPECT_GT(r.pose().x, 0.18);
    EXPECT_LT(r.pose().x, 0.2);
}

TEST(Geometry, AnglesWrapIntoMinusPiToPi) {
    EXPECT_NEAR(normalizeAngle(1.5 * kPi), -kPi / 2, kEps);
    EXPECT_NEAR(normalizeAngle(-kPi / 2), -kPi / 2, kEps);
    EXPECT_NEAR(normalizeAngle(2 * kPi + 0.25), 0.25, kEps);
}

TEST(Geometry, MapFrameHasYUp) {
    const Grid g(30, 13, 0.5);
    const Point c = g.cellCenter({1, 1});
    EXPECT_NEAR(c.x, 0.75, kEps);
    EXPECT_NEAR(c.y, 5.75, kEps) << "row 1 is near the TOP of the map";
    EXPECT_EQ(g.worldToCell(c), (Pos{1, 1}));
}
