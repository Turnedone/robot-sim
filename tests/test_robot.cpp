// Checks the robot's motion against answers worked out by hand.
// Run with: ctest --test-dir build   (or .\build\test_robot.exe)

#include <cmath>
#include <cstdio>

#include "robot.hpp"

namespace {

int failures = 0;

void check(bool ok, const char* what) {
    std::printf("%s  %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) ++failures;
}

bool near(double a, double b, double tolerance = 1e-6) { return std::abs(a - b) < tolerance; }

// Wheels reach their target speed at once, so the maths is easy to check.
DiffDriveConfig instantWheels() {
    DiffDriveConfig c;
    c.maxWheelAccel = 1e9;
    return c;
}

// 20 m x 20 m of open floor, so walls never get in the way.
Grid openFloor() { return Grid(40, 40, 0.5); }

void run(DiffDriveRobot& robot, const Grid& grid, double seconds) {
    const double dt = 0.02;
    const long steps = std::lround(seconds / dt);
    for (long i = 0; i < steps; ++i) robot.step(dt, grid);
}

}  // namespace

int main() {
    const Grid world = openFloor();
    const double pi = std::acos(-1.0);

    {
        DiffDriveRobot r(instantWheels(), {10, 10, 0});
        r.setCommand({0.5, 0});
        run(r, world, 2.0);
        check(near(r.pose().x, 11.0) && near(r.pose().y, 10.0) && near(r.pose().theta, 0.0),
              "straight: 0.5 m/s for 2 s moves exactly 1 m forward");
    }
    {
        DiffDriveRobot r(instantWheels(), {10, 10, 0});
        r.setCommand({0, 1.0});
        run(r, world, 1.0);
        check(near(r.pose().x, 10.0) && near(r.pose().y, 10.0) && near(r.pose().theta, 1.0),
              "spin in place: 1 rad/s for 1 s turns 1 rad without moving");
        check(near(r.leftSpeed(), -r.rightSpeed()), "spin in place: wheels turn in opposite directions");
    }
    {
        DiffDriveRobot r(instantWheels(), {10, 10, 0});
        r.setCommand({0.5, 0});
        run(r, world, 1.0);
        r.setCommand({0, pi / 2});  // a quarter turn left takes 1 s
        run(r, world, 1.0);
        r.setCommand({0.5, 0});
        run(r, world, 1.0);
        check(near(r.pose().x, 10.5) && near(r.pose().y, 10.5) && near(r.pose().theta, pi / 2),
              "forward, turn left, forward: ends up up-and-to-the-right, facing +y");
    }
    {
        // One full circle at omega = 2*pi / 12.5 takes exactly 12.5 s (625 steps).
        DiffDriveRobot r(instantWheels(), {10, 10, 0});
        r.setCommand({0.3, 2 * pi / 12.5});
        run(r, world, 12.5);
        check(near(r.pose().x, 10.0) && near(r.pose().y, 10.0),
              "circle: one full lap returns to the starting point");
        check(near(r.distanceTravelled(), 0.3 * 12.5), "circle: odometer equals speed x time");
    }
    {
        DiffDriveRobot r(instantWheels(), {10, 10, 0});
        r.setCommand({1.2, 2.0});  // too fast: wheels would need 0.9 and 1.5 m/s
        r.step(0.02, world);
        const Twist v = r.velocity();
        check(near(r.rightSpeed(), 0.6), "speed limit: the fastest wheel is capped at 0.6 m/s");
        check(near(v.angular / v.linear, 2.0 / 1.2), "speed limit: the curve is kept, only slower");
    }
    {
        DiffDriveRobot r(DiffDriveConfig{}, {10, 10, 0});
        r.setCommand({0.6, 0});
        r.step(0.1, world);
        check(near(r.velocity().linear, 0.12), "acceleration: after 0.1 s at 1.2 m/s^2 the speed is 0.12 m/s");
    }
    {
        // Drive left towards the map edge (x = 0). Radius 0.18 m, so it must stop just short of it.
        DiffDriveRobot r(instantWheels(), {1.0, 2.5, pi});
        r.setCommand({0.5, 0});
        bool bumped = false;
        for (int i = 0; i < 200; ++i) bumped |= !r.step(0.02, world);
        check(bumped && r.pose().x > 0.18 && r.pose().x < 0.2, "wall: the bumper stops the robot at the edge");
    }
    {
        check(near(normalizeAngle(1.5 * pi), -pi / 2) && near(normalizeAngle(-pi / 2), -pi / 2) &&
                  near(normalizeAngle(2 * pi + 0.25), 0.25),
              "angles wrap into [-pi, pi)");
    }
    {
        const Grid g(30, 13, 0.5);
        const Point c = g.cellCenter({1, 1});
        check(near(c.x, 0.75) && near(c.y, 5.75), "map frame: cell (1,1) is near the TOP-left, y grows up");
        check(g.worldToCell(c) == Pos{1, 1}, "map frame: meters -> cell -> meters round-trips");
    }

    std::printf("\n%s (%d failed)\n", failures == 0 ? "ALL PASSED" : "SOME FAILED", failures);
    return failures == 0 ? 0 : 1;
}
