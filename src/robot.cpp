#include "robot.hpp"

#include <algorithm>
#include <cmath>

namespace {

constexpr double kPi = 3.14159265358979323846;

// Moves `current` towards `target` by at most `maxChange`.
double approach(double current, double target, double maxChange) {
    if (target > current) return std::min(target, current + maxChange);
    return std::max(target, current - maxChange);
}

}  // namespace

double normalizeAngle(double a) {
    a = std::fmod(a + kPi, 2 * kPi);
    if (a < 0) a += 2 * kPi;
    return a - kPi;
}

bool collides(const Grid& grid, Point c, double r) {
    const double res = grid.resolution();
    // Only the cells under the circle's bounding box can touch it.
    const Pos topLeft = grid.worldToCell({c.x - r, c.y + r});
    const Pos bottomRight = grid.worldToCell({c.x + r, c.y - r});

    for (int row = topLeft.y; row <= bottomRight.y; ++row) {
        for (int col = topLeft.x; col <= bottomRight.x; ++col) {
            if (grid.at({col, row}) != Cell::Wall) continue;
            // The cell's square in meters, then the point of it closest to the centre.
            const double minX = col * res;
            const double minY = (grid.height() - 1 - row) * res;
            const double nearestX = std::clamp(c.x, minX, minX + res);
            const double nearestY = std::clamp(c.y, minY, minY + res);
            const double dx = c.x - nearestX;
            const double dy = c.y - nearestY;
            if (dx * dx + dy * dy < r * r) return true;
        }
    }
    return false;
}

DiffDriveRobot::DiffDriveRobot(const DiffDriveConfig& config, Pose start)
    : config_(config), pose_(start) {}

void DiffDriveRobot::setCommand(Twist cmd) {
    // Inverse kinematics: the wheel speeds that give this v and omega.
    const double halfBase = config_.wheelBase / 2;
    double left = cmd.linear - cmd.angular * halfBase;
    double right = cmd.linear + cmd.angular * halfBase;

    // A wheel asked to go faster than it can: slow BOTH by the same factor,
    // so the robot still follows the same curve, only more slowly.
    const double fastest = std::max(std::abs(left), std::abs(right));
    if (fastest > config_.maxWheelSpeed) {
        const double scale = config_.maxWheelSpeed / fastest;
        left *= scale;
        right *= scale;
    }
    targetLeft_ = left;
    targetRight_ = right;
}

Twist DiffDriveRobot::velocity() const {
    return {(left_ + right_) / 2, (right_ - left_) / config_.wheelBase};
}

bool DiffDriveRobot::step(double dt, const Grid& grid) {
    // Motors ramp up and down; they cannot jump to a new speed.
    const double maxChange = config_.maxWheelAccel * dt;
    left_ = approach(left_, targetLeft_, maxChange);
    right_ = approach(right_, targetRight_, maxChange);

    // Forward kinematics: wheel speeds -> how the body moves.
    const Twist vel = velocity();
    const double v = vel.linear;
    const double w = vel.angular;

    Pose next = pose_;
    if (std::abs(w) < 1e-9) {
        // Driving straight.
        next.x += v * std::cos(pose_.theta) * dt;
        next.y += v * std::sin(pose_.theta) * dt;
    } else {
        // Driving along an arc of radius v / w. Integrating the arc exactly
        // (instead of "move straight, then turn") keeps the error at zero
        // however large dt is.
        const double newTheta = pose_.theta + w * dt;
        next.x += v / w * (std::sin(newTheta) - std::sin(pose_.theta));
        next.y -= v / w * (std::cos(newTheta) - std::cos(pose_.theta));
        next.theta = newTheta;
    }
    next.theta = normalizeAngle(next.theta);

    if (collides(grid, {next.x, next.y}, config_.radius)) {
        left_ = 0;
        right_ = 0;
        return false;
    }
    distance_ += std::abs(v) * dt;
    pose_ = next;
    return true;
}
