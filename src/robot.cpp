#include "robot.hpp"

#include <algorithm>
#include <cmath>

namespace {

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

bool collides(const Grid& grid, Point centre, double radius) {
    // Outside the map counts as wall (Grid::at returns Wall there).
    return circleOverlapsWall(centre, radius, grid.height(), grid.resolution(),
                              [&grid](Pos cell) { return grid.at(cell) == Cell::Wall; });
}

Pose integrateArc(const Pose& pose, double v, double w, double dt) {
    Pose next = pose;
    if (std::abs(w) < 1e-9) {
        // Driving straight.
        next.x += v * std::cos(pose.theta) * dt;
        next.y += v * std::sin(pose.theta) * dt;
    } else {
        // Driving along an arc of radius v / w. Integrating the arc exactly
        // (instead of "move straight, then turn") keeps the error at zero
        // however large dt is.
        const double newTheta = pose.theta + w * dt;
        next.x += v / w * (std::sin(newTheta) - std::sin(pose.theta));
        next.y -= v / w * (std::cos(newTheta) - std::cos(pose.theta));
        next.theta = newTheta;
    }
    next.theta = normalizeAngle(next.theta);
    return next;
}

DiffDriveRobot::DiffDriveRobot(const DiffDriveConfig& config, Pose start, unsigned seed)
    : config_(config), pose_(start), rng_(seed) {}

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

    // How the wheels really move the ground: the motor speeds, scaled by the
    // true wheel sizes, plus random slip.
    double groundLeft = left_ * config_.leftWheelScale;
    double groundRight = right_ * config_.rightWheelScale;
    if (config_.slipNoise > 0) {
        std::normal_distribution<double> slip(0.0, config_.slipNoise);
        groundLeft *= 1.0 + slip(rng_);
        groundRight *= 1.0 + slip(rng_);
    }

    // Forward kinematics: wheel speeds -> how the body moves.
    const double v = (groundLeft + groundRight) / 2;
    const double w = (groundRight - groundLeft) / config_.wheelBase;
    const Pose next = integrateArc(pose_, v, w, dt);

    if (collides(grid, {next.x, next.y}, config_.radius)) {
        left_ = 0;
        right_ = 0;
        return false;
    }
    distance_ += std::abs(v) * dt;
    pose_ = next;
    return true;
}
