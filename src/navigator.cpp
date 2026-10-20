#include "navigator.hpp"

#include <algorithm>
#include <cmath>

namespace {

constexpr double kCheckStep = 0.04;       // m between collision checks along the arc
constexpr double kCheckDistance = 0.45;   // m of the arc ahead that must be clear
constexpr double kRobotRadius = 0.18;     // m (same as DiffDriveConfig)
constexpr double kSafetyMargin = 0.01;    // m extra clearance

bool circleHitsWall(const OccupancyMap& map, Point c, double r) {
    const double res = map.resolution();
    const Pos topLeft = map.worldToCell({c.x - r, c.y + r});
    const Pos bottomRight = map.worldToCell({c.x + r, c.y - r});
    for (int row = topLeft.y; row <= bottomRight.y; ++row) {
        for (int col = topLeft.x; col <= bottomRight.x; ++col) {
            if (map.inBounds({col, row}) && map.classify({col, row}) != Occupancy::Occupied) continue;
            const double minX = col * res;
            const double minY = (map.height() - 1 - row) * res;
            const double dx = c.x - std::clamp(c.x, minX, minX + res);
            const double dy = c.y - std::clamp(c.y, minY, minY + res);
            if (dx * dx + dy * dy < r * r) return true;
        }
    }
    return false;
}

}  // namespace

bool arcHitsWall(const OccupancyMap& map, const Pose& pose, const Twist& cmd, double distance, double radius) {
    if (std::abs(cmd.linear) < 1e-9) return false;  // turning on the spot never moves the circle
    const double curvature = cmd.angular / cmd.linear;
    const double dir = cmd.linear > 0 ? 1.0 : -1.0;
    for (double s = kCheckStep; s <= distance + 1e-9; s += kCheckStep) {
        // Position after driving s meters along the arc (same maths as the robot's step()).
        Point p;
        if (std::abs(curvature) < 1e-9) {
            p = {pose.x + dir * s * std::cos(pose.theta), pose.y + dir * s * std::sin(pose.theta)};
        } else {
            const double theta = pose.theta + dir * s * curvature;
            p = {pose.x + (std::sin(theta) - std::sin(pose.theta)) / curvature,
                 pose.y - (std::cos(theta) - std::cos(pose.theta)) / curvature};
        }
        if (circleHitsWall(map, p, radius)) return true;
    }
    return false;
}

Navigator::Navigator(const PlannerConfig& planner, const PursuitConfig& pursuit)
    : plannerConfig_(planner), pursuit_(pursuit) {}

void Navigator::setGoal(Pos goal) {
    goal_ = goal;
    status_ = NavStatus::Driving;
    pursuit_.clear();
}

void Navigator::cancel() {
    goal_.reset();
    status_ = NavStatus::Idle;
    pursuit_.clear();
}

void Navigator::replan(const OccupancyMap& map, const Pose& pose) {
    if (!goal_ || status_ == NavStatus::Idle || status_ == NavStatus::Arrived) return;

    const Pos start = map.worldToCell({pose.x, pose.y});
    const std::optional<std::vector<Pos>> cells = planPath(map, start, *goal_, plannerConfig_);
    ++replans_;
    if (!cells) {
        // Maybe the map is wrong or not explored enough yet: keep the goal and
        // try again after the next scan, but do not drive meanwhile.
        status_ = NavStatus::NoPath;
        pursuit_.clear();
        return;
    }

    // The path starts where the robot actually is, then follows cell centres.
    // The centre of the robot's OWN cell is kept unless the robot has already
    // passed it: skipping it at a turn would make the path cut diagonally
    // across the corner, which in a narrow corridor wedges the robot.
    std::vector<Point> points{{pose.x, pose.y}};
    if (cells->size() >= 2) {
        const Point here = map.cellCenter((*cells)[0]);
        const Point next = map.cellCenter((*cells)[1]);
        const double len = std::hypot(next.x - here.x, next.y - here.y);
        const double ahead = ((here.x - pose.x) * (next.x - here.x) + (here.y - pose.y) * (next.y - here.y)) / len;
        if (ahead > -0.02) points.push_back(here);
    }
    for (size_t i = 1; i < cells->size(); ++i) points.push_back(map.cellCenter((*cells)[i]));
    pursuit_.setPath(points);
    status_ = NavStatus::Driving;
}

Twist Navigator::command(const Pose& pose, const OccupancyMap& map) {
    if (status_ != NavStatus::Driving) return {};
    Twist cmd = pursuit_.compute(pose);
    if (pursuit_.arrived()) {
        status_ = NavStatus::Arrived;
        return {};
    }

    // Safety check: would this arc run into a wall the robot already knows about?
    const double ahead = std::min(kCheckDistance, std::max(kCheckStep, std::abs(cmd.linear) * 1.0));
    if (arcHitsWall(map, pose, cmd, ahead, kRobotRadius + kSafetyMargin) && pursuit_.lookaheadPoint()) {
        ++collisionsAvoided_;
        // Turn on the spot towards the lookahead point; driving resumes once the
        // arc from the new heading is clear.
        const Point target = *pursuit_.lookaheadPoint();
        const double bearing = std::remainder(std::atan2(target.y - pose.y, target.x - pose.x) - pose.theta,
                                              2 * 3.14159265358979323846);
        cmd = {0.0, bearing >= 0 ? pursuit_.config().rotateSpeed : -pursuit_.config().rotateSpeed};
    }
    return cmd;
}
