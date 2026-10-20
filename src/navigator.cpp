#include "navigator.hpp"

#include <algorithm>
#include <cmath>

bool arcHitsWall(const OccupancyMap& map, const Pose& pose, const Twist& cmd, double distance, double radius,
                 double step) {
    // A cell counts as wall if the map believes it is one; outside the map too.
    const auto isWall = [&map](Pos cell) { return !map.inBounds(cell) || map.classify(cell) == Occupancy::Occupied; };
    if (std::abs(cmd.linear) < 1e-9) return false;  // turning on the spot never moves the circle
    const double curvature = cmd.angular / cmd.linear;
    const double dir = cmd.linear > 0 ? 1.0 : -1.0;
    const int steps = static_cast<int>(std::floor(distance / step + 1e-9));
    for (int i = 1; i <= steps; ++i) {
        const double s = i * step;
        // Position after driving s meters along the arc (same maths as the robot's step()).
        Point p;
        if (std::abs(curvature) < 1e-9) {
            p = {pose.x + dir * s * std::cos(pose.theta), pose.y + dir * s * std::sin(pose.theta)};
        } else {
            const double theta = pose.theta + dir * s * curvature;
            p = {pose.x + (std::sin(theta) - std::sin(pose.theta)) / curvature,
                 pose.y - (std::cos(theta) - std::cos(pose.theta)) / curvature};
        }
        if (circleOverlapsWall(p, radius, map.height(), map.resolution(), isWall)) return true;
    }
    return false;
}

Navigator::Navigator(const NavigatorConfig& config) : config_(config), pursuit_(config.pursuit) {}

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
    const std::optional<std::vector<Pos>> cells = planPath(map, start, *goal_, config_.planner);
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
    const double ahead = std::min(config_.checkDistance, std::max(config_.checkStep, std::abs(cmd.linear) * 1.0));
    const std::optional<Point> aim = pursuit_.lookaheadPoint();
    if (aim && arcHitsWall(map, pose, cmd, ahead, config_.robotRadius + config_.safetyMargin, config_.checkStep)) {
        ++collisionsAvoided_;
        // Turn on the spot towards the lookahead point; driving resumes once the
        // arc from the new heading is clear.
        const Point target = *aim;
        const double bearing = std::remainder(std::atan2(target.y - pose.y, target.x - pose.x) - pose.theta, 2 * kPi);
        cmd = {0.0, bearing >= 0 ? pursuit_.config().rotateSpeed : -pursuit_.config().rotateSpeed};
    }
    return cmd;
}
