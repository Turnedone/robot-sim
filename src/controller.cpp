#include "controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace {

constexpr double kSampleSpacing = 0.05;  // m between resampled path points

double dist(Point a, Point b) { return std::hypot(a.x - b.x, a.y - b.y); }

}  // namespace

PurePursuit::PurePursuit(const PursuitConfig& config) : config_(config) {}

void PurePursuit::clear() {
    path_.clear();
    corners_.clear();
    progress_ = 0;
    arrived_ = false;
    lookahead_.reset();
}

void PurePursuit::setPath(const std::vector<Point>& path) {
    clear();
    if (path.empty()) return;
    // Resample to evenly spaced points, so "the point 0.35 m ahead" is accurate,
    // and remember where the original path turns.
    path_.push_back(path.front());
    for (size_t i = 1; i < path.size(); ++i) {
        const Point a = path[i - 1];
        const Point b = path[i];
        if (i >= 2) {
            const Point before = path[i - 2];
            const double turn = std::remainder(std::atan2(b.y - a.y, b.x - a.x) -
                                                   std::atan2(a.y - before.y, a.x - before.x),
                                               2 * 3.14159265358979323846);
            if (std::abs(turn) > 0.1) corners_.push_back(path_.size() - 1);  // index of point a
        }
        const int pieces = std::max(1, static_cast<int>(std::ceil(dist(a, b) / kSampleSpacing)));
        for (int k = 1; k <= pieces; ++k) {
            const double t = static_cast<double>(k) / pieces;
            path_.push_back({a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t});
        }
    }
}

Twist PurePursuit::compute(const Pose& pose) {
    lookahead_.reset();
    if (path_.empty() || arrived_) return {};
    const Point here{pose.x, pose.y};

    // 1. Where along the path are we? Search a little way forward from last
    //    time, so the robot never jumps back to an earlier part of the path.
    const size_t searchEnd = std::min(path_.size(), progress_ + 40);
    double best = std::numeric_limits<double>::infinity();
    for (size_t i = progress_; i < searchEnd; ++i) {
        const double d = dist(path_[i], here);
        if (d < best) {
            best = d;
            progress_ = i;
        }
    }

    const double toGoal = dist(path_.back(), here);
    if (toGoal < config_.goalTolerance) {
        arrived_ = true;
        return {};
    }

    // 2. The lookahead point: the first path point at least `lookahead` away,
    //    but never past the next corner (one we have not reached yet).
    size_t limit = path_.size() - 1;
    for (size_t corner : corners_) {
        if (corner > progress_ + 1) {
            limit = corner;
            break;
        }
    }
    size_t aim = limit;
    for (size_t i = progress_; i < limit; ++i) {
        if (dist(path_[i], here) >= config_.lookahead) {
            aim = i;
            break;
        }
    }
    const Point target = path_[aim];
    lookahead_ = target;
    const bool aimingAtStop = aim == limit;  // a corner or the goal: slow down for it

    // 3. The target in the robot's own frame: x ahead, y to the left.
    const double dx = target.x - pose.x;
    const double dy = target.y - pose.y;
    const double ahead = std::cos(pose.theta) * dx + std::sin(pose.theta) * dy;
    const double left = -std::sin(pose.theta) * dx + std::cos(pose.theta) * dy;
    const double bearing = std::atan2(left, ahead);

    // Target far off to the side or behind: turn on the spot first.
    if (std::abs(bearing) > config_.rotateInPlaceAbove)
        return {0.0, bearing > 0 ? config_.rotateSpeed : -config_.rotateSpeed};

    // 4. The arc through the target: curvature = 2 * left / distance^2.
    const double d2 = ahead * ahead + left * left;
    const double curvature = 2.0 * left / d2;

    double v = config_.maxSpeed / (1.0 + config_.curvatureSlowdown * std::abs(curvature));
    v = std::min(v, config_.goalSlowdown * (aimingAtStop ? dist(target, here) : toGoal));
    v = std::max(v, config_.minSpeed);
    return {v, v * curvature};  // omega = v * curvature keeps the robot on that arc
}
