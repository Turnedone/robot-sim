#pragma once

#include <optional>
#include <vector>

#include "robot.hpp"

struct PursuitConfig {
    double lookahead = 0.35;          // m: how far ahead on the path to aim
    double maxSpeed = 0.4;            // m/s
    double curvatureSlowdown = 0.6;   // slow down on tight curves: v = max / (1 + k * |curvature|)
    double goalSlowdown = 1.0;        // near the goal or a corner: v <= k * distance to it
    double minSpeed = 0.05;           // m/s, so it never stalls just short of the goal
    double rotateInPlaceAbove = 1.0;  // rad (~57 deg): target this far off the nose -> turn on the spot first
    double rotateSpeed = 1.5;         // rad/s when turning on the spot
    double goalTolerance = 0.12;      // m: close enough counts as arrived
};

// Pure pursuit: aim at a point a fixed distance ahead on the path and drive
// the circular arc that passes through it. Simple, smooth, and still widely used.
//
// One change from the textbook version: the aim point never moves past the
// next CORNER of the path. Textbook pure pursuit starts turning before a corner
// and cuts it, which shifts the robot sideways; in a corridor only slightly
// wider than the robot that wedges it against a wall. Stopping the aim point
// at the corner makes the robot drive to the corner, turn, and carry on along
// the centre line.
class PurePursuit {
public:
    explicit PurePursuit(const PursuitConfig& config = {});

    // A new path (map frame). It is resampled to small, even steps.
    void setPath(const std::vector<Point>& path);
    void clear();

    // The velocity command for the robot's current pose.
    Twist compute(const Pose& pose);

    bool arrived() const { return arrived_; }
    const std::vector<Point>& path() const { return path_; }
    std::optional<Point> lookaheadPoint() const { return lookahead_; }
    const PursuitConfig& config() const { return config_; }

private:
    PursuitConfig config_;
    std::vector<Point> path_;
    std::vector<size_t> corners_;  // indices in path_ where the direction changes
    size_t progress_ = 0;          // index of the path point closest to the robot
    bool arrived_ = false;
    std::optional<Point> lookahead_;
};
