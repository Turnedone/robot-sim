#pragma once

#include <optional>
#include <vector>

#include "controller.hpp"
#include "occupancy_map.hpp"
#include "planner.hpp"
#include "robot.hpp"

enum class NavStatus : std::uint8_t { Idle, Driving, Arrived, NoPath };

struct NavigatorConfig {
    PlannerConfig planner;
    PursuitConfig pursuit;
    double robotRadius = DiffDriveConfig{}.radius;  // m; pass the real robot's radius
    double safetyMargin = 0.01;                     // m of extra clearance in the arc check
    double checkDistance = 0.45;                    // m of the arc ahead that must be clear
    double checkStep = 0.04;                        // m between collision checks along the arc
};

// Ties planning and control together, like a tiny version of ROS's Nav2:
// give it a goal; after every map update it re-plans with A* on the robot's
// own map; every control tick it asks pure pursuit for a velocity command.
class Navigator {
public:
    explicit Navigator(const NavigatorConfig& config = {});

    void setGoal(Pos goal);
    void cancel();

    // Call after the map changes (each scan). Re-plans from the robot's pose.
    void replan(const OccupancyMap& map, const Pose& pose);

    // Call every control tick. Returns the velocity to send to the robot.
    // Before driving, the arc the command would follow is checked against the
    // robot's own map; if it would hit a known wall, the robot turns on the
    // spot towards the path instead (a round robot can always do that safely).
    Twist command(const Pose& pose, const OccupancyMap& map);

    NavStatus status() const { return status_; }
    std::optional<Pos> goal() const { return goal_; }
    const std::vector<Point>& path() const { return pursuit_.path(); }
    std::optional<Point> lookaheadPoint() const { return pursuit_.lookaheadPoint(); }
    int replans() const { return replans_; }
    int collisionsAvoided() const { return collisionsAvoided_; }

private:
    NavigatorConfig config_;
    PurePursuit pursuit_;
    std::optional<Pos> goal_;
    NavStatus status_ = NavStatus::Idle;
    int replans_ = 0;
    int collisionsAvoided_ = 0;
};

// Collision check against the robot's OWN map: would a circle of `radius`,
// driving with `cmd` from `pose` for `distance` meters (checked every `step`
// meters), overlap a cell the map believes is a wall?
bool arcHitsWall(const OccupancyMap& map, const Pose& pose, const Twist& cmd, double distance, double radius,
                 double step);
