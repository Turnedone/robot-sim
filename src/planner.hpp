#pragma once

#include <optional>
#include <vector>

#include "occupancy_map.hpp"

// Extra cost on top of distance, so A* prefers safe, known routes.
struct PlannerConfig {
    double unknownPenalty = 0.5;   // unknown cell: +50% cost (passable: "optimistic planning")
    double nearWallPenalty = 2.0;  // cell touching a known wall: +200% cost (keeps away from walls)

    // Recovery mode (Nav2: "clear costmap"): treat WEAK walls, ones the map
    // has seen only a few times (log-odds below strongWall), as passable at a
    // high extra cost. Real walls are seen over and over and saturate at the
    // log-odds cap; a false wall from a few grazing beams does not.
    bool passWeakWalls = false;
    double strongWall = 3.0;
    double weakWallPenalty = 20.0;

    // For maps with cells much smaller than the robot (e.g. ROS maps, 5 cm):
    // inflation, as in Nav2's inflation layer. Cells whose centre is closer
    // than inflationRadius to a wall are blocked (the robot would touch it);
    // cells within clearanceRange beyond that cost up to clearancePenalty
    // extra, fading with distance, so paths keep to the middle of passages.
    // 0 = off (on 0.5 m maps the nearWallPenalty above does this job).
    double inflationRadius = 0.0;  // m
    double clearanceRange = 0.0;   // m
    double clearancePenalty = 3.0;
    // Straighten the path afterwards ("string pulling"): skip cells as long as
    // the straight line between the kept ones only crosses passable cells.
    // On fine grids this turns a staircase of tiny steps into a few segments.
    bool smoothPath = false;
};

// A* on the robot's own map, 8-connected (straight and diagonal moves).
// - Cells the map believes are walls are blocked; unknown cells are allowed.
// - A diagonal move is only allowed if both cells it squeezes between are known
//   to be free, so the path never cuts the corner of a wall, seen or unseen.
// Returns the cells from start to goal (only the corners when smoothPath is
// set), or nothing if no path exists.
std::optional<std::vector<Pos>> planPath(const OccupancyMap& map, Pos start, Pos goal,
                                         const PlannerConfig& config = {});
