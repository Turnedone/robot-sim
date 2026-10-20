#pragma once

#include <optional>
#include <vector>

#include "occupancy_map.hpp"

// Extra cost on top of distance, so A* prefers safe, known routes.
struct PlannerConfig {
    double unknownPenalty = 0.5;   // unknown cell: +50% cost (passable: "optimistic planning")
    double nearWallPenalty = 2.0;  // cell touching a known wall: +200% cost (keeps away from walls)
};

// A* on the robot's own map, 8-connected (straight and diagonal moves).
// - Cells the map believes are walls are blocked; unknown cells are allowed.
// - A diagonal move is only allowed if both cells it squeezes between are known
//   to be free, so the path never cuts the corner of a wall, seen or unseen.
// Returns the cells from start to goal, or nothing if no path exists.
std::optional<std::vector<Pos>> planPath(const OccupancyMap& map, Pos start, Pos goal,
                                         const PlannerConfig& config = {});
