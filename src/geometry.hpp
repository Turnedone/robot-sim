#pragma once

#include <algorithm>
#include <cmath>

#include "grid.hpp"

inline constexpr double kPi = 3.14159265358979323846;

// Wraps any angle into [-pi, pi).
double normalizeAngle(double a);

// True if a circle of `radius` at `centre` overlaps a cell for which
// isWall(cell) is true. Works for any grid with this cell layout: the true
// world (Grid) and the robot's own map (OccupancyMap) both use it.
// Distance from point p to the nearest cell for which isWall(cell) is true,
// looking no further than maxDistance (returns maxDistance if none is closer).
// Exact distance to the cell's square, not to its centre.
template <typename IsWall>
double distanceToNearestWall(Point p, double maxDistance, int mapHeight, double resolution, IsWall isWall) {
    const Pos topLeft = worldToCell({p.x - maxDistance, p.y + maxDistance}, mapHeight, resolution);
    const Pos bottomRight = worldToCell({p.x + maxDistance, p.y - maxDistance}, mapHeight, resolution);
    double best2 = maxDistance * maxDistance;
    for (int row = topLeft.y; row <= bottomRight.y; ++row) {
        for (int col = topLeft.x; col <= bottomRight.x; ++col) {
            if (!isWall(Pos{col, row})) continue;
            const double minX = col * resolution;
            const double minY = (mapHeight - 1 - row) * resolution;
            const double dx = p.x - std::clamp(p.x, minX, minX + resolution);
            const double dy = p.y - std::clamp(p.y, minY, minY + resolution);
            best2 = std::min(best2, dx * dx + dy * dy);
        }
    }
    return std::sqrt(best2);
}

template <typename IsWall>
bool circleOverlapsWall(Point centre, double radius, int mapHeight, double resolution, IsWall isWall) {
    // Only the cells under the circle's bounding box can touch it.
    const Pos topLeft = worldToCell({centre.x - radius, centre.y + radius}, mapHeight, resolution);
    const Pos bottomRight = worldToCell({centre.x + radius, centre.y - radius}, mapHeight, resolution);

    for (int row = topLeft.y; row <= bottomRight.y; ++row) {
        for (int col = topLeft.x; col <= bottomRight.x; ++col) {
            if (!isWall(Pos{col, row})) continue;
            // The cell's square in meters, then the point of it closest to the centre.
            const double minX = col * resolution;
            const double minY = (mapHeight - 1 - row) * resolution;
            const double dx = centre.x - std::clamp(centre.x, minX, minX + resolution);
            const double dy = centre.y - std::clamp(centre.y, minY, minY + resolution);
            if (dx * dx + dy * dy < radius * radius) return true;
        }
    }
    return false;
}
