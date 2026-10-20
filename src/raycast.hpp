#pragma once

#include <cmath>
#include <limits>

#include "grid.hpp"

// Walks a ray through a grid of square cells with the DDA algorithm
// (Amanatides & Woo): it visits exactly the cells the ray passes through,
// in order, without skipping any and without visiting one twice.
//
// For every cell it calls visit(cell, enterDistance), where enterDistance is
// how far along the ray (in meters) the ray entered that cell (0 for the cell
// the ray starts in). Cells outside the map are visited too; the caller
// decides what they mean. Walking stops when visit returns false or the ray
// has gone further than maxDistance.
template <typename Visit>
void walkRay(int mapHeight, double resolution, Point origin, double angle, double maxDistance, Visit visit) {
    constexpr double kInf = std::numeric_limits<double>::infinity();

    // Work in cell units with rows counted from the BOTTOM, so x and y both
    // grow the same way as in the map frame.
    const double ox = origin.x / resolution;
    const double oy = origin.y / resolution;
    const double dx = std::cos(angle);
    const double dy = std::sin(angle);

    int col = static_cast<int>(std::floor(ox));
    int rowUp = static_cast<int>(std::floor(oy));
    const int stepX = dx > 0 ? 1 : -1;
    const int stepY = dy > 0 ? 1 : -1;

    // tDelta: how far along the ray (in cells) to cross one whole cell in x / y.
    // tMax:   how far along the ray the NEXT vertical / horizontal grid line is.
    const double tDeltaX = dx != 0 ? std::abs(1 / dx) : kInf;
    const double tDeltaY = dy != 0 ? std::abs(1 / dy) : kInf;
    double tMaxX = dx > 0 ? (col + 1 - ox) / dx : dx < 0 ? (ox - col) / -dx : kInf;
    double tMaxY = dy > 0 ? (rowUp + 1 - oy) / dy : dy < 0 ? (oy - rowUp) / -dy : kInf;

    const double maxT = maxDistance / resolution;
    double t = 0;
    while (t <= maxT) {
        if (!visit(Pos{col, mapHeight - 1 - rowUp}, t * resolution)) return;
        // Step into whichever neighbouring cell the ray reaches first.
        if (tMaxX < tMaxY) {
            t = tMaxX;
            tMaxX += tDeltaX;
            col += stepX;
        } else {
            t = tMaxY;
            tMaxY += tDeltaY;
            rowUp += stepY;
        }
    }
}
