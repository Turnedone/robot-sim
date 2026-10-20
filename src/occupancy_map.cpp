#include "occupancy_map.hpp"

#include <algorithm>
#include <cmath>

#include "raycast.hpp"

double logOddsToProbability(double l) {
    return 1.0 - 1.0 / (1.0 + std::exp(l));
}

OccupancyMap knownMap(const Grid& world) {
    OccupancyMap map(world.width(), world.height(), world.resolution());
    for (int y = 0; y < world.height(); ++y)
        for (int x = 0; x < world.width(); ++x)
            map.set({x, y}, world.at({x, y}) == Cell::Wall ? Occupancy::Occupied : Occupancy::Free);
    return map;
}

OccupancyMap::OccupancyMap(int width, int height, double resolution, const OccupancyConfig& config)
    : width_(width),
      height_(height),
      resolution_(resolution),
      config_(config),
      logOdds_(cellCount(width, height), 0.0),
      observed_(cellCount(width, height), false),
      scanHits_(cellCount(width, height), 0),
      scanMisses_(cellCount(width, height), 0) {}

void OccupancyMap::update(Pos p, double delta) {
    const size_t i = index(p);
    logOdds_[i] = std::clamp(logOdds_[i] + delta, config_.minLogOdds, config_.maxLogOdds);
    observed_[i] = true;
}

void OccupancyMap::integrate(const LaserScan& scan, const Pose& pose) {
    const Point origin{pose.x, pose.y};

    // Near the robot many beams of the same scan cross the same cell. They are
    // one look at the world, not independent evidence, so first count what the
    // whole scan says about each cell, then update each cell ONCE, by majority.
    // A real wall is never passed through, so it wins easily; a free cell that
    // one stray beam ended in is outvoted by all the beams that crossed it.
    std::fill(scanHits_.begin(), scanHits_.end(), 0);
    std::fill(scanMisses_.begin(), scanMisses_.end(), 0);

    for (size_t i = 0; i < scan.ranges.size(); ++i) {
        const double angle = pose.theta + scan.angle(i);
        const double r = scan.ranges[i];

        // No return can mean "nothing within range" OR "the sensor missed a
        // surface right there" (a dropout). Letting it clear cells out to max
        // range would erase real walls behind the gap, so it does not vote.
        if (!std::isfinite(r)) continue;

        if (r > config_.usableRange) {
            walkRay(height_, resolution_, origin, angle, config_.usableRange, [&](Pos cell, double) {
                if (!inBounds(cell)) return false;
                ++scanMisses_[index(cell)];
                return true;
            });
            continue;
        }

        // The beam passed through every cell before the surface and ended in one.
        // With noise, the true surface is within [r - tol, r + tol]. Surfaces are
        // where the beam enters a new cell, so the wall is most likely the cell
        // whose entry point is CLOSEST to r. If no cell is entered in that window,
        // the wall is simply the cell the beam is in at distance r.
        const double tol = config_.surfaceTolerance;
        beamCells_.clear();
        walkRay(height_, resolution_, origin, angle, r + tol, [&](Pos cell, double enter) {
            beamCells_.push_back({cell, enter});
            return true;
        });

        size_t hit = 0;
        double bestGap = tol;
        for (size_t c = 1; c < beamCells_.size(); ++c) {
            if (!inBounds(beamCells_[c].cell)) break;  // beyond the map's edge is not part of the map
            const double gap = std::abs(beamCells_[c].enter - r);
            if (gap <= bestGap) {
                bestGap = gap;
                hit = c;
            }
        }
        if (bestGap == tol && hit == 0) {
            for (size_t c = 1; c < beamCells_.size() && beamCells_[c].enter <= r; ++c) hit = c;
        }

        for (size_t c = 0; c < hit; ++c) {
            if (!inBounds(beamCells_[c].cell)) break;
            ++scanMisses_[index(beamCells_[c].cell)];
        }
        if (inBounds(beamCells_[hit].cell)) ++scanHits_[index(beamCells_[hit].cell)];
    }

    for (size_t i = 0; i < logOdds_.size(); ++i) {
        if (scanHits_[i] == 0 && scanMisses_[i] == 0) continue;
        const Pos p{static_cast<int>(i) % width_, static_cast<int>(i) / width_};
        update(p, scanHits_[i] >= scanMisses_[i] ? config_.hit : config_.miss);
    }
}

void OccupancyMap::set(Pos p, Occupancy o) {
    if (!inBounds(p)) return;
    const size_t i = index(p);
    logOdds_[i] = o == Occupancy::Occupied ? config_.maxLogOdds : o == Occupancy::Free ? config_.minLogOdds : 0.0;
    observed_[i] = o != Occupancy::Unknown;
}

bool OccupancyMap::observed(Pos p) const {
    return inBounds(p) && observed_[index(p)];
}

double OccupancyMap::logOdds(Pos p) const {
    return inBounds(p) ? logOdds_[index(p)] : 0.0;
}

double OccupancyMap::probability(Pos p) const {
    return logOddsToProbability(logOdds(p));
}

Occupancy OccupancyMap::classify(Pos p) const {
    if (!observed(p)) return Occupancy::Unknown;
    const double l = logOdds(p);
    if (l > config_.occupiedAbove) return Occupancy::Occupied;
    if (l < config_.freeBelow) return Occupancy::Free;
    return Occupancy::Unknown;
}
