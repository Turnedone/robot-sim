#pragma once

#include <vector>

#include "grid.hpp"
#include "lidar.hpp"
#include "robot.hpp"

// How much one reading moves a cell's belief, in log-odds.
// log-odds l = log(p / (1 - p)): 0 = 50% (unknown), positive = probably a wall.
// Updating a belief with a new reading (Bayes' rule) then becomes plain addition.
struct OccupancyConfig {
    double hit = 0.85;          // a beam ended here:      p = 0.70
    double miss = -0.41;        // a beam passed through:  p = 0.40
    double minLogOdds = -2.0;   // caps (p = 0.12 .. 0.97), so a belief never gets so
    double maxLogOdds = 3.5;    //   strong that new evidence cannot change it
    double occupiedAbove = 0.62;  // classify: p > 0.65 -> occupied
    double freeBelow = -0.62;     //           p < 0.35 -> free
    // A reading says "the surface is within this distance of here". Without
    // it, noise would drop half the hits into the free cell just in front of
    // a wall, and walls would grow a cell thicker.
    double surfaceTolerance = 0.05;  // m, about 3x the lidar's noise
};

enum class Occupancy { Unknown, Free, Occupied };

double logOddsToProbability(double l);

// The robot's own map, built only from its lidar scans.
// It has the same size and cell layout as the true map, but starts empty.
class OccupancyMap {
public:
    OccupancyMap(int width, int height, double resolution, const OccupancyConfig& config = {});

    // Adds one scan, taken from `pose`, to the map. Each cell gets at most one
    // vote per scan (hit or miss, by majority of the beams that touched it).
    void integrate(const LaserScan& scan, const Pose& pose);

    int width() const { return width_; }
    int height() const { return height_; }
    double resolution() const { return resolution_; }

    bool observed(Pos p) const;        // has any beam ever touched this cell?
    double logOdds(Pos p) const;
    double probability(Pos p) const;
    Occupancy classify(Pos p) const;

private:
    void update(Pos p, double delta);
    size_t index(Pos p) const { return static_cast<size_t>(p.y * width_ + p.x); }
    bool inBounds(Pos p) const { return p.x >= 0 && p.y >= 0 && p.x < width_ && p.y < height_; }

    int width_;
    int height_;
    double resolution_;
    OccupancyConfig config_;
    std::vector<double> logOdds_;
    std::vector<bool> observed_;
    // Scratch space for one scan: how many beams ended in / passed through each cell.
    std::vector<unsigned short> scanHits_;
    std::vector<unsigned short> scanMisses_;

    struct BeamCell {
        Pos cell;
        double enter;  // distance along the beam where it entered the cell, m
    };
    std::vector<BeamCell> beamCells_;  // scratch: the cells one beam crosses
};
