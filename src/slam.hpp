#pragma once

#include <random>
#include <vector>

#include "lidar.hpp"
#include "occupancy_map.hpp"
#include "robot.hpp"

// SLAM: building a map and localizing in it at the same time, starting with
// no map and slipping wheels.
//
// A grid-based Rao-Blackwellized particle filter, the idea behind GMapping
// (Grisetti, Stachniss, Burgard 2007): every particle is a full hypothesis,
// a pose AND the map built as if that particle's path were true. Before a scan
// is added, each particle improves its pose by scan matching against its own
// map, which corrects most of the odometry drift; particles whose maps keep
// fitting the scans gain weight, and resampling keeps the good histories,
// maps included.
struct SlamConfig {
    int particles = 30;

    // Odometry motion model noise, calibrated to this robot's odometry (a few
    // percent). Much more noise made the particles random-walk along
    // featureless corridors. These are VARIANCES: alpha3 = 0.002 is a 4.5%
    // standard deviation of the distance driven.
    double alpha1 = 0.01;   // rotation error from rotation
    double alpha2 = 0.003;  // rotation error from translation
    double alpha3 = 0.002;  // translation error from translation
    double alpha4 = 0.001;  // translation error from rotation

    // Scan matching: hill climbing on the scan's fit to the particle's map.
    int beamStep = 5;                // 36 of 180 beams
    double hitSigma = 0.08;          // m
    double maxWallDistance = 0.5;    // m: search radius for the nearest wall
    double matchMaxRange = 3.0;      // m: ignore farther beams when matching
    double initialStepXY = 0.1;      // m
    double initialStepTheta = 0.05;  // rad
    int maxIterations = 30;
    // Odometry prior: moving the pose away from where odometry put it costs
    // (distance / sigma)^2 / 2. Where the scan carries no information (along a
    // featureless corridor) odometry decides; elsewhere the scan wins easily.
    double priorSigmaXY = 0.03;     // m (0 = no prior)
    double priorSigmaTheta = 0.03;  // rad

    // Only update after moving (as in MclConfig), and resample only when the
    // weights have become uneven: every resample throws away map histories.
    double updateMinDistance = 0.1;  // m
    double updateMinAngle = 0.15;    // rad
    double resampleBelowEss = 0.5;   // share of particles
};

class GridSlam {
public:
    // The map has a known size and the robot a known start pose (the map is
    // built relative to where the robot starts).
    GridSlam(int width, int height, double resolution, const Pose& start, const SlamConfig& config = {},
             unsigned seed = 11);

    // Odometry change since the last call: move every particle, with noise.
    void predict(const Pose& odomBefore, const Pose& odomAfter);
    // Scan matching, weighting, adding the scan to each particle's map, and
    // resampling when needed. Skipped (returns false) until the robot has moved.
    bool update(const LaserScan& scan);

    // The current best particle: its pose and its map.
    const Pose& estimate() const { return particles_[best_].pose; }
    const OccupancyMap& map() const { return particles_[best_].map; }
    std::vector<Pose> poses() const;  // every particle's pose, for drawing
    int resamples() const { return resamples_; }

    // How well a scan fits `map` seen from `pose` (log-likelihood, higher is
    // better), and the hill-climbing scan matcher. Exposed for tests.
    double scanFit(const LaserScan& scan, const OccupancyMap& map, const Pose& pose) const;
    Pose matchScan(const LaserScan& scan, const OccupancyMap& map, const Pose& guess) const;

private:
    struct Particle {
        Pose pose;
        OccupancyMap map;
        double logWeight = 0;
    };

    SlamConfig config_;
    std::mt19937 rng_;
    std::vector<Particle> particles_;
    size_t best_ = 0;
    int resamples_ = 0;
    double movedSinceUpdate_ = 0;
    double turnedSinceUpdate_ = 0;
    double lastMotion_ = 0;  // m + rad of the most recent odometry change
    bool updatedOnce_ = false;
};
