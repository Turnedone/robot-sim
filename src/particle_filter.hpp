#pragma once

#include <random>
#include <vector>

#include "lidar.hpp"
#include "occupancy_map.hpp"
#include "robot.hpp"

// Monte Carlo localization (MCL), the algorithm behind ROS's AMCL.
// Follows "Probabilistic Robotics" (Thrun, Burgard, Fox): the odometry motion
// model and the likelihood-field sensor model, plus two practical additions
// AMCL also has in some form: an adaptive particle count and re-localizing
// when the filter is confident but the scans disagree with the map.
struct MclConfig {
    int particles = 500;         // when confident (pose tracking)
    int globalParticles = 6000;  // when searching the whole map

    // Odometry motion model noise (Probabilistic Robotics, table 5.6):
    // how uncertain each part of a move is, in proportion to the move itself.
    double alpha1 = 0.08;  // rotation error from rotation
    double alpha2 = 0.02;  // rotation error from translation (per meter)
    double alpha3 = 0.08;  // translation error from translation
    double alpha4 = 0.02;  // translation error from rotation

    // Likelihood-field sensor model.
    int beamStep = 5;               // use every 5th beam (36 of 180): beams next to each other say much the same
    double hitSigma = 0.15;         // m: how far a beam's end may be from a wall and still "fit"
    double hitWeight = 0.95;        // share of readings explained by the map...
    double randomWeight = 0.05;     // ...and by something unexpected
    double fieldResolution = 0.05;  // m: cell size of the precomputed distance field
    double fieldMaxDistance = 1.0;  // m: distances beyond this are stored as this
    // Where the lidar sits on the robot, in the robot's frame (on a TurtleBot3
    // it is a few cm behind the centre). Beams start from there.
    Pose sensorOffset{};

    // Only update after the robot has moved or turned this much (AMCL:
    // update_min_d, update_min_a). A robot standing still sees the same scan
    // over and over; weighing particles by it again and again would collapse
    // the cloud onto one point with nothing to correct it.
    double updateMinDistance = 0.1;  // m
    double updateMinAngle = 0.15;    // rad

    // Confident = the particles are within this spread of the estimate. Then
    // the filter shrinks to `particles`.
    double confidentSpread = 0.3;  // m

    // Lost = confident, but from the estimate fewer than `lostBelow` of the
    // beams measure what the map says they should ("scan match"), for
    // `lostUpdates` updates in a row. Then the filter starts a global search again.
    double matchDistance = 0.15;  // m: a beam "matches" if its range is within this of the expected range
    double lostBelow = 0.85;      // correctly localized, 95-100% of beams match
    int lostUpdates = 10;
};

struct Particle {
    Pose pose;
    double weight = 0;
};

class ParticleFilter {
public:
    explicit ParticleFilter(const MclConfig& config = {}, unsigned seed = 7);

    // Pose tracking: the robot roughly knows where it starts.
    void initAround(const Pose& guess, double sigmaXY, double sigmaTheta);
    // Global localization: no idea where it is; particles anywhere free on the map.
    void initUniform(const OccupancyMap& map);

    // 1. PREDICT: move every particle by the odometry change between two
    //    odometry poses, with random noise added (the wheels may have slipped).
    void predict(const Pose& odomBefore, const Pose& odomAfter);
    // 2. CORRECT: weigh every particle by how well the scan fits the map from
    //    its pose. 3. RESAMPLE: draw a new set in proportion to the weights.
    //    Skipped (returns false) until the robot has moved far enough.
    //    The map must stay the same between calls (it is precomputed).
    bool correct(const LaserScan& scan, const OccupancyMap& map);

    // Best guess: the weighted mean of the particles near the best one (a plain
    // mean of all particles would land between two clusters).
    Pose estimate() const { return estimate_; }
    // Spread of the particles around the estimate, m: small = confident.
    double spread() const;
    const std::vector<Particle>& particles() const { return particles_; }
    // Share of beams whose range fits the map, seen from the estimate (0..1).
    double scanMatch() const { return scanMatch_; }
    // How often the filter decided it was lost and searched the whole map again.
    int relocalizations() const { return relocalizations_; }
    // Confident AND the last scan matched the map: safe to navigate on the
    // estimate. A robot should not drive towards a goal while this is false.
    bool healthy() const { return spread() < config_.confidentSpread && scanMatch_ >= config_.lostBelow; }

    // How well a scan fits the map seen from `pose`: sum of log-likelihoods of
    // the used beams. Higher = better fit. Exposed for tests.
    double scanLogLikelihood(const LaserScan& scan, const OccupancyMap& map, const Pose& robot) const;
    // Share of a scan's beams (with a return) whose range is within
    // matchDistance of what the map predicts from `pose`. Exposed for tests.
    double scanMatchAt(const LaserScan& scan, const OccupancyMap& map, const Pose& robot) const;

private:
    // Distance from any point to the nearest wall, precomputed on a fine grid
    // once per map: the "likelihood field". One lookup per beam instead of a
    // search, which is what makes thousands of particles affordable.
    class DistanceField {
    public:
        void build(const OccupancyMap& map, double resolution, double maxDistance);
        bool builtFor(const OccupancyMap& map) const { return source_ == &map; }
        double at(Point p) const;

    private:
        const OccupancyMap* source_ = nullptr;
        double resolution_ = 0;
        double maxDistance_ = 0;
        int width_ = 0;
        int height_ = 0;
        std::vector<float> distance_;
    };

    const DistanceField& field(const OccupancyMap& map) const;
    Pose randomFreePose(const OccupancyMap& map);
    void computeEstimate();
    void resample(size_t count);

    MclConfig config_;
    std::mt19937 rng_;
    std::vector<Particle> particles_;
    Pose estimate_;
    double scanMatch_ = 1;
    int lostCount_ = 0;
    int relocalizations_ = 0;
    double movedSinceUpdate_ = 0;   // m of odometry since the last correction
    double turnedSinceUpdate_ = 0;  // rad of odometry since the last correction
    bool updatedOnce_ = false;
    mutable DistanceField field_;  // a cache: built on first use for a map
};
