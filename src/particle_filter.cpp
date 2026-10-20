#include "particle_filter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "raycast.hpp"

namespace {

// A sample from a normal distribution with this VARIANCE (0 -> exactly 0).
double sampleWithVariance(std::mt19937& rng, double variance) {
    if (variance <= 0) return 0.0;
    return std::normal_distribution<double>(0.0, std::sqrt(variance))(rng);
}

bool isWallOrOutside(const OccupancyMap& map, Pos cell) {
    return !map.inBounds(cell) || map.classify(cell) == Occupancy::Occupied;
}

}  // namespace

// --- Distance field ----------------------------------------------------------

void ParticleFilter::DistanceField::build(const OccupancyMap& map, double resolution, double maxDistance) {
    source_ = &map;
    resolution_ = resolution;
    width_ = static_cast<int>(std::ceil(map.width() * map.resolution() / resolution));
    height_ = static_cast<int>(std::ceil(map.height() * map.resolution() / resolution));
    maxDistance_ = maxDistance;
    distance_.assign(static_cast<size_t>(width_) * static_cast<size_t>(height_), 0.0F);
    // A beam ends at a wall's SURFACE. So the distance that matters is the
    // distance to the nearest surface: from free space, to the nearest wall
    // cell; from inside a wall, to the nearest free cell (a beam cannot end
    // deep inside a wall). Outside the map is neither: nothing is known there.
    const auto isWall = [&map](Pos c) { return map.inBounds(c) && map.classify(c) == Occupancy::Occupied; };
    const auto isOpen = [&map](Pos c) { return map.inBounds(c) && map.classify(c) != Occupancy::Occupied; };
    for (int iy = 0; iy < height_; ++iy) {
        for (int ix = 0; ix < width_; ++ix) {
            const Point centre{(ix + 0.5) * resolution, (iy + 0.5) * resolution};
            const bool insideWall = isWall(map.worldToCell(centre));
            const double d = insideWall
                                 ? distanceToNearestWall(centre, maxDistance, map.height(), map.resolution(), isOpen)
                                 : distanceToNearestWall(centre, maxDistance, map.height(), map.resolution(), isWall);
            distance_[static_cast<size_t>(iy) * static_cast<size_t>(width_) + static_cast<size_t>(ix)] =
                static_cast<float>(d);
        }
    }
}

double ParticleFilter::DistanceField::at(Point p) const {
    const int ix = static_cast<int>(std::floor(p.x / resolution_));
    const int iy = static_cast<int>(std::floor(p.y / resolution_));
    // Outside the map: as far from any known wall as it gets. (Treating it as
    // "at a wall" made poses near the map's edge fit almost any scan.)
    if (ix < 0 || iy < 0 || ix >= width_ || iy >= height_) return maxDistance_;
    return distance_[static_cast<size_t>(iy) * static_cast<size_t>(width_) + static_cast<size_t>(ix)];
}

const ParticleFilter::DistanceField& ParticleFilter::field(const OccupancyMap& map) const {
    if (!field_.builtFor(map)) field_.build(map, config_.fieldResolution, config_.fieldMaxDistance);
    return field_;
}

// --- Initialisation ----------------------------------------------------------

ParticleFilter::ParticleFilter(const MclConfig& config, unsigned seed) : config_(config), rng_(seed) {}

void ParticleFilter::initAround(const Pose& guess, double sigmaXY, double sigmaTheta) {
    particles_.assign(static_cast<size_t>(config_.particles), {});
    for (Particle& p : particles_) {
        p.pose = {guess.x + sampleWithVariance(rng_, sigmaXY * sigmaXY),
                  guess.y + sampleWithVariance(rng_, sigmaXY * sigmaXY),
                  normalizeAngle(guess.theta + sampleWithVariance(rng_, sigmaTheta * sigmaTheta))};
        p.weight = 1.0 / config_.particles;
    }
    updatedOnce_ = false;
    lostCount_ = 0;
    computeEstimate();
}

Pose ParticleFilter::randomFreePose(const OccupancyMap& map) {
    std::uniform_real_distribution<double> x(0.0, map.width() * map.resolution());
    std::uniform_real_distribution<double> y(0.0, map.height() * map.resolution());
    std::uniform_real_distribution<double> th(-kPi, kPi);
    for (;;) {
        const Point p{x(rng_), y(rng_)};
        if (map.classify(map.worldToCell(p)) == Occupancy::Free) return {p.x, p.y, th(rng_)};
    }
}

void ParticleFilter::initUniform(const OccupancyMap& map) {
    particles_.assign(static_cast<size_t>(config_.globalParticles), {});
    for (Particle& p : particles_) {
        p.pose = randomFreePose(map);
        p.weight = 1.0 / config_.globalParticles;
    }
    updatedOnce_ = false;
    lostCount_ = 0;
    computeEstimate();
}

// --- 1. Predict --------------------------------------------------------------

void ParticleFilter::predict(const Pose& before, const Pose& after) {
    // Describe the odometry move as: turn (rot1), drive straight (trans), turn (rot2).
    const double dx = after.x - before.x;
    const double dy = after.y - before.y;
    const double trans = std::hypot(dx, dy);
    const double rot1 = trans < 0.01 ? 0.0 : normalizeAngle(std::atan2(dy, dx) - before.theta);
    const double rot2 = normalizeAngle(after.theta - before.theta - rot1);
    // Driving backwards: describe it as a backwards move, not a half turn.
    const bool backwards = std::abs(rot1) > kPi / 2;
    const double r1 = backwards ? normalizeAngle(rot1 - kPi) : rot1;
    const double r2 = backwards ? normalizeAngle(rot2 - kPi) : rot2;
    const double t = backwards ? -trans : trans;

    movedSinceUpdate_ += trans;
    turnedSinceUpdate_ += std::abs(normalizeAngle(after.theta - before.theta));

    const double a1 = config_.alpha1, a2 = config_.alpha2, a3 = config_.alpha3, a4 = config_.alpha4;
    for (Particle& p : particles_) {
        // Each particle gets its own noisy version of the move (Prob. Robotics, table 5.6).
        const double nr1 = r1 - sampleWithVariance(rng_, a1 * r1 * r1 + a2 * t * t);
        const double nt = t - sampleWithVariance(rng_, a3 * t * t + a4 * (r1 * r1 + r2 * r2));
        const double nr2 = r2 - sampleWithVariance(rng_, a1 * r2 * r2 + a2 * t * t);
        p.pose.x += nt * std::cos(p.pose.theta + nr1);
        p.pose.y += nt * std::sin(p.pose.theta + nr1);
        p.pose.theta = normalizeAngle(p.pose.theta + nr1 + nr2);
    }
}

// --- 2. Correct --------------------------------------------------------------

double ParticleFilter::scanLogLikelihood(const LaserScan& scan, const OccupancyMap& map, const Pose& pose) const {
    // Likelihood field: a beam "fits" if its end point lies close to a wall.
    // It does not trace the beam, which makes it fast and smooth.
    const DistanceField& distances = field(map);
    const double norm = 1.0 / (std::sqrt(2 * kPi) * config_.hitSigma);
    const double randomDensity = 1.0 / scan.rangeMax;
    double logSum = 0;
    for (size_t i = 0; i < scan.ranges.size(); i += static_cast<size_t>(config_.beamStep)) {
        const double r = scan.ranges[i];
        if (!std::isfinite(r)) continue;  // no return: no information here (as in AMCL)
        const double a = pose.theta + scan.angle(i);
        const double d = distances.at({pose.x + r * std::cos(a), pose.y + r * std::sin(a)});
        const double hit = norm * std::exp(-0.5 * d * d / (config_.hitSigma * config_.hitSigma));
        logSum += std::log(config_.hitWeight * hit + config_.randomWeight * randomDensity);
    }
    return logSum;
}

double ParticleFilter::scanMatchAt(const LaserScan& scan, const OccupancyMap& map, const Pose& pose) const {
    // Cast each beam on the map from `pose` and compare the distance it SHOULD
    // measure with what it did measure. Unlike the likelihood field this also
    // uses the free space along the beam: from a wrong pose, a beam whose end
    // lands near some wall still fails if it would have had to pass through
    // another wall to get there. Too slow for thousands of particles, but only
    // run once per update, for the estimate.
    int used = 0;
    int matched = 0;
    for (size_t i = 0; i < scan.ranges.size(); i += static_cast<size_t>(config_.beamStep)) {
        const double r = scan.ranges[i];
        if (!std::isfinite(r)) continue;
        const double angle = pose.theta + scan.angle(i);
        double expected = std::numeric_limits<double>::infinity();
        walkRay(map.height(), map.resolution(), {pose.x, pose.y}, angle, scan.rangeMax + config_.matchDistance,
                [&](Pos cell, double enter) {
                    if (!isWallOrOutside(map, cell)) return true;
                    expected = enter;
                    return false;
                });
        ++used;
        if (std::abs(expected - r) < config_.matchDistance) ++matched;
    }
    return used == 0 ? 1.0 : static_cast<double>(matched) / used;
}

bool ParticleFilter::correct(const LaserScan& scan, const OccupancyMap& map) {
    if (particles_.empty()) return false;
    if (updatedOnce_ && movedSinceUpdate_ < config_.updateMinDistance && turnedSinceUpdate_ < config_.updateMinAngle) {
        return false;
    }
    updatedOnce_ = true;
    movedSinceUpdate_ = 0;
    turnedSinceUpdate_ = 0;

    // Weigh. Work with logarithms: a product of 36 small numbers underflows.
    std::vector<double> logWeights(particles_.size());
    for (size_t i = 0; i < particles_.size(); ++i) {
        const Pose& pose = particles_[i].pose;
        // A robot cannot be inside a wall.
        logWeights[i] = isWallOrOutside(map, map.worldToCell({pose.x, pose.y}))
                            ? -std::numeric_limits<double>::infinity()
                            : scanLogLikelihood(scan, map, pose);
    }
    const double best = *std::max_element(logWeights.begin(), logWeights.end());
    if (!std::isfinite(best)) {  // every particle is inside a wall: certainly lost, search everywhere
        ++relocalizations_;
        initUniform(map);
        return true;
    }
    double sum = 0;
    for (size_t i = 0; i < particles_.size(); ++i) {
        particles_[i].weight = std::exp(logWeights[i] - best);
        sum += particles_[i].weight;
    }
    for (Particle& p : particles_) p.weight /= sum;
    computeEstimate();  // from the weighted particles, before resampling

    // Confident but wrong? Check the scan against the map from the estimate.
    scanMatch_ = scanMatchAt(scan, map, estimate_);
    const bool confident = spread() < config_.confidentSpread;
    lostCount_ = confident && scanMatch_ < config_.lostBelow ? lostCount_ + 1 : 0;
    if (lostCount_ >= config_.lostUpdates) {
        ++relocalizations_;
        initUniform(map);  // search the whole map again
        return true;
    }

    // 3. Resample. Many particles while searching, few once confident.
    resample(confident ? static_cast<size_t>(config_.particles) : particles_.size());
    return true;
}

// --- 3. Resample -------------------------------------------------------------

void ParticleFilter::resample(size_t count) {
    // Low-variance ("systematic") resampling: one random number, then evenly
    // spaced picks, so particles are kept in proportion to their weight with
    // as little extra randomness as possible.
    std::vector<Particle> next;
    next.reserve(count);
    const double step = 1.0 / static_cast<double>(count);
    double pick = std::uniform_real_distribution<double>(0.0, step)(rng_);
    double cumulative = particles_[0].weight;
    size_t i = 0;
    for (size_t m = 0; m < count; ++m) {
        while (pick > cumulative && i + 1 < particles_.size()) cumulative += particles_[++i].weight;
        next.push_back({particles_[i].pose, step});
        pick += step;
    }
    particles_ = std::move(next);
}

// --- Estimate ----------------------------------------------------------------

void ParticleFilter::computeEstimate() {
    if (particles_.empty()) return;
    const Particle& best = *std::max_element(particles_.begin(), particles_.end(),
                                             [](const Particle& a, const Particle& b) { return a.weight < b.weight; });
    // Weighted mean of the particles within 0.5 m of the best one. Angles are
    // averaged as unit vectors, so 179 deg and -179 deg average to 180, not 0.
    double sx = 0, sy = 0, sc = 0, ss = 0, sw = 0;
    for (const Particle& p : particles_) {
        if (std::hypot(p.pose.x - best.pose.x, p.pose.y - best.pose.y) > 0.5) continue;
        sx += p.weight * p.pose.x;
        sy += p.weight * p.pose.y;
        sc += p.weight * std::cos(p.pose.theta);
        ss += p.weight * std::sin(p.pose.theta);
        sw += p.weight;
    }
    estimate_ = sw > 0 ? Pose{sx / sw, sy / sw, std::atan2(ss, sc)} : best.pose;
}

double ParticleFilter::spread() const {
    if (particles_.empty()) return 0;
    // Weighted, so right after an update it reflects the new weights.
    double sum = 0;
    double weights = 0;
    for (const Particle& p : particles_) {
        const double dx = p.pose.x - estimate_.x;
        const double dy = p.pose.y - estimate_.y;
        sum += p.weight * (dx * dx + dy * dy);
        weights += p.weight;
    }
    return weights > 0 ? std::sqrt(sum / weights) : 0.0;
}
