#include "slam.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace {

double sampleWithVariance(std::mt19937& rng, double variance) {
    if (variance <= 0) return 0.0;
    return std::normal_distribution<double>(0.0, std::sqrt(variance))(rng);
}

}  // namespace

GridSlam::GridSlam(int width, int height, double resolution, const Pose& start, const SlamConfig& config, unsigned seed)
    : config_(config), rng_(seed) {
    particles_.reserve(static_cast<size_t>(config_.particles));
    for (int i = 0; i < config_.particles; ++i)
        particles_.push_back({start, OccupancyMap(width, height, resolution), 0.0});
}

std::vector<Pose> GridSlam::poses() const {
    std::vector<Pose> out;
    out.reserve(particles_.size());
    for (const Particle& p : particles_) out.push_back(p.pose);
    return out;
}

void GridSlam::predict(const Pose& before, const Pose& after) {
    // The same odometry motion model as the localizer: turn, drive, turn.
    const double dx = after.x - before.x;
    const double dy = after.y - before.y;
    const double trans = std::hypot(dx, dy);
    const double rot1 = trans < 0.01 ? 0.0 : normalizeAngle(std::atan2(dy, dx) - before.theta);
    const double rot2 = normalizeAngle(after.theta - before.theta - rot1);
    const bool backwards = std::abs(rot1) > kPi / 2;
    const double r1 = backwards ? normalizeAngle(rot1 - kPi) : rot1;
    const double r2 = backwards ? normalizeAngle(rot2 - kPi) : rot2;
    const double t = backwards ? -trans : trans;
    movedSinceUpdate_ += trans;
    turnedSinceUpdate_ += std::abs(normalizeAngle(after.theta - before.theta));
    lastMotion_ = trans + std::abs(normalizeAngle(after.theta - before.theta));

    const double a1 = config_.alpha1, a2 = config_.alpha2, a3 = config_.alpha3, a4 = config_.alpha4;
    for (Particle& p : particles_) {
        const double nr1 = r1 - sampleWithVariance(rng_, a1 * r1 * r1 + a2 * t * t);
        const double nt = t - sampleWithVariance(rng_, a3 * t * t + a4 * (r1 * r1 + r2 * r2));
        const double nr2 = r2 - sampleWithVariance(rng_, a1 * r2 * r2 + a2 * t * t);
        p.pose.x += nt * std::cos(p.pose.theta + nr1);
        p.pose.y += nt * std::sin(p.pose.theta + nr1);
        p.pose.theta = normalizeAngle(p.pose.theta + nr1 + nr2);
    }
}

double GridSlam::scanFit(const LaserScan& scan, const OccupancyMap& map, const Pose& pose) const {
    // Likelihood field on the particle's own map: beams whose ends lie near
    // walls the particle has already mapped fit well.
    const auto isWall = [&map](Pos c) { return map.inBounds(c) && map.logOdds(c) > 0; };
    const double s2 = config_.hitSigma * config_.hitSigma;
    double fit = 0;
    for (size_t i = 0; i < scan.ranges.size(); i += static_cast<size_t>(config_.beamStep)) {
        const double r = scan.ranges[i];
        if (!std::isfinite(r) || r > config_.matchMaxRange) continue;
        const double a = pose.theta + scan.angle(i);
        const Point end{pose.x + r * std::cos(a), pose.y + r * std::sin(a)};
        // A beam ending where this map has never looked says nothing about the
        // pose. Scoring it as a bad fit would pull every match back towards the
        // already-mapped area, so while exploring the estimate would lag behind.
        if (!map.observed(map.worldToCell(end))) continue;
        const double d = distanceToNearestWall(end, config_.maxWallDistance, map.height(), map.resolution(), isWall);
        // A Gaussian around the nearest wall plus a small floor, so one
        // unexplained beam (a new, not yet mapped wall) does not dominate.
        fit += std::log(std::exp(-0.5 * d * d / s2) + 0.05);
    }
    return fit;
}

Pose GridSlam::matchScan(const LaserScan& scan, const OccupancyMap& map, const Pose& guess) const {
    // Hill climbing (as in GMapping): try a small step in each direction of
    // x, y and heading; keep any improvement; when none helps, halve the steps.
    const auto score = [&](const Pose& p) {
        double s = scanFit(scan, map, p);
        if (config_.priorSigmaXY > 0) {
            const double dxy2 = (p.x - guess.x) * (p.x - guess.x) + (p.y - guess.y) * (p.y - guess.y);
            const double dth = normalizeAngle(p.theta - guess.theta);
            s -= 0.5 * dxy2 / (config_.priorSigmaXY * config_.priorSigmaXY);
            s -= 0.5 * dth * dth / (config_.priorSigmaTheta * config_.priorSigmaTheta);
        }
        return s;
    };
    Pose best = guess;
    double bestFit = score(best);
    double stepXY = config_.initialStepXY;
    double stepTh = config_.initialStepTheta;
    for (int it = 0; it < config_.maxIterations && stepXY > 0.005; ++it) {
        bool improved = false;
        const Pose candidates[] = {{best.x + stepXY, best.y, best.theta}, {best.x - stepXY, best.y, best.theta},
                                   {best.x, best.y + stepXY, best.theta}, {best.x, best.y - stepXY, best.theta},
                                   {best.x, best.y, best.theta + stepTh}, {best.x, best.y, best.theta - stepTh}};
        for (const Pose& c : candidates) {
            const double fit = score(c);
            if (fit > bestFit) {
                bestFit = fit;
                best = c;
                improved = true;
            }
        }
        if (!improved) {
            stepXY /= 2;
            stepTh /= 2;
        }
    }
    best.theta = normalizeAngle(best.theta);
    return best;
}

bool GridSlam::update(const LaserScan& scan) {
    if (updatedOnce_ && movedSinceUpdate_ < config_.updateMinDistance && turnedSinceUpdate_ < config_.updateMinAngle) {
        // Really standing still (not just between updates): no scan matching or
        // re-weighting (the same scan over and over would collapse the
        // weights), but the maps still take the scan. Otherwise a robot that
        // stopped, e.g. because a false wall blocked its only path, would never
        // see that wall disappear. While moving, poses between updates are
        // only predictions, so scans are not added then.
        if (lastMotion_ < 1e-6)
            for (Particle& p : particles_) p.map.integrate(scan, p.pose);
        return false;
    }
    const bool first = !updatedOnce_;
    updatedOnce_ = true;
    movedSinceUpdate_ = 0;
    turnedSinceUpdate_ = 0;

    for (Particle& p : particles_) {
        if (!first) {
            // 1. Improve the pose by matching the scan to this particle's map,
            // 2. then weigh the particle by how well the scan fits there.
            p.pose = matchScan(scan, p.map, p.pose);
            p.logWeight += scanFit(scan, p.map, p.pose);
        }
        // 3. Add the scan to this particle's map, from this particle's pose.
        p.map.integrate(scan, p.pose);
    }

    // Normalise (in log space), find the best particle, and resample if the
    // weights have become uneven.
    const double maxLog =
        std::max_element(particles_.begin(), particles_.end(), [](const Particle& a, const Particle& b) {
            return a.logWeight < b.logWeight;
        })->logWeight;
    std::vector<double> w(particles_.size());
    double sum = 0;
    for (size_t i = 0; i < particles_.size(); ++i) {
        w[i] = std::exp(particles_[i].logWeight - maxLog);
        sum += w[i];
    }
    double sumSquares = 0;
    best_ = 0;
    for (size_t i = 0; i < w.size(); ++i) {
        w[i] /= sum;
        sumSquares += w[i] * w[i];
        if (w[i] > w[best_]) best_ = i;
    }
    for (size_t i = 0; i < particles_.size(); ++i) particles_[i].logWeight = std::log(std::max(w[i], 1e-300));

    const double effectiveShare = 1.0 / sumSquares / static_cast<double>(particles_.size());
    if (effectiveShare < config_.resampleBelowEss) {
        // Low-variance resampling; copies keep their whole map history.
        std::vector<Particle> next;
        next.reserve(particles_.size());
        const double step = 1.0 / static_cast<double>(particles_.size());
        double pick = std::uniform_real_distribution<double>(0.0, step)(rng_);
        double cumulative = w[0];
        size_t i = 0;
        size_t newBest = 0;
        for (size_t m = 0; m < particles_.size(); ++m) {
            while (pick > cumulative && i + 1 < w.size()) cumulative += w[++i];
            next.push_back(particles_[i]);
            next.back().logWeight = 0;
            if (i == best_) newBest = m;
            pick += step;
        }
        particles_ = std::move(next);
        best_ = newBest;
        ++resamples_;
    }
    return true;
}
