#include "planner.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>

namespace {

constexpr double kSqrt2 = 1.4142135623730951;

// Straight-line-ish distance on an 8-connected grid ("octile" distance).
// It never overestimates the true cost, which is what makes A* optimal.
double octile(Pos a, Pos b) {
    const int dx = std::abs(a.x - b.x);
    const int dy = std::abs(a.y - b.y);
    return (dx + dy) + (kSqrt2 - 2) * std::min(dx, dy);
}

}  // namespace

std::optional<std::vector<Pos>> planPath(const OccupancyMap& map, Pos start, Pos goal, const PlannerConfig& config) {
    if (!map.inBounds(start) || !map.inBounds(goal)) return std::nullopt;
    const auto blocked = [&](Pos p) {
        if (!map.inBounds(p) || map.classify(p) != Occupancy::Occupied) return !map.inBounds(p);
        return !config.passWeakWalls || map.logOdds(p) >= config.strongWall;
    };
    // Cutting diagonally past a corner is only safe if both corner cells are KNOWN
    // to be free: an unseen corner may well turn out to be a wall.
    const auto safeCorner = [&](Pos p) { return map.inBounds(p) && map.classify(p) == Occupancy::Free; };
    if (blocked(goal)) return std::nullopt;

    const int w = map.width();
    const int h = map.height();
    const size_t n = cellCount(w, h);
    const auto idx = [w](Pos p) { return cellIndex(p, w); };
    constexpr double kInf = std::numeric_limits<double>::infinity();

    // Inflation: distance from every cell's centre to the nearest wall cell's
    // edge. A Dijkstra flood fill from all walls at once that carries along
    // WHICH wall cell is nearest, and measures the true straight-line distance
    // to it. (Adding up grid steps instead overestimates diagonal-ish
    // distances, by enough to let a path graze a wall.)
    std::vector<double> wallDistance;
    const bool inflate = config.inflationRadius > 0;
    if (inflate) {
        const double res = map.resolution();
        wallDistance.assign(n, kInf);
        std::vector<int> nearestWall(n, -1);
        const auto distanceToCell = [&](Pos from, int wall) {
            const Pos q{wall % w, wall / w};
            const double dx = std::max(0.0, (std::abs(from.x - q.x) - 0.5) * res);
            const double dy = std::max(0.0, (std::abs(from.y - q.y) - 0.5) * res);
            return std::hypot(dx, dy);
        };
        using Item = std::pair<double, int>;
        std::priority_queue<Item, std::vector<Item>, std::greater<Item>> frontier;
        for (int y = 0; y < h; ++y)
            for (int x = 0; x < w; ++x)
                if (map.classify({x, y}) == Occupancy::Occupied) {
                    const int i = static_cast<int>(idx({x, y}));
                    wallDistance[static_cast<size_t>(i)] = 0;
                    nearestWall[static_cast<size_t>(i)] = i;
                    frontier.push({0.0, i});
                }
        while (!frontier.empty()) {
            const auto [d, i] = frontier.top();
            frontier.pop();
            if (d > wallDistance[static_cast<size_t>(i)]) continue;
            const Pos c{i % w, i / w};
            const int wall = nearestWall[static_cast<size_t>(i)];
            for (int dy = -1; dy <= 1; ++dy)
                for (int dx = -1; dx <= 1; ++dx) {
                    const Pos q{c.x + dx, c.y + dy};
                    if ((!dx && !dy) || !map.inBounds(q)) continue;
                    const double nd = distanceToCell(q, wall);
                    if (nd < wallDistance[idx(q)]) {
                        wallDistance[idx(q)] = nd;
                        nearestWall[idx(q)] = wall;
                        frontier.push({nd, static_cast<int>(idx(q))});
                    }
                }
        }
    }
    const auto tooClose = [&](Pos p) { return inflate && p != goal && wallDistance[idx(p)] < config.inflationRadius; };
    const auto impassable = [&](Pos p) { return blocked(p) || (map.inBounds(p) && tooClose(p)); };

    // Cost of stepping INTO a cell, per cell of distance.
    const auto cellCost = [&](Pos p) {
        double cost = 1.0;
        if (map.classify(p) == Occupancy::Unknown) cost += config.unknownPenalty;
        if (map.classify(p) == Occupancy::Occupied) cost += config.weakWallPenalty;  // only reachable when passable
        if (inflate) {
            const double beyond = wallDistance[idx(p)] - config.inflationRadius;
            if (config.clearanceRange > 0 && beyond < config.clearanceRange)
                cost += config.clearancePenalty * (1.0 - std::max(0.0, beyond) / config.clearanceRange);
            return cost;
        }
        for (int dy = -1; dy <= 1; ++dy)
            for (int dx = -1; dx <= 1; ++dx)
                if ((dx || dy) && map.inBounds({p.x + dx, p.y + dy}) &&
                    map.classify({p.x + dx, p.y + dy}) == Occupancy::Occupied) {
                    return cost + config.nearWallPenalty;
                }
        return cost;
    };

    std::vector<double> costSoFar(n, kInf);  // "g" in A* textbooks
    std::vector<int> cameFrom(n, -1);
    std::vector<bool> done(n, false);

    // Open list: cells to explore, cheapest (cost so far + estimate to goal) first.
    using Entry = std::pair<double, int>;  // (priority, cell index)
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
    costSoFar[idx(start)] = 0;
    open.push({octile(start, goal), static_cast<int>(idx(start))});

    while (!open.empty()) {
        const int current = open.top().second;
        open.pop();
        if (done[current]) continue;  // an outdated, more expensive entry
        done[current] = true;
        const Pos cur{current % w, current / w};
        if (cur == goal) break;

        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                if (!dx && !dy) continue;
                const Pos next{cur.x + dx, cur.y + dy};
                if (impassable(next)) continue;
                const bool diagonal = dx && dy;
                if (diagonal && !(safeCorner({cur.x + dx, cur.y}) && safeCorner({cur.x, cur.y + dy}))) continue;

                const double step = (diagonal ? kSqrt2 : 1.0) * cellCost(next);
                const double candidate = costSoFar[current] + step;
                if (candidate < costSoFar[idx(next)]) {
                    costSoFar[idx(next)] = candidate;
                    cameFrom[idx(next)] = current;
                    open.push({candidate + octile(next, goal), static_cast<int>(idx(next))});
                }
            }
        }
    }

    if (!done[idx(goal)]) return std::nullopt;

    // Walk back from the goal along cameFrom, then reverse.
    std::vector<Pos> path;
    for (int at = static_cast<int>(idx(goal)); at != -1; at = cameFrom[at]) path.push_back({at % w, at / w});
    std::reverse(path.begin(), path.end());
    if (!config.smoothPath || path.size() <= 2) return path;

    // String pulling: from each kept cell, go as far along the path as a
    // straight line allows (sampled every quarter cell, every sample must be in
    // a passable cell), and keep only that cell.
    const auto lineIsClear = [&](Pos a, Pos b) {
        const double dx = b.x - a.x;
        const double dy = b.y - a.y;
        const int samples = static_cast<int>(std::ceil(4 * std::max(std::abs(dx), std::abs(dy))));
        for (int k = 1; k < samples; ++k) {
            const double t = static_cast<double>(k) / samples;
            const Pos c{static_cast<int>(std::lround(a.x + dx * t)), static_cast<int>(std::lround(a.y + dy * t))};
            if (c != a && c != b && impassable(c)) return false;
        }
        return true;
    };
    std::vector<Pos> smooth{path.front()};
    size_t i = 0;
    while (i + 1 < path.size()) {
        size_t j = i + 1;
        while (j + 1 < path.size() && lineIsClear(path[i], path[j + 1])) ++j;
        smooth.push_back(path[j]);
        i = j;
    }
    return smooth;
}
