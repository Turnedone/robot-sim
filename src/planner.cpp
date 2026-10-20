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
    const auto blocked = [&](Pos p) { return !map.inBounds(p) || map.classify(p) == Occupancy::Occupied; };
    // Cutting diagonally past a corner is only safe if both corner cells are KNOWN
    // to be free: an unseen corner may well turn out to be a wall.
    const auto safeCorner = [&](Pos p) { return map.inBounds(p) && map.classify(p) == Occupancy::Free; };
    if (blocked(goal)) return std::nullopt;

    // Cost of stepping INTO a cell, per cell of distance.
    const auto cellCost = [&](Pos p) {
        double cost = 1.0;
        if (map.classify(p) == Occupancy::Unknown) cost += config.unknownPenalty;
        for (int dy = -1; dy <= 1; ++dy)
            for (int dx = -1; dx <= 1; ++dx)
                if ((dx || dy) && map.inBounds({p.x + dx, p.y + dy}) &&
                    map.classify({p.x + dx, p.y + dy}) == Occupancy::Occupied) {
                    return cost + config.nearWallPenalty;
                }
        return cost;
    };

    const int w = map.width();
    const size_t n = cellCount(w, map.height());
    const auto idx = [w](Pos p) { return cellIndex(p, w); };
    constexpr double kInf = std::numeric_limits<double>::infinity();

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
                if (blocked(next)) continue;
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
    return path;
}
