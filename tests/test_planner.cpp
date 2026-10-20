// A* on the robot's own map.

#include <gtest/gtest.h>

#include <algorithm>

#include "geometry.hpp"
#include "planner.hpp"

namespace {

bool onPath(const std::vector<Pos>& path, Pos p) {
    return std::find(path.begin(), path.end(), p) != path.end();
}

}  // namespace

TEST(Planner, OpenMapGivesAStraightLine) {
    const OccupancyMap map(10, 10, 0.5);  // nothing known yet
    const auto path = planPath(map, {0, 0}, {4, 0});
    ASSERT_TRUE(path);
    EXPECT_EQ(path->size(), 5U);
    EXPECT_EQ(path->front(), (Pos{0, 0}));
    EXPECT_EQ(path->back(), (Pos{4, 0}));
}

TEST(Planner, GoesThroughTheGapInAWall) {
    OccupancyMap map(10, 10, 0.5);
    for (int y = 0; y < 10; ++y)
        if (y != 7) map.set({5, y}, Occupancy::Occupied);
    const auto path = planPath(map, {1, 1}, {8, 1});
    ASSERT_TRUE(path);
    EXPECT_TRUE(onPath(*path, {5, 7}));
}

TEST(Planner, NoPathToAWalledInOrBlockedGoal) {
    OccupancyMap map(10, 10, 0.5);
    for (int i = 3; i <= 5; ++i) {
        map.set({i, 3}, Occupancy::Occupied);
        map.set({i, 5}, Occupancy::Occupied);
        map.set({3, i}, Occupancy::Occupied);
        map.set({5, i}, Occupancy::Occupied);
    }
    EXPECT_FALSE(planPath(map, {0, 0}, {4, 4})) << "walled in on all sides";
    EXPECT_FALSE(planPath(map, {0, 0}, {3, 3})) << "the goal is a wall";
}

TEST(Planner, NeverSqueezesDiagonallyBetweenWalls) {
    OccupancyMap map(10, 10, 0.5);
    map.set({1, 0}, Occupancy::Occupied);
    map.set({0, 1}, Occupancy::Occupied);
    EXPECT_FALSE(planPath(map, {0, 0}, {1, 1}));
}

TEST(Planner, PrefersKnownFreeSpaceOverUnknown) {
    // A pillar in the middle; going over it is through known free cells, going
    // under it through an unknown row. Both are about as long.
    OccupancyMap map(10, 10, 0.5);
    for (int x = 0; x < 10; ++x)
        for (int y = 0; y < 10; ++y) map.set({x, y}, Occupancy::Free);
    for (int y = 2; y <= 8; ++y) map.set({5, y}, Occupancy::Occupied);
    for (int x = 0; x < 10; ++x) map.set({x, 9}, Occupancy::Unknown);

    const auto path = planPath(map, {2, 5}, {8, 5});
    ASSERT_TRUE(path);
    const bool over = std::any_of(path->begin(), path->end(), [](Pos p) { return p.x == 5 && p.y <= 1; });
    const bool under = std::any_of(path->begin(), path->end(), [](Pos p) { return p.x == 5 && p.y >= 9; });
    EXPECT_TRUE(over);
    EXPECT_FALSE(under);
}

// A ROS-style fine map: 5 cm cells, a 4 m x 2 m room with a 1 m thick wall in
// the middle that leaves a 0.6 m wide gap at the bottom.
OccupancyMap fineRoom() {
    OccupancyMap map(80, 40, 0.05);
    for (int x = 0; x < 80; ++x)
        for (int y = 0; y < 40; ++y) map.set({x, y}, Occupancy::Free);
    for (int x = 0; x < 80; ++x) {
        map.set({x, 0}, Occupancy::Occupied);
        map.set({x, 39}, Occupancy::Occupied);
    }
    for (int y = 0; y < 40; ++y) {
        map.set({0, y}, Occupancy::Occupied);
        map.set({79, y}, Occupancy::Occupied);
    }
    for (int x = 30; x < 50; ++x)
        for (int y = 0; y < 26; ++y) map.set({x, y}, Occupancy::Occupied);  // gap: rows 26..38
    return map;
}

TEST(Planner, InflationKeepsTheRobotsRadiusFromEveryWall) {
    const OccupancyMap map = fineRoom();
    PlannerConfig c;
    c.inflationRadius = 0.22;
    const auto path = planPath(map, {10, 10}, {70, 10}, c);
    ASSERT_TRUE(path);
    for (Pos p : *path) {
        const Point centre = map.cellCenter(p);
        const auto isWall = [&map](Pos q) { return !map.inBounds(q) || map.classify(q) == Occupancy::Occupied; };
        EXPECT_GE(distanceToNearestWall(centre, 1.0, map.height(), map.resolution(), isWall), 0.22 - 1e-9)
            << "cell (" << p.x << ", " << p.y << ")";
    }
}

TEST(Planner, InflationClosesGapsTooNarrowForTheRobot) {
    const OccupancyMap map = fineRoom();  // the gap is 0.6 m: fits a 0.22 m radius robot, not a 0.35 m one
    PlannerConfig fits;
    fits.inflationRadius = 0.22;
    EXPECT_TRUE(planPath(map, {10, 10}, {70, 10}, fits));
    PlannerConfig tooBig;
    tooBig.inflationRadius = 0.35;
    EXPECT_FALSE(planPath(map, {10, 10}, {70, 10}, tooBig));
}

TEST(Planner, SmoothingTurnsAStaircaseIntoAFewStraightSegments) {
    const OccupancyMap map = fineRoom();
    PlannerConfig c;
    c.inflationRadius = 0.22;
    const auto raw = planPath(map, {10, 10}, {70, 10}, c);
    c.smoothPath = true;
    const auto smooth = planPath(map, {10, 10}, {70, 10}, c);
    ASSERT_TRUE(raw && smooth);
    EXPECT_GT(raw->size(), 60U);
    EXPECT_LE(smooth->size(), 6U) << "start, a few corners around the wall, goal";
    EXPECT_EQ(smooth->front(), raw->front());
    EXPECT_EQ(smooth->back(), raw->back());
}
