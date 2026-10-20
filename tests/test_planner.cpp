// A* on the robot's own map.

#include <gtest/gtest.h>

#include <algorithm>

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
