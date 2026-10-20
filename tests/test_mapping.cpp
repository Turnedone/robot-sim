// The robot's own map, built only from lidar scans, against the real world.

#include <gtest/gtest.h>

#include <cmath>

#include "lidar.hpp"
#include "occupancy_map.hpp"

namespace {

LidarConfig perfect() {
    LidarConfig c;
    c.noiseStdDev = 0;
    c.dropoutChance = 0;
    return c;
}

// A 5 m x 5 m room: 10 x 10 cells with walls all round and one pillar at (5, 5).
Grid roomWithPillar() {
    Grid g(10, 10, 0.5);
    for (int i = 0; i < 10; ++i) {
        g.set({i, 0}, Cell::Wall);
        g.set({i, 9}, Cell::Wall);
        g.set({0, i}, Cell::Wall);
        g.set({9, i}, Cell::Wall);
    }
    g.set({5, 5}, Cell::Wall);
    return g;
}

Pose poseAtCell(const Grid& g, Pos cell) {
    const Point c = g.cellCenter(cell);
    return {c.x, c.y, 0};
}

void scanRepeatedly(OccupancyMap& map, Lidar& lidar, const Grid& world, const Pose& pose, int times) {
    for (int i = 0; i < times; ++i) map.integrate(lidar.scan(world, pose), pose);
}

class Mapping : public ::testing::Test {
protected:
    Grid world = roomWithPillar();
    Pose robot = poseAtCell(world, {2, 5});  // same row as the pillar, 3 cells left of it
    OccupancyMap map{10, 10, 0.5};
    Lidar lidar{perfect()};
};

}  // namespace

TEST(LogOdds, ConvertsToProbability) {
    EXPECT_DOUBLE_EQ(logOddsToProbability(0), 0.5);
    EXPECT_NEAR(logOddsToProbability(std::log(0.7 / 0.3)), 0.7, 1e-12);
}

TEST_F(Mapping, NewMapKnowsNothing) {
    EXPECT_EQ(map.classify({3, 3}), Occupancy::Unknown);
    EXPECT_DOUBLE_EQ(map.probability({3, 3}), 0.5);
}

TEST_F(Mapping, OneScanIsOneVoteEitherWay) {
    scanRepeatedly(map, lidar, world, robot, 1);
    EXPECT_EQ(map.classify({3, 5}), Occupancy::Unknown);
    EXPECT_LT(map.probability({3, 5}), 0.5) << "the gap in front of the pillar leans free";
    EXPECT_EQ(map.classify({5, 5}), Occupancy::Unknown);
    EXPECT_GT(map.probability({5, 5}), 0.5) << "the pillar leans wall";
}

TEST_F(Mapping, ThreeScansClassifyWhatTheRobotSees) {
    scanRepeatedly(map, lidar, world, robot, 3);
    EXPECT_EQ(map.classify({3, 5}), Occupancy::Free);
    EXPECT_EQ(map.classify({5, 5}), Occupancy::Occupied) << "pillar";
    EXPECT_EQ(map.classify({0, 5}), Occupancy::Occupied) << "outer wall next to the robot";
}

TEST_F(Mapping, HiddenAndDistantCellsStayUnknown) {
    scanRepeatedly(map, lidar, world, robot, 3);
    EXPECT_FALSE(map.observed({7, 5})) << "hidden behind the pillar";
    EXPECT_EQ(map.classify({9, 3}), Occupancy::Unknown) << "3.3 m: seen, but beyond the 2.5 m usable range";
    EXPECT_FALSE(map.observed({9, 2})) << "3.69 m: beyond the 3.5 m lidar range";
}

TEST_F(Mapping, RemovedObstacleFades) {
    scanRepeatedly(map, lidar, world, robot, 30);
    Grid moved = world;
    moved.set({5, 5}, Cell::Free);
    scanRepeatedly(map, lidar, moved, robot, 15);
    EXPECT_EQ(map.classify({5, 5}), Occupancy::Free);
}

TEST_F(Mapping, NoisyLidarMapsTheRoomWithoutMistakes) {
    Lidar noisy(LidarConfig{}, 99);
    for (Pos at : {Pos{2, 5}, Pos{2, 2}, Pos{7, 2}, Pos{7, 7}, Pos{3, 7}})
        scanRepeatedly(map, noisy, world, poseAtCell(world, at), 10);

    int sure = 0;
    for (int y = 0; y < 10; ++y) {
        for (int x = 0; x < 10; ++x) {
            const Occupancy o = map.classify({x, y});
            if (o == Occupancy::Unknown) continue;
            ++sure;
            const bool isWall = world.at({x, y}) == Cell::Wall;
            EXPECT_EQ(o == Occupancy::Occupied, isWall) << "cell (" << x << ", " << y << ")";
        }
    }
    // The 4 corner cells can never be seen properly: the walls next to them block the beams.
    EXPECT_GE(sure, 96);
}
