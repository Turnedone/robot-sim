#include "viewer.hpp"

#include <raylib.h>

namespace {

constexpr int kCellSize = 32;     // pixels per map cell
constexpr int kHeaderHeight = 40; // strip at the top for text

const Color kFreeColor{235, 235, 230, 255};
const Color kWallColor{60, 63, 70, 255};
const Color kGridLineColor{210, 210, 205, 255};
const Color kRobotColor{40, 110, 220, 255};
const Color kGoalColor{40, 170, 90, 255};
const Color kHeaderColor{30, 32, 36, 255};

// Top-left pixel of a map cell on screen.
Vector2 cellToScreen(Pos p) {
    return {static_cast<float>(p.x * kCellSize), static_cast<float>(kHeaderHeight + p.y * kCellSize)};
}

void drawGrid(const Grid& grid) {
    for (int y = 0; y < grid.height(); ++y) {
        for (int x = 0; x < grid.width(); ++x) {
            const Vector2 s = cellToScreen({x, y});
            const Color c = grid.at({x, y}) == Cell::Wall ? kWallColor : kFreeColor;
            DrawRectangle(static_cast<int>(s.x), static_cast<int>(s.y), kCellSize, kCellSize, c);
            DrawRectangleLines(static_cast<int>(s.x), static_cast<int>(s.y), kCellSize, kCellSize, kGridLineColor);
        }
    }
}

void drawGoal(Pos goal) {
    const Vector2 s = cellToScreen(goal);
    const int pad = kCellSize / 5;
    DrawRectangle(static_cast<int>(s.x) + pad, static_cast<int>(s.y) + pad, kCellSize - 2 * pad,
                  kCellSize - 2 * pad, kGoalColor);
}

void drawRobot(Pos robot) {
    const Vector2 s = cellToScreen(robot);
    const Vector2 centre{s.x + kCellSize / 2.0f, s.y + kCellSize / 2.0f};
    DrawCircleV(centre, kCellSize * 0.38f, kRobotColor);
}

}  // namespace

void runViewer(const Scenario& scenario) {
    const Grid& grid = scenario.grid;
    const int width = grid.width() * kCellSize;
    const int height = kHeaderHeight + grid.height() * kCellSize;

    SetConfigFlags(FLAG_MSAA_4X_HINT);  // smoother circles
    InitWindow(width, height, "robot-sim");
    SetTargetFPS(60);

    while (!WindowShouldClose()) {
        BeginDrawing();
        ClearBackground(kHeaderColor);
        DrawText("robot-sim   |   blue: robot   green: goal   |   Esc to quit", 12, 11, 18, RAYWHITE);
        drawGrid(grid);
        drawGoal(scenario.goal);
        drawRobot(scenario.start);
        EndDrawing();
    }
    CloseWindow();
}
