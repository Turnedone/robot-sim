#include "viewer.hpp"

#include <optional>

#include <raylib.h>

namespace {

constexpr int kCellSize = 32;     // pixels per map cell
constexpr int kHeaderHeight = 40; // strip at the top for text
constexpr int kMinWidth = 760;    // so the header text always fits

const Color kFreeColor{235, 235, 230, 255};
const Color kWallColor{60, 63, 70, 255};
const Color kGridLineColor{210, 210, 205, 255};
const Color kHoverColor{255, 200, 60, 255};
const Color kRobotColor{40, 110, 220, 255};
const Color kGoalColor{40, 170, 90, 255};
const Color kHeaderColor{30, 32, 36, 255};
const Color kHeaderDim{150, 150, 150, 255};

// Top-left pixel of a map cell on screen.
Vector2 cellToScreen(Pos p) {
    return {static_cast<float>(p.x * kCellSize), static_cast<float>(kHeaderHeight + p.y * kCellSize)};
}

// The reverse: which cell a pixel is in, or nothing if it is outside the map.
std::optional<Pos> screenToCell(Vector2 pixel, const Grid& grid) {
    if (pixel.y < kHeaderHeight) return std::nullopt;
    const Pos p{static_cast<int>(pixel.x) / kCellSize,
                (static_cast<int>(pixel.y) - kHeaderHeight) / kCellSize};
    if (!grid.inBounds(p)) return std::nullopt;
    return p;
}

// Arrow key pressed this frame as a one-cell step, or {0, 0} if none.
Pos readArrowKeys() {
    if (IsKeyPressed(KEY_RIGHT)) return {1, 0};
    if (IsKeyPressed(KEY_LEFT)) return {-1, 0};
    if (IsKeyPressed(KEY_DOWN)) return {0, 1};
    if (IsKeyPressed(KEY_UP)) return {0, -1};
    return {0, 0};
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

void drawHover(Pos cell) {
    const Vector2 s = cellToScreen(cell);
    DrawRectangleLinesEx({s.x, s.y, kCellSize, kCellSize}, 2.0f, kHoverColor);
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
    // The viewer works on its own copy, so R can always go back to the original.
    Grid grid = scenario.grid;
    Pos robot = scenario.start;
    const Pos goal = scenario.goal;
    int moves = 0;

    const int mapWidth = grid.width() * kCellSize;
    const int width = mapWidth > kMinWidth ? mapWidth : kMinWidth;
    const int height = kHeaderHeight + grid.height() * kCellSize;

    SetConfigFlags(FLAG_MSAA_4X_HINT);  // smoother circles
    SetTraceLogLevel(LOG_WARNING);      // keep raylib's startup messages out of the terminal
    InitWindow(width, height, "robot-sim");
    SetTargetFPS(60);

    while (!WindowShouldClose()) {
        // --- input -------------------------------------------------------
        const std::optional<Pos> hover = screenToCell(GetMousePosition(), grid);
        const bool atGoal = robot == goal;

        const Pos step = readArrowKeys();
        if ((step.x != 0 || step.y != 0) && !atGoal) {
            const Pos next{robot.x + step.x, robot.y + step.y};
            if (grid.at(next) == Cell::Free) {  // walls (and the map edge) block the robot
                robot = next;
                ++moves;
            }
        }

        if (hover && IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && *hover != robot && *hover != goal) {
            grid.set(*hover, grid.at(*hover) == Cell::Wall ? Cell::Free : Cell::Wall);
        }

        if (IsKeyPressed(KEY_R)) {
            grid = scenario.grid;
            robot = scenario.start;
            moves = 0;
        }

        // --- draw --------------------------------------------------------
        BeginDrawing();
        ClearBackground(kHeaderColor);

        if (robot == goal) {
            DrawText(TextFormat("Goal reached in %d moves!   R to reset", moves), 12, 11, 18, kGoalColor);
        } else {
            DrawText(TextFormat("moves %d", moves), 12, 11, 18, RAYWHITE);
            DrawText("arrows: drive   click: wall   R: reset", 120, 11, 18, kHeaderDim);
        }
        if (hover) {
            const char* label = TextFormat("cell (%d, %d)", hover->x, hover->y);
            DrawText(label, width - MeasureText(label, 18) - 12, 11, 18, kHoverColor);
        }

        drawGrid(grid);
        drawGoal(goal);
        drawRobot(robot);
        if (hover) drawHover(*hover);
        EndDrawing();
    }
    CloseWindow();
}
