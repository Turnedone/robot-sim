#include "viewer.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>

#include <raylib.h>

#include "lidar.hpp"
#include "navigator.hpp"
#include "occupancy_map.hpp"
#include "robot.hpp"

namespace {

constexpr int kCellSize = 32;      // pixels per map cell
constexpr int kHeaderHeight = 80;  // strip at the top for text (three lines)
constexpr int kMinWidth = 760;     // so the header text always fits

constexpr double kPhysicsDt = 0.02;     // simulate at 50 Hz, a typical robot control rate
constexpr double kTeleopLinear = 0.5;   // m/s while up/down is held
constexpr double kTeleopAngular = 2.0;  // rad/s while left/right is held
constexpr double kTrailSpacing = 0.05;  // m between trail points
constexpr size_t kTrailMax = 4000;
constexpr double kBumpFlash = 0.3;  // s the robot shows red after a bump

const Color kFreeColor{235, 235, 230, 255};
const Color kWallColor{60, 63, 70, 255};
const Color kGridLineColor{210, 210, 205, 255};
const Color kHoverColor{255, 200, 60, 255};
const Color kRobotColor{40, 110, 220, 255};
const Color kBumpColor{220, 60, 50, 255};
const Color kWheelColor{25, 25, 30, 255};
const Color kTrailColor{40, 110, 220, 110};
const Color kGoalColor{40, 170, 90, 255};
const Color kRayColor{230, 60, 50, 45};
const Color kHitColor{230, 50, 40, 255};
const Color kHiddenMapColor{18, 19, 22, 255};
const Color kUnknownColor{112, 120, 132, 255};  // as in RViz: unknown grey, free white, wall black
const Color kOccupiedColor{22, 24, 28, 255};
const Color kWrongColor{230, 40, 40, 255};
const Color kPathColor{240, 150, 30, 255};
const Color kTargetColor{240, 150, 30, 255};
const Color kHeaderColor{30, 32, 36, 255};
const Color kHeaderDim{150, 150, 150, 255};

float pixelsPerMeter(const Grid& grid) {
    return static_cast<float>(kCellSize / grid.resolution());
}

// Top-left pixel of a map cell on screen.
Vector2 cellToScreen(Pos p) {
    return {static_cast<float>(p.x * kCellSize), static_cast<float>(kHeaderHeight + p.y * kCellSize)};
}

// Map frame (meters, y up) -> screen pixels (y down).
Vector2 worldToScreen(Point w, const Grid& grid) {
    const float ppm = pixelsPerMeter(grid);
    const double mapHeight = grid.height() * grid.resolution();
    return {static_cast<float>(w.x * ppm), static_cast<float>(kHeaderHeight + (mapHeight - w.y) * ppm)};
}

// Which cell a pixel is in, or nothing if it is outside the map.
std::optional<Pos> screenToCell(Vector2 pixel, const Grid& grid) {
    if (pixel.y < kHeaderHeight) return std::nullopt;
    const Pos p{static_cast<int>(pixel.x) / kCellSize, (static_cast<int>(pixel.y) - kHeaderHeight) / kCellSize};
    if (!grid.inBounds(p)) return std::nullopt;
    return p;
}

bool anyArrowKeyDown() {
    return IsKeyDown(KEY_UP) || IsKeyDown(KEY_DOWN) || IsKeyDown(KEY_LEFT) || IsKeyDown(KEY_RIGHT);
}

const char* statusText(NavStatus s) {
    switch (s) {
        case NavStatus::Idle: return "manual";
        case NavStatus::Driving: return "driving to goal";
        case NavStatus::Arrived: return "arrived";
        case NavStatus::NoPath: return "NO PATH (waiting for the map to change)";
    }
    return "";
}

void drawPlan(const Navigator& nav, const Grid& grid) {
    const std::vector<Point>& path = nav.path();
    for (size_t i = 1; i < path.size(); ++i)
        DrawLineEx(worldToScreen(path[i - 1], grid), worldToScreen(path[i], grid), 3.0f, kPathColor);
    if (const std::optional<Pos> goal = nav.goal()) {
        const Vector2 s = cellToScreen(*goal);
        DrawRectangleLinesEx({s.x + 3, s.y + 3, kCellSize - 6, kCellSize - 6}, 3.0f, kTargetColor);
    }
    if (const std::optional<Point> aim = nav.lookaheadPoint())
        DrawCircleLinesV(worldToScreen(*aim, grid), 6.0f, kTargetColor);
}

// Held arrow keys -> a velocity command, like ROS's teleop_twist_keyboard.
Twist readTeleop() {
    Twist cmd;
    if (IsKeyDown(KEY_UP)) cmd.linear += kTeleopLinear;
    if (IsKeyDown(KEY_DOWN)) cmd.linear -= kTeleopLinear;
    if (IsKeyDown(KEY_LEFT)) cmd.angular += kTeleopAngular;  // counter-clockwise
    if (IsKeyDown(KEY_RIGHT)) cmd.angular -= kTeleopAngular;
    return cmd;
}

Pose startPose(const Scenario& s) {
    const Point c = s.grid.cellCenter(s.start);
    return {c.x, c.y, 0.0};
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
    DrawRectangle(static_cast<int>(s.x) + pad, static_cast<int>(s.y) + pad, kCellSize - 2 * pad, kCellSize - 2 * pad,
                  kGoalColor);
}

// What the screen shows under the robot (M cycles through them).
enum class View : std::uint8_t { RobotMap, TrueMap, LidarOnly };

const char* viewName(View v) {
    switch (v) {
        case View::RobotMap: return "ROBOT'S MAP (built from lidar)";
        case View::TrueMap: return "TRUE MAP (what is really there)";
        case View::LidarOnly: return "LIDAR ONLY (no memory)";
    }
    return "";
}

Color mix(Color a, Color b, float t) {
    const auto ch = [t](unsigned char x, unsigned char y) { return static_cast<unsigned char>(x + (y - x) * t); };
    return {ch(a.r, b.r), ch(a.g, b.g), ch(a.b, b.b), 255};
}

// The robot's belief: never-seen cells grey, the rest shaded from light
// (sure it is free) to dark (sure it is a wall). Cells the robot is sure about
// but has WRONG are outlined in red.
void drawOccupancy(const OccupancyMap& map, const Grid& truth) {
    for (int y = 0; y < map.height(); ++y) {
        for (int x = 0; x < map.width(); ++x) {
            const Vector2 s = cellToScreen({x, y});
            const Color c = map.observed({x, y})
                                ? mix(kFreeColor, kOccupiedColor, static_cast<float>(map.probability({x, y})))
                                : kUnknownColor;
            DrawRectangle(static_cast<int>(s.x), static_cast<int>(s.y), kCellSize, kCellSize, c);
            DrawRectangleLines(static_cast<int>(s.x), static_cast<int>(s.y), kCellSize, kCellSize,
                               mix(c, kGridLineColor, 0.25f));
            const Occupancy o = map.classify({x, y});
            const bool isWall = truth.at({x, y}) == Cell::Wall;
            if (o != Occupancy::Unknown && (o == Occupancy::Occupied) != isWall)
                DrawRectangleLinesEx({s.x + 2, s.y + 2, kCellSize - 4, kCellSize - 4}, 2.0f, kWrongColor);
        }
    }
}

// Draws a scan from the pose it was taken at: a faint ray per beam and a dot
// where it hit something. Beams with no return are left out.
void drawScan(const LaserScan& scan, const Pose& from, const Grid& grid, bool showRays) {
    const Vector2 origin = worldToScreen({from.x, from.y}, grid);
    for (size_t i = 0; i < scan.ranges.size(); ++i) {
        const double r = scan.ranges[i];
        if (!std::isfinite(r)) continue;
        const double a = from.theta + scan.angle(i);
        const Vector2 hit = worldToScreen({from.x + r * std::cos(a), from.y + r * std::sin(a)}, grid);
        if (showRays) DrawLineV(origin, hit, kRayColor);
        DrawCircleV(hit, 2.5f, kHitColor);
    }
}

void drawTrail(const std::vector<Point>& trail, const Grid& grid) {
    for (size_t i = 1; i < trail.size(); ++i)
        DrawLineEx(worldToScreen(trail[i - 1], grid), worldToScreen(trail[i], grid), 2.0f, kTrailColor);
}

void drawRobot(const DiffDriveRobot& robot, const Grid& grid, bool bumped) {
    const Pose& p = robot.pose();
    const DiffDriveConfig& cfg = robot.config();
    const float ppm = pixelsPerMeter(grid);
    const Vector2 centre = worldToScreen({p.x, p.y}, grid);

    DrawCircleV(centre, static_cast<float>(cfg.radius * ppm), bumped ? kBumpColor : kRobotColor);

    // The two wheels sit half a wheel base to the left and right of the centre.
    const float wheelLength = static_cast<float>(0.12 * ppm);
    const float wheelWidth = static_cast<float>(0.045 * ppm);
    const float rotationDeg = static_cast<float>(-p.theta * RAD2DEG);  // screen y is flipped
    for (int side : {-1, 1}) {
        const double offset = side * cfg.wheelBase / 2;
        const Point wheel{p.x - offset * std::sin(p.theta), p.y + offset * std::cos(p.theta)};
        const Vector2 w = worldToScreen(wheel, grid);
        DrawRectanglePro({w.x, w.y, wheelLength, wheelWidth}, {wheelLength / 2, wheelWidth / 2}, rotationDeg,
                         kWheelColor);
    }

    // Heading: a line from the centre to the front edge.
    const Point nose{p.x + cfg.radius * std::cos(p.theta), p.y + cfg.radius * std::sin(p.theta)};
    DrawLineEx(centre, worldToScreen(nose, grid), 3.0f, RAYWHITE);
}

}  // namespace

void runViewer(const Scenario& scenario, bool goToGoal) {
    // The viewer works on its own copies, so R can always go back to the original.
    Grid grid = scenario.grid;
    const DiffDriveConfig config;
    DiffDriveRobot robot(config, startPose(scenario));
    std::vector<Point> trail{{robot.pose().x, robot.pose().y}};
    double simTime = 0;
    double accumulator = 0;
    double bumpTimer = 0;
    std::optional<double> goalTime;

    Lidar lidar{LidarConfig{}};
    const double scanPeriod = 1.0 / lidar.config().rateHz;
    double scanTimer = 0;
    LaserScan lastScan = lidar.scan(grid, robot.pose());
    Pose lastScanPose = robot.pose();
    bool showRays = true;
    View view = View::RobotMap;

    OccupancyMap map(grid.width(), grid.height(), grid.resolution());
    map.integrate(lastScan, lastScanPose);
    NavigatorConfig navConfig;
    navConfig.robotRadius = config.radius;
    Navigator nav(navConfig);
    if (goToGoal) {
        nav.setGoal(scenario.goal);
        nav.replan(map, robot.pose());
    }

    const int mapWidth = grid.width() * kCellSize;
    const int width = std::max(mapWidth, kMinWidth);
    const int height = kHeaderHeight + grid.height() * kCellSize;

    SetConfigFlags(FLAG_MSAA_4X_HINT);  // smoother circles
    SetTraceLogLevel(LOG_WARNING);      // keep raylib's startup messages out of the terminal
    InitWindow(width, height, "robot-sim");
    SetTargetFPS(60);

    while (!WindowShouldClose()) {
        // --- input -------------------------------------------------------
        const std::optional<Pos> hover = screenToCell(GetMousePosition(), grid);
        // Arrow keys always win (manual driving) and cancel any goal.
        if (anyArrowKeyDown() && nav.status() != NavStatus::Idle) nav.cancel();
        if (hover && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            nav.setGoal(*hover);
            nav.replan(map, robot.pose());
        }
        if (IsKeyPressed(KEY_G)) {
            nav.setGoal(scenario.goal);
            nav.replan(map, robot.pose());
        }

        if (hover && IsMouseButtonPressed(MOUSE_BUTTON_RIGHT) && *hover != scenario.goal) {
            grid.set(*hover, grid.at(*hover) == Cell::Wall ? Cell::Free : Cell::Wall);
            // Never drop a wall on top of the robot.
            if (collides(grid, {robot.pose().x, robot.pose().y}, config.radius)) grid.set(*hover, Cell::Free);
        }

        if (IsKeyPressed(KEY_L)) showRays = !showRays;
        if (IsKeyPressed(KEY_M)) view = static_cast<View>((static_cast<int>(view) + 1) % 3);

        if (IsKeyPressed(KEY_R)) {
            grid = scenario.grid;
            robot = DiffDriveRobot(config, startPose(scenario));
            trail.assign(1, {robot.pose().x, robot.pose().y});
            simTime = 0;
            accumulator = 0;
            goalTime.reset();
            scanTimer = 0;
            lastScan = lidar.scan(grid, robot.pose());
            lastScanPose = robot.pose();
            map = OccupancyMap(grid.width(), grid.height(), grid.resolution());
            map.integrate(lastScan, lastScanPose);
            nav = Navigator(navConfig);
        }

        // --- simulate ----------------------------------------------------
        // Physics runs in fixed 20 ms steps whatever the frame rate, so the
        // robot behaves the same on a fast or a slow PC.
        accumulator += std::min(GetFrameTime(), 0.25f);
        while (accumulator >= kPhysicsDt) {
            accumulator -= kPhysicsDt;
            simTime += kPhysicsDt;
            // The controller runs at the physics rate (50 Hz), like a real control loop.
            robot.setCommand(nav.status() == NavStatus::Idle ? readTeleop() : nav.command(robot.pose(), map));
            if (!robot.step(kPhysicsDt, grid)) bumpTimer = kBumpFlash;

            const Point here{robot.pose().x, robot.pose().y};
            if (std::hypot(here.x - trail.back().x, here.y - trail.back().y) > kTrailSpacing) {
                trail.push_back(here);
                if (trail.size() > kTrailMax) trail.erase(trail.begin());
            }
            if (!goalTime && grid.worldToCell(here) == scenario.goal) goalTime = simTime;

            // The lidar spins at its own rate (5 Hz), slower than the physics.
            scanTimer += kPhysicsDt;
            if (scanTimer >= scanPeriod) {
                scanTimer -= scanPeriod;
                lastScan = lidar.scan(grid, robot.pose());
                lastScanPose = robot.pose();
                map.integrate(lastScan, lastScanPose);  // the robot learns from every scan
                nav.replan(map, robot.pose());          // ...and re-plans on what it learned
            }
        }
        bumpTimer = std::max(0.0, bumpTimer - GetFrameTime());

        // --- draw --------------------------------------------------------
        BeginDrawing();
        ClearBackground(kHeaderColor);

        if (goalTime) {
            DrawText(
                TextFormat("Goal reached in %.1f s, %.1f m driven.   R to reset", *goalTime, robot.distanceTravelled()),
                12, 8, 18, kGoalColor);
        } else {
            DrawText("click: go  G: goal  arrows: drive  right-click: wall  M: view  L: rays  R: reset", 12, 8, 16,
                     kHeaderDim);
        }
        if (hover) {
            const char* label = TextFormat("cell (%d, %d)", hover->x, hover->y);
            DrawText(label, width - MeasureText(label, 18) - 12, 8, 18, kHoverColor);
        }

        const Pose& p = robot.pose();
        const Twist v = robot.velocity();
        DrawText(TextFormat("x %5.2f  y %5.2f m  heading %4.0f deg   v %5.2f m/s  w %5.2f rad/s   "
                            "wheels L %5.2f  R %5.2f m/s   %.1f m",
                            p.x, p.y, p.theta * RAD2DEG, v.linear, v.angular, robot.leftSpeed(), robot.rightSpeed(),
                            robot.distanceTravelled()),
                 12, 32, 16, RAYWHITE);

        // How good is the robot's map? Explored = cells it is sure about;
        // wrong = cells it is sure about but has the opposite of reality.
        int explored = 0;
        int wrong = 0;
        for (int y = 0; y < grid.height(); ++y) {
            for (int x = 0; x < grid.width(); ++x) {
                const Occupancy o = map.classify({x, y});
                if (o == Occupancy::Unknown) continue;
                ++explored;
                if ((o == Occupancy::Occupied) != (grid.at({x, y}) == Cell::Wall)) ++wrong;
            }
        }
        DrawText(TextFormat("view: %s   explored %d%%   wrong cells %d   |   %s", viewName(view),
                            100 * explored / (grid.width() * grid.height()), wrong, statusText(nav.status())),
                 12, 56, 16, wrong > 0 ? kWrongColor : kHoverColor);

        switch (view) {
            case View::RobotMap: drawOccupancy(map, grid); break;
            case View::TrueMap: drawGrid(grid); break;
            case View::LidarOnly:
                DrawRectangle(0, kHeaderHeight, width, height - kHeaderHeight, kHiddenMapColor);
                break;
        }
        drawGoal(scenario.goal);
        drawTrail(trail, grid);
        drawScan(lastScan, lastScanPose, grid, showRays);
        drawPlan(nav, grid);
        drawRobot(robot, grid, bumpTimer > 0);
        if (hover) drawHover(*hover);
        EndDrawing();
    }
    CloseWindow();
}
