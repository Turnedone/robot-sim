#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

// A cell position on the map. x = column, y = row (row 0 is the top).
struct Pos {
    int x = 0;
    int y = 0;
};

inline bool operator==(Pos a, Pos b) {
    return a.x == b.x && a.y == b.y;
}
inline bool operator!=(Pos a, Pos b) {
    return !(a == b);
}

// A point in the world, in meters, in the "map frame": origin at the
// bottom-left corner of the map, x to the right, y UP. That is the convention
// ROS uses (REP 103), so it is the opposite of screen rows, which go down.
struct Point {
    double x = 0;
    double y = 0;
};

// Converting between cells and meters for a map `mapHeight` cells tall with
// square cells of `resolution` meters. Row 0 is the TOP, but y grows UP.
Point cellCenter(Pos p, int mapHeight, double resolution);
Pos worldToCell(Point w, int mapHeight, double resolution);

// Row-major index of a cell in a map `width` cells wide, and the number of
// cells in a map. Converted to size_t BEFORE multiplying, so a large map
// cannot overflow int first.
inline size_t cellIndex(Pos p, int width) {
    return static_cast<size_t>(p.y) * static_cast<size_t>(width) + static_cast<size_t>(p.x);
}
inline size_t cellCount(int width, int height) {
    return static_cast<size_t>(width) * static_cast<size_t>(height);
}

enum class Cell : std::uint8_t { Free, Wall };

// The real world: which cells are walls. The robot will NOT get to read this
// directly later on; it has to discover walls with its sensor.
class Grid {
public:
    // resolution = size of one cell in meters (ROS maps call it the same).
    Grid(int width, int height, double resolution = 0.5);

    int width() const { return width_; }
    int height() const { return height_; }
    double resolution() const { return resolution_; }

    bool inBounds(Pos p) const;
    Cell at(Pos p) const;
    void set(Pos p, Cell c);

    // Converting between cells and meters.
    Point cellCenter(Pos p) const { return ::cellCenter(p, height_, resolution_); }
    Pos worldToCell(Point w) const { return ::worldToCell(w, height_, resolution_); }

private:
    int width_;
    int height_;
    double resolution_;
    std::vector<Cell> cells_;  // row by row: index = y * width + x
};

// Everything read from a map file.
struct Scenario {
    Grid grid;
    Pos start;
    Pos goal;
};

// Map file format: '#' wall, '.' free, 'S' robot start, 'G' goal.
// Throws std::runtime_error if the file is missing or malformed.
Scenario loadScenario(const std::string& path);
