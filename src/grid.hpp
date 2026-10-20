#pragma once

#include <string>
#include <vector>

// A cell position on the map. x = column, y = row (row 0 is the top).
struct Pos {
    int x = 0;
    int y = 0;
};

inline bool operator==(Pos a, Pos b) { return a.x == b.x && a.y == b.y; }
inline bool operator!=(Pos a, Pos b) { return !(a == b); }

enum class Cell { Free, Wall };

// The real world: which cells are walls. The robot will NOT get to read this
// directly later on; it has to discover walls with its sensor.
class Grid {
public:
    Grid(int width, int height);

    int width() const { return width_; }
    int height() const { return height_; }

    bool inBounds(Pos p) const;
    Cell at(Pos p) const;
    void set(Pos p, Cell c);

private:
    int width_;
    int height_;
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
