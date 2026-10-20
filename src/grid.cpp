#include "grid.hpp"

#include <cmath>
#include <fstream>
#include <stdexcept>

Grid::Grid(int width, int height, double resolution)
    : width_(width),
      height_(height),
      resolution_(resolution),
      cells_(static_cast<size_t>(width * height), Cell::Free) {}

Point cellCenter(Pos p, int mapHeight, double resolution) {
    // Row 0 is the TOP of the map, but y grows UP, hence the flip.
    return {(p.x + 0.5) * resolution, (mapHeight - p.y - 0.5) * resolution};
}

Pos worldToCell(Point w, int mapHeight, double resolution) {
    const int col = static_cast<int>(std::floor(w.x / resolution));
    const int rowFromBottom = static_cast<int>(std::floor(w.y / resolution));
    return {col, mapHeight - 1 - rowFromBottom};
}

bool Grid::inBounds(Pos p) const {
    return p.x >= 0 && p.y >= 0 && p.x < width_ && p.y < height_;
}

Cell Grid::at(Pos p) const {
    // Outside the map counts as wall, so the robot can never drive off the edge.
    if (!inBounds(p)) return Cell::Wall;
    return cells_[static_cast<size_t>(p.y * width_ + p.x)];
}

void Grid::set(Pos p, Cell c) {
    if (!inBounds(p)) throw std::out_of_range("Grid::set outside the map");
    cells_[static_cast<size_t>(p.y * width_ + p.x)] = c;
}

Scenario loadScenario(const std::string& path) {
    std::ifstream file(path);
    if (!file) throw std::runtime_error("cannot open map file: " + path);

    std::vector<std::string> lines;
    for (std::string line; std::getline(file, line);) {
        if (!line.empty() && line.back() == '\r') line.pop_back();  // Windows line endings
        if (!line.empty()) lines.push_back(line);
    }
    if (lines.empty()) throw std::runtime_error("map file is empty: " + path);

    const int width = static_cast<int>(lines[0].size());
    const int height = static_cast<int>(lines.size());
    Grid grid(width, height);
    Pos start{-1, -1};
    Pos goal{-1, -1};

    for (int y = 0; y < height; ++y) {
        if (static_cast<int>(lines[y].size()) != width)
            throw std::runtime_error("map rows must all be the same width (row " + std::to_string(y) + ")");
        for (int x = 0; x < width; ++x) {
            switch (lines[y][x]) {
                case '#': grid.set({x, y}, Cell::Wall); break;
                case '.': break;
                case 'S': start = {x, y}; break;
                case 'G': goal = {x, y}; break;
                default:
                    throw std::runtime_error(std::string("unknown map character '") + lines[y][x] + "'");
            }
        }
    }
    if (start.x < 0) throw std::runtime_error("map has no start 'S'");
    if (goal.x < 0) throw std::runtime_error("map has no goal 'G'");

    return {grid, start, goal};
}
