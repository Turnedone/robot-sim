#include <exception>
#include <iostream>
#include <string>

#include "grid.hpp"

// Draws the map in the terminal: walls, the robot (R) and the goal (G).
void draw(const Grid& grid, Pos robot, Pos goal) {
    for (int y = 0; y < grid.height(); ++y) {
        for (int x = 0; x < grid.width(); ++x) {
            const Pos p{x, y};
            if (p == robot)
                std::cout << 'R';
            else if (p == goal)
                std::cout << 'G';
            else
                std::cout << (grid.at(p) == Cell::Wall ? '#' : '.');
        }
        std::cout << '\n';
    }
}

int main(int argc, char** argv) {
    const std::string mapPath = argc > 1 ? argv[1] : "maps/warehouse.txt";

    try {
        const Scenario s = loadScenario(mapPath);
        std::cout << "Map " << mapPath << ": " << s.grid.width() << "x" << s.grid.height()
                  << ", robot at (" << s.start.x << "," << s.start.y << "), goal at ("
                  << s.goal.x << "," << s.goal.y << ")\n\n";
        draw(s.grid, s.start, s.goal);
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << '\n';
        return 1;
    }
    return 0;
}
