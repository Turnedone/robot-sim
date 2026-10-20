#include <exception>
#include <iostream>
#include <string>

#include "grid.hpp"
#include "viewer.hpp"

// Prints the map in the terminal: walls, the robot (R) and the goal (G).
void printMap(const Grid& grid, Pos robot, Pos goal) {
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

// Usage: robot_sim [map file] [--text] [--go] [--localize] [--global]
//   --text      prints the map in the terminal instead of opening a window.
//   --go        the robot sets off for the goal by itself straight away.
//   --localize  the robot has a saved map but slipping wheels, and localizes
//               itself with a particle filter.
//   --global    as --localize, but starting with no idea where it is.
int main(int argc, char** argv) {
    std::string mapPath = "maps/warehouse.txt";
    bool textMode = false;
    ViewerOptions options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--text")
            textMode = true;
        else if (arg == "--go")
            options.goToGoal = true;
        else if (arg == "--localize")
            options.localize = true;
        else if (arg == "--global")
            options.localize = options.globalStart = true;
        else
            mapPath = arg;
    }

    try {
        const Scenario s = loadScenario(mapPath);
        std::cout << "Map " << mapPath << ": " << s.grid.width() << "x" << s.grid.height() << ", robot at ("
                  << s.start.x << "," << s.start.y << "), goal at (" << s.goal.x << "," << s.goal.y << ")\n";
        if (textMode) {
            std::cout << '\n';
            printMap(s.grid, s.start, s.goal);
        } else {
            runViewer(s, options);
        }
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << '\n';
        return 1;
    }
    return 0;
}
