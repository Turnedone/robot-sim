#pragma once

#include <string>

#include "grid.hpp"

struct ViewerOptions {
    bool goToGoal = false;  // start driving to the map's goal straight away (like pressing G)
    // Localization mode: the robot has a saved map, its wheels slip, and it
    // must work out where it is with a particle filter (no mapping).
    bool localize = false;
    bool globalStart = false;  // ...starting with no idea where it is
    // SLAM mode: no map AND slipping wheels; the robot maps and localizes at once.
    bool slam = false;
    // Save one frame after this many simulated seconds to `screenshot`, from a
    // hidden window, then quit (for documentation images).
    std::string screenshot;
    double screenshotAfter = 0;
};

// Opens a window and runs the interactive viewer until it is closed:
//   left click  sends the robot to that cell (A* + pure pursuit)
//   G           sends the robot to the map's goal
//   arrow keys  drive the robot by hand (cancels the goal)
//   right click toggles a wall on the clicked cell (mapping mode)
//   K           kidnaps the robot to a random place (localization mode)
//   R           resets the robot and the map to how they were loaded
// This is the only part of the program that knows about graphics: the robot
// logic never calls into it, it only gets drawn by it.
void runViewer(const Scenario& scenario, const ViewerOptions& options = {});