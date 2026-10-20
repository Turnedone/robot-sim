#pragma once

#include "grid.hpp"

// Opens a window and runs the interactive viewer until it is closed:
//   arrow keys  drive the robot (hold: up/down = forward/back, left/right = turn)
//   left click  toggles a wall on the clicked cell
//   R           resets the robot and the map to how they were loaded
// This is the only part of the program that knows about graphics: the robot
// logic never calls into it, it only gets drawn by it.
void runViewer(const Scenario& scenario);
