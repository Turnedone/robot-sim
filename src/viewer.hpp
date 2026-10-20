#pragma once

#include "grid.hpp"

// Opens a window and runs the interactive viewer until it is closed:
//   left click  sends the robot to that cell (A* on its own map + pure pursuit)
//   G           sends the robot to the map's goal
//   arrow keys  drive the robot by hand (cancels the goal)
//   right click toggles a wall on the clicked cell
//   R           resets the robot and the map to how they were loaded
// This is the only part of the program that knows about graphics: the robot
// logic never calls into it, it only gets drawn by it.
// goToGoal: start driving to the map's goal straight away (like pressing G).
void runViewer(const Scenario& scenario, bool goToGoal = false);
