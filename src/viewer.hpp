#pragma once

#include "grid.hpp"

// Opens a window and shows the scenario until the window is closed.
// This is the only part of the program that knows about graphics: the robot
// logic never calls into it, it only gets drawn by it.
void runViewer(const Scenario& scenario);
