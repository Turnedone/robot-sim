#!/usr/bin/env bash
# Sends Nav2 a navigation goal (default: across the TurtleBot3 world) and
# waits for the result: the same as RViz's "Nav2 Goal" tool.
#   bash nav2-demo-goal.sh [x] [y]
source /opt/ros/jazzy/setup.bash
x="${1:-2.0}"
y="${2:--0.5}"
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: $x, y: $y}, orientation: {w: 1.0}}}}"
