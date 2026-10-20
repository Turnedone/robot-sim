#!/usr/bin/env bash
# Sends our navigator a goal on /goal_pose (what RViz's "2D Goal Pose" does).
#   bash goal.sh [x] [y]      (map frame, default: across Nav2's TurtleBot3 world)
source /opt/ros/jazzy/setup.bash
x="${1:-2.0}"
y="${2:--0.5}"
ros2 topic pub --once -w 1 /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: $x, y: $y}, orientation: {w: 1.0}}}"
