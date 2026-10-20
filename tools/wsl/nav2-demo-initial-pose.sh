#!/usr/bin/env bash
# Tells AMCL where the TurtleBot3 starts in Nav2's tb3_simulation_launch.py demo
# (it spawns at x = -2.0, y = -0.5, facing +x): the same message RViz's
# "2D Pose Estimate" tool sends.
source /opt/ros/jazzy/setup.bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: -2.0, y: -0.5, z: 0.0}, orientation: {w: 1.0}},
    covariance: [0.25, 0, 0, 0, 0, 0, 0, 0.25, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.068]}}"
