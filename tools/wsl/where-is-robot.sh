#!/usr/bin/env bash
# Prints where the TurtleBot3 REALLY is (Gazebo's ground truth) and where our
# localizer thinks it is (TF map -> base_link), both in the map frame.
source /opt/ros/jazzy/setup.bash
echo "Gazebo ground truth:"
gz model -m turtlebot3_waffle -p 2>/dev/null | sed -n '/Pose/,+3p' | sed -n '1,3p'
echo "Our localizer's estimate (TF map -> base_link):"
timeout 8 ros2 run tf2_ros tf2_echo map base_link --ros-args -p use_sim_time:=true 2>/dev/null | grep -m1 "Translation"
