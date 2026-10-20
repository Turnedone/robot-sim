#!/usr/bin/env bash
# Stops anything left over from a Gazebo / ROS 2 run (Gazebo, RViz, Nav2, our nodes).
for pattern in "gz sim" "ruby.*gz" rviz2 component_container parameter_bridge robot_state_publisher \
               map_server lifecycle_manager localizer_node navigator_node; do
    pkill -f "$pattern" 2>/dev/null
done
sleep 1
echo "stopped"
