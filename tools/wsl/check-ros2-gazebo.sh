#!/usr/bin/env bash
# Checks that the ROS 2 / Gazebo / Nav2 setup from setup-ros2-gazebo.sh is complete.
source /opt/ros/jazzy/setup.bash
echo "ROS_DISTRO=$ROS_DISTRO"
gz sim --versions 2>/dev/null | head -1
for p in ros_gz_sim ros_gz_bridge nav2_bringup nav2_minimal_tb3_sim rviz2 teleop_twist_keyboard; do
    if ros2 pkg prefix "$p" >/dev/null 2>&1; then echo "ok       $p"; else echo "MISSING  $p"; fi
done
rosdep update >/dev/null 2>&1 && echo "rosdep updated"
