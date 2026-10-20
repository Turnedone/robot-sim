"""Localization: the robot has a saved map but slipping wheels.

    ros2 launch robot_sim_ros localization.launch.py
    ros2 launch robot_sim_ros localization.launch.py map:=maze global:=true

Then in RViz use "2D Goal Pose" to send the robot somewhere, or
"2D Pose Estimate" to tell the localizer where the robot is.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    share = get_package_share_directory("robot_sim_ros")
    map_file = os.path.join(share, "maps", LaunchConfiguration("map").perform(context) + ".txt")
    global_start = LaunchConfiguration("global").perform(context).lower() == "true"
    return [
        Node(package="robot_sim_ros", executable="sim_node", name="sim_node",
             parameters=[{"map_file": map_file, "slip": True}]),
        Node(package="robot_sim_ros", executable="localizer_node", name="localizer_node",
             parameters=[{"map_file": map_file, "global": global_start}]),
        Node(package="robot_sim_ros", executable="navigator_node", name="navigator_node"),
        Node(package="rviz2", executable="rviz2", name="rviz2",
             arguments=["-d", os.path.join(share, "rviz", "robot_sim.rviz")],
             condition=IfCondition(LaunchConfiguration("rviz"))),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("map", default_value="warehouse", description="warehouse or maze"),
        DeclareLaunchArgument("global", default_value="false", description="start with no idea where the robot is"),
        DeclareLaunchArgument("rviz", default_value="true"),
        OpaqueFunction(function=nodes),
    ])
