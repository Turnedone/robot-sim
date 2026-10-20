"""Mapping: the robot starts with an empty map and builds it from its lidar.

    ros2 launch robot_sim_ros mapping.launch.py
    ros2 launch robot_sim_ros mapping.launch.py map:=maze

Odometry does not slip here and map -> odom is fixed, because this mapper
trusts the pose it is given (mapping only, not SLAM).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Map sizes in cells, as in the map files.
SIZES = {"warehouse": (30, 13), "maze": (26, 13)}


def nodes(context):
    share = get_package_share_directory("robot_sim_ros")
    name = LaunchConfiguration("map").perform(context)
    width, height = SIZES[name]
    return [
        Node(package="robot_sim_ros", executable="sim_node", name="sim_node",
             parameters=[{"map_file": os.path.join(share, "maps", name + ".txt"), "slip": False}]),
        Node(package="tf2_ros", executable="static_transform_publisher", name="map_to_odom",
             arguments=["--frame-id", "map", "--child-frame-id", "odom"]),
        Node(package="robot_sim_ros", executable="mapper_node", name="mapper_node",
             parameters=[{"width": width, "height": height, "resolution": 0.5}]),
        Node(package="robot_sim_ros", executable="navigator_node", name="navigator_node"),
        Node(package="rviz2", executable="rviz2", name="rviz2",
             arguments=["-d", os.path.join(share, "rviz", "robot_sim.rviz")],
             condition=IfCondition(LaunchConfiguration("rviz"))),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("map", default_value="warehouse", description="warehouse or maze"),
        DeclareLaunchArgument("rviz", default_value="true"),
        OpaqueFunction(function=nodes),
    ])
