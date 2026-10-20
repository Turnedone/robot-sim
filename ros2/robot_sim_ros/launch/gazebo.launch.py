"""Our localizer and navigator driving a TurtleBot3 in Gazebo, instead of Nav2.

    ros2 launch robot_sim_ros gazebo.launch.py

Starts Gazebo Harmonic with Nav2's TurtleBot3 world and robot, the robot model,
a map_server with the world's saved map, and then OUR nodes (no AMCL, no Nav2):
  localizer_node  particle filter on the map_server's map  (AMCL's job)
  navigator_node  A* with inflation + pure pursuit          (Nav2's job)
Use RViz's "2D Goal Pose" to send the robot somewhere.
"""

import os
import subprocess
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Where Nav2's TurtleBot3 demo spawns the robot, in the map frame.
START = {"x": -2.0, "y": -0.5, "yaw": 0.0}


def nodes(context):
    sim_dir = get_package_share_directory("nav2_minimal_tb3_sim")
    nav2_dir = get_package_share_directory("nav2_bringup")
    own_dir = get_package_share_directory("robot_sim_ros")
    gui = LaunchConfiguration("gui").perform(context).lower() == "true"
    sim_time = {"use_sim_time": True}

    # Gazebo needs the world as a plain SDF file; it is generated from xacro.
    world_sdf = tempfile.mktemp(prefix="robot_sim_", suffix=".sdf")
    world_xacro = os.path.join(sim_dir, "worlds", "tb3_sandbox.sdf.xacro")
    with open(os.path.join(sim_dir, "urdf", "turtlebot3_waffle.urdf")) as f:
        robot_description = f.read()

    # Generated before anything starts, so Gazebo never reads a half-written file.
    subprocess.run(["xacro", "-o", world_sdf, f"headless:={'false' if gui else 'true'}", world_xacro], check=True)
    actions = [
        # Where Gazebo finds the TurtleBot3 world's 3D models (model://turtlebot3_world).
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.join(sim_dir, "models")),
        ExecuteProcess(cmd=["gz", "sim", "-r", "-s", world_sdf], output="screen"),
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(
            function=lambda _: os.remove(world_sdf) if os.path.exists(world_sdf) else None)])),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(sim_dir, "launch", "spawn_tb3.launch.py")),
            launch_arguments={"use_sim_time": "True", "robot_name": "turtlebot3_waffle",
                              "robot_sdf": os.path.join(sim_dir, "urdf", "gz_waffle.sdf.xacro"),
                              "x_pose": str(START["x"]), "y_pose": str(START["y"]), "z_pose": "0.01",
                              "roll": "0.0", "pitch": "0.0", "yaw": str(START["yaw"])}.items()),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": robot_description, **sim_time}]),
        # The world's saved map, served the standard way.
        Node(package="nav2_map_server", executable="map_server", name="map_server",
             parameters=[{"yaml_filename": os.path.join(nav2_dir, "maps", "tb3_sandbox.yaml"), **sim_time}]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_map",
             parameters=[{"autostart": True, "node_names": ["map_server"], **sim_time}]),
        # Our nodes.
        Node(package="robot_sim_ros", executable="localizer_node", name="localizer_node", output="screen",
             parameters=[{"map_file": "", "initial_x": START["x"], "initial_y": START["y"],
                          "initial_yaw": START["yaw"], "sensor_x": -0.064, **sim_time}]),
        Node(package="robot_sim_ros", executable="navigator_node", name="navigator_node", output="screen",
             parameters=[{"robot_radius": 0.22, "inflation_radius": 0.25, "clearance_range": 0.3,
                          "smooth_path": True, "max_speed": 0.26, **sim_time}]),
    ]
    if gui:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("ros_gz_sim"), "launch",
                                                       "gz_sim.launch.py")),
            launch_arguments={"gz_args": "-v4 -g "}.items()))
    actions.append(Node(package="rviz2", executable="rviz2", name="rviz2",
                        arguments=["-d", os.path.join(own_dir, "rviz", "gazebo.rviz")],
                        parameters=[sim_time], condition=IfCondition(LaunchConfiguration("rviz"))))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true", description="show the Gazebo window"),
        DeclareLaunchArgument("rviz", default_value="true"),
        OpaqueFunction(function=nodes),
    ])
